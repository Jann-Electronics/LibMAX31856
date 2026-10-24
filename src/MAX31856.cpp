#include "MAX31856.h"

#include <math.h>

namespace MAX31856 {

/* ---------------------------------------------------------------------------------------------- */
/*                                       Register addresses                                       */
/* ---------------------------------------------------------------------------------------------- */
namespace Reg {
constexpr uint8_t CR0    = 0x00; // Config 0
constexpr uint8_t CR1    = 0x01; // Config 1
constexpr uint8_t MASK   = 0x02; // Fault Mask
constexpr uint8_t CJHF   = 0x03; // Cold-Junction High Fault threshold
constexpr uint8_t CJLF   = 0x04; // Cold-Junction Low Fault threshold
constexpr uint8_t LTHFTH = 0x05; // Linearized TC High Fault threshold MSB
constexpr uint8_t LTHFTL = 0x06; // Linearized TC High Fault threshold LSB
constexpr uint8_t LTLFTH = 0x07; // Linearized TC Low Fault threshold MSB
constexpr uint8_t LTLFTL = 0x08; // Linearized TC Low Fault threshold LSB
constexpr uint8_t CJTO   = 0x09; // Cold-Junction Temperature Offset
constexpr uint8_t CJTH   = 0x0A; // Cold-Junction Temperature MSB
constexpr uint8_t CJTL   = 0x0B; // Cold-Junction Temperature LSB
constexpr uint8_t LTCBH  = 0x0C; // Linearized TC Temperature, byte 2
constexpr uint8_t SR     = 0x0F; // Fault Status
} // namespace Reg

namespace CR0Bit {
constexpr uint8_t AUTOCONVERT   = 0x80;
constexpr uint8_t ONESHOT       = 0x40; // self-clears when the conversion is done
constexpr uint8_t OCFAULT_SHIFT = 4;    // open-circuit detection, bits 5:4
constexpr uint8_t CJ_DISABLE    = 0x08;
constexpr uint8_t FAULT_INT     = 0x04; // interrupt fault mode
constexpr uint8_t FAULTCLR      = 0x02; // self-clears
constexpr uint8_t NOISE_50HZ    = 0x01;
} // namespace CR0Bit

namespace CR1Bit {
constexpr uint8_t AVGSEL_SHIFT = 4; // averaging, bits 6:4
constexpr uint8_t TYPE_MASK    = 0x0F;
} // namespace CR1Bit

namespace {
constexpr uint32_t SPI_MAX_FREQ_HZ  = 5000000;
constexpr uint32_t POLL_INTERVAL_MS = 5;

// Worst-case conversion times from the datasheet (ms)
constexpr uint32_t TCONV_MAX_50HZ_MS  = 185;
constexpr uint32_t TCONV_MAX_60HZ_MS  = 155;
constexpr uint32_t AVG_SAMPLE_50HZ_MS = 40;
constexpr uint32_t AVG_SAMPLE_60HZ_MS = 34;
constexpr uint8_t OC_TEST_CJ_ON_MS[]  = {0, 15, 37, 125}; // by OpenCircuitDetection
constexpr uint8_t OC_TEST_CJ_OFF_MS[] = {0, 44, 66, 154};
// Longest possible conversion (16 samples, 50 Hz, slowest open-circuit test) plus margin
constexpr uint32_t MAX_CONVERSION_TIMEOUT_MS = 1200;

// Chip defaults for the fault thresholds
constexpr int16_t PROBE_HIGH_DEFAULT = 0x7FFF;
constexpr int16_t PROBE_LOW_DEFAULT  = -0x8000;
constexpr int8_t CJ_HIGH_DEFAULT     = 127;
constexpr int8_t CJ_LOW_DEFAULT      = -64;

constexpr float TC_LSB_C        = 0.0078125f; // 2^-7 °C
constexpr float CJ_LSB_C        = 0.015625f;  // 2^-6 °C
constexpr float THRESHOLD_LSB_C = 0.0625f;    // 2^-4 °C, also used by CJTO

// Voltage mode transfer function (datasheet, CR1 register): Code = gain x 1.6 x 2^17 x VIN
constexpr float VOLTAGE_SCALE = 1.6f * 131072.0f;

constexpr bool isVoltageType(Type type) { return (static_cast<uint8_t>(type) & 0x08) != 0; }

// Rounds value / lsb to the nearest integer, clamped to [min, max]
int32_t toFixed(float value, float lsb, int32_t min, int32_t max) {
  const float scaled = roundf(value / lsb);
  if (scaled <= static_cast<float>(min)) return min;
  if (scaled >= static_cast<float>(max)) return max;
  return static_cast<int32_t>(scaled);
}

// 19-bit two's complement value, left-aligned in LTCBH:LTCBM:LTCBL
int32_t decodeLinearized(const uint8_t* b) {
  int32_t code =
    (static_cast<int32_t>(b[0]) << 11) | (static_cast<int32_t>(b[1]) << 3) | (b[2] >> 5);
  if (code & 0x40000) code -= 0x80000;
  return code;
}

// 14-bit two's complement value, left-aligned in CJTH:CJTL
float decodeColdJunction(const uint8_t* b) {
  int32_t code = (static_cast<int32_t>(b[0]) << 6) | (b[1] >> 2);
  if (code & 0x2000) code -= 0x4000;
  return code * CJ_LSB_C;
}
} // namespace

/* ---------------------------------------------------------------------------------------------- */
/*                                          toString()                                            */
/* ---------------------------------------------------------------------------------------------- */
const char* toString(Type type) {
  switch (type) {
    case Type::B: return "B";
    case Type::E: return "E";
    case Type::J: return "J";
    case Type::K: return "K";
    case Type::N: return "N";
    case Type::R: return "R";
    case Type::S: return "S";
    case Type::T: return "T";
    case Type::VoltageGain8: return "Voltage x8";
    case Type::VoltageGain32: return "Voltage x32";
    default: return "Unknown";
  }
}

const char* toString(Fault flag) {
  switch (flag) {
    case Fault::None: return "None";
    case Fault::Open: return "Open";
    case Fault::OverUnder: return "OverUnder";
    case Fault::TCLow: return "TCLow";
    case Fault::TCHigh: return "TCHigh";
    case Fault::CJLow: return "CJLow";
    case Fault::CJHigh: return "CJHigh";
    case Fault::TCRange: return "TCRange";
    case Fault::CJRange: return "CJRange";
    default: return "Multiple";
  }
}

void toString(Fault f, char* buffer, size_t buffer_size) {
  if (buffer == nullptr || buffer_size == 0) return;

  if (!any(f)) {
    strncpy(buffer, "None", buffer_size);
    buffer[buffer_size - 1] = '\0';
    return;
  }

  static constexpr Fault flags[] = {Fault::Open,
    Fault::OverUnder,
    Fault::TCLow,
    Fault::TCHigh,
    Fault::CJLow,
    Fault::CJHigh,
    Fault::TCRange,
    Fault::CJRange};

  buffer[0]      = '\0';
  size_t written = 0;
  bool first     = true;

  for (Fault flag : flags) {
    if (!has(f, flag)) continue;

    const char* sep  = first ? "" : " | ";
    const char* name = toString(flag);
    int n            = snprintf(buffer + written, buffer_size - written, "%s%s", sep, name);

    if (n <= 0) break;
    if (static_cast<size_t>(n) >= buffer_size - written) {
      buffer[written] = '\0'; // truncated: drop the partial name
      break;
    }

    written += static_cast<size_t>(n);
    first = false;
  }
}

/* ---------------------------------------------------------------------------------------------- */
/*                                          MAX31856                                              */
/* ---------------------------------------------------------------------------------------------- */
Driver::Driver(int8_t cs_pin, SPIClass& spi, uint32_t spi_freq_hz)
    : _cs_pin(cs_pin)
    , _drdy_pin(-1)
    , _spi(&spi)
    , _spi_settings(spi_freq_hz > SPI_MAX_FREQ_HZ ? SPI_MAX_FREQ_HZ : spi_freq_hz, MSBFIRST,
        SPI_MODE1)
    , _type(Type::K)
    , _conversion_mode(ConversionMode::OneShot)
    , _noise_filter(NoiseFilter::Hz50)
    , _averaging(Averaging::Samples1)
    , _open_circuit(OpenCircuitDetection::Below5k)
    , _fault_mode(FaultMode::Comparator)
    , _fault_output(Fault::All)
    , _cj_sensor_enabled(true)
    , _probe_low_raw(PROBE_LOW_DEFAULT)
    , _probe_high_raw(PROBE_HIGH_DEFAULT)
    , _cj_low(CJ_LOW_DEFAULT)
    , _cj_high(CJ_HIGH_DEFAULT)
    , _cj_offset_raw(0)
    , _initialized(false)
    , _has_result(false)
    , _conversion_start_ms(0)
    , _active_conversion_ms(0) {}

bool Driver::begin() {
  _initialized = false;

  // The chip may keep settings from a previous run (MCU reset without power cycle)
  _active_conversion_ms = MAX_CONVERSION_TIMEOUT_MS;

  pinMode(_cs_pin, OUTPUT);
  digitalWrite(_cs_pin, HIGH);
  if (_drdy_pin >= 0) pinMode(_drdy_pin, INPUT_PULLUP);

  if (!_checkCommunication()) return false;

  _initialized = true;
  _writeLimits();
  _reconfigure();
  return true;
}

/* ----------------------------------------- Configuration -------------------------------------- */
void Driver::setType(Type type) {
  _type = type;
  _reconfigure();
}

Type Driver::getType() const { return _type; }

void Driver::setConversionMode(ConversionMode mode) {
  _conversion_mode = mode;
  _reconfigure();
}

ConversionMode Driver::getConversionMode() const { return _conversion_mode; }

void Driver::setNoiseFilter(NoiseFilter filter) {
  _noise_filter = filter;
  _reconfigure();
}

NoiseFilter Driver::getNoiseFilter() const { return _noise_filter; }

void Driver::setAveraging(Averaging averaging) {
  _averaging = averaging;
  _reconfigure();
}

Averaging Driver::getAveraging() const { return _averaging; }

void Driver::setOpenCircuitDetection(OpenCircuitDetection detection) {
  _open_circuit = detection;
  _reconfigure();
}

OpenCircuitDetection Driver::getOpenCircuitDetection() const { return _open_circuit; }

bool Driver::setProbeFaultThresholds(float low, float high) {
  if (isnan(low) || isnan(high) || low > high) return false;

  // Two's complement, 0.0625 °C / LSB
  _probe_low_raw  = static_cast<int16_t>(toFixed(low, THRESHOLD_LSB_C, INT16_MIN, INT16_MAX));
  _probe_high_raw = static_cast<int16_t>(toFixed(high, THRESHOLD_LSB_C, INT16_MIN, INT16_MAX));

  if (_initialized) _writeLimits();
  return true;
}

bool Driver::setColdJunctionFaultThresholds(int8_t low, int8_t high) {
  if (low > high) return false;

  _cj_low  = low;
  _cj_high = high;

  if (_initialized) _writeLimits();
  return true;
}

/* -------------------------------------------- Faults ------------------------------------------ */
void Driver::setFaultMode(FaultMode mode) {
  _fault_mode = mode;
  _reconfigure();
}

FaultMode Driver::getFaultMode() const { return _fault_mode; }

void Driver::setFaultOutput(Fault faults) {
  _fault_output = faults;
  if (_initialized) _writeLimits();
}

Fault Driver::getFaultOutput() const { return _fault_output; }

void Driver::clearFaults() {
  if (!_initialized) return;

  if (_conversion_mode == ConversionMode::Continuous) {
    _writeRegister8(Reg::CR0, _cr0() | CR0Bit::AUTOCONVERT | CR0Bit::FAULTCLR);
  } else {
    // CR0 is rewritten with 1SHOT = 0, so a running conversion must finish first
    _waitOneShotIdle(_conversionTimeoutMs());
    _writeRegister8(Reg::CR0, _cr0() | CR0Bit::FAULTCLR);
  }
}

/* ----------------------------------------- Cold junction -------------------------------------- */
bool Driver::setColdJunctionOffset(float offset) {
  if (isnan(offset)) return false;

  // Two's complement, 0.0625 °C / LSB (-8 to +7.9375 °C)
  _cj_offset_raw = static_cast<int8_t>(toFixed(offset, THRESHOLD_LSB_C, INT8_MIN, INT8_MAX));

  if (_initialized) _writeLimits();
  return true;
}

float Driver::getColdJunctionOffset() const { return _cj_offset_raw * THRESHOLD_LSB_C; }

void Driver::setColdJunctionSensorEnabled(bool enabled) {
  _cj_sensor_enabled = enabled;
  _reconfigure();
}

bool Driver::isColdJunctionSensorEnabled() const { return _cj_sensor_enabled; }

bool Driver::writeColdJunctionTemperature(float temperature) {
  if (!_initialized || _cj_sensor_enabled || isnan(temperature)) return false;

  // 14-bit two's complement, 0.015625 °C / LSB, left-aligned in CJTH:CJTL.
  // The chip clamps the value to -64..+128 °C
  const int32_t code  = toFixed(temperature, CJ_LSB_C, -64 * 64, 128 * 64 - 1);
  const uint16_t reg  = static_cast<uint16_t>(static_cast<uint16_t>(code) << 2);
  const uint8_t buf[] = {static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg)};
  _writeRegisterN(Reg::CJTH, buf, sizeof(buf));
  return true;
}

/* ----------------------------------------- Data-ready pin ------------------------------------- */
void Driver::setDataReadyPin(int8_t drdy_pin) {
  _drdy_pin = drdy_pin;
  if (_initialized && _drdy_pin >= 0) pinMode(_drdy_pin, INPUT_PULLUP);
}

/* ------------------------------------------- Readings ----------------------------------------- */
bool Driver::read(Measurement& measurement) {
  measurement = Measurement{NAN, NAN, NAN, Fault::None};
  if (!_acquire()) return false;

  // CJTH, CJTL, LTCBH, LTCBM, LTCBL, SR in one transfer, so all values belong to the same update
  uint8_t buf[6];
  _readRegisterN(Reg::CJTH, buf, sizeof(buf));
  _release();

  const int32_t code = decodeLinearized(&buf[2]);
  if (isVoltageType(_type))
    measurement.voltage = _toVoltage(code);
  else
    measurement.temperature = code * TC_LSB_C;

  measurement.cold_junction = decodeColdJunction(&buf[0]);
  measurement.fault         = _toFault(buf[5]);
  return true;
}

float Driver::readProbeTemperature() {
  if (isVoltageType(_type) || !_acquire()) return NAN;

  uint8_t buf[3];
  _readRegisterN(Reg::LTCBH, buf, sizeof(buf));
  _release();

  return decodeLinearized(buf) * TC_LSB_C;
}

float Driver::readVoltage() {
  if (!isVoltageType(_type) || !_acquire()) return NAN;

  uint8_t buf[3];
  _readRegisterN(Reg::LTCBH, buf, sizeof(buf));
  _release();

  return _toVoltage(decodeLinearized(buf));
}

float Driver::readColdJunctionTemperature() {
  if (!_acquire()) return NAN;

  uint8_t buf[2];
  _readRegisterN(Reg::CJTH, buf, sizeof(buf));
  _release();

  return decodeColdJunction(buf);
}

Fault Driver::readFault() {
  if (!_initialized) return Fault::None;
  return _toFault(_readRegister8(Reg::SR));
}

/* ---------------------------------------- One-shot control ------------------------------------ */
void Driver::triggerOneShot() {
  if (!_initialized || _conversion_mode == ConversionMode::Continuous) return;
  if (!conversionComplete()) return; // already running

  _writeRegister8(Reg::CR0, _cr0() | CR0Bit::ONESHOT);
}

bool Driver::conversionComplete() {
  if (!_initialized) return false;

  if (_conversion_mode == ConversionMode::Continuous) {
    if (_drdy_pin >= 0) return digitalRead(_drdy_pin) == LOW;
    return _has_result || millis() - _conversion_start_ms >= _active_conversion_ms;
  }

  return (_readRegister8(Reg::CR0) & CR0Bit::ONESHOT) == 0;
}

/* ---------------------------------------------------------------------------------------------- */
/*                                     Configuration helpers                                      */
/* ---------------------------------------------------------------------------------------------- */

// CR0 for the current configuration, in normally-off mode with no conversion requested
uint8_t Driver::_cr0() const {
  uint8_t cr0 = static_cast<uint8_t>(_open_circuit) << CR0Bit::OCFAULT_SHIFT;
  if (!_cj_sensor_enabled) cr0 |= CR0Bit::CJ_DISABLE;
  if (_fault_mode == FaultMode::Interrupt) cr0 |= CR0Bit::FAULT_INT;
  if (_noise_filter == NoiseFilter::Hz50) cr0 |= CR0Bit::NOISE_50HZ;
  return cr0;
}

uint8_t Driver::_cr1() const {
  return (static_cast<uint8_t>(_averaging) << CR1Bit::AVGSEL_SHIFT) |
         (static_cast<uint8_t>(_type) & CR1Bit::TYPE_MASK);
}

// Mask bits set to 1 keep the fault from asserting the FAULT pin (bits 7:6 are reserved)
uint8_t Driver::_maskRegister() const { return ~static_cast<uint8_t>(_fault_output) & 0x3F; }

// Writes CR0 and CR1. The datasheet requires the noise filter and averaging to be changed in
// normally-off mode with no conversion running, so conversions are stopped first and then restarted
void Driver::_reconfigure() {
  if (!_initialized) return;

  const uint8_t cr0 = _readRegister8(Reg::CR0);
  if (cr0 & CR0Bit::AUTOCONVERT) {
    // The conversion in progress is not visible in any register: wait out its worst-case duration
    _writeRegister8(Reg::CR0,
      static_cast<uint8_t>(cr0 & ~(CR0Bit::AUTOCONVERT | CR0Bit::FAULTCLR)));
    delay(_active_conversion_ms);
  } else if (cr0 & CR0Bit::ONESHOT) {
    _waitOneShotIdle(MAX_CONVERSION_TIMEOUT_MS);
  }

  _writeRegister8(Reg::CR1, _cr1());
  _writeRegister8(Reg::CR0, _cr0());
  _startConversions();
}

void Driver::_startConversions() {
  _has_result           = false;
  _conversion_start_ms  = millis();
  _active_conversion_ms = _conversionTimeMs();

  // Reading the thermocouple register releases DRDY, so it only signals results from now on
  if (_drdy_pin >= 0) _readRegister8(Reg::LTCBH);

  if (_conversion_mode == ConversionMode::Continuous)
    _writeRegister8(Reg::CR0, _cr0() | CR0Bit::AUTOCONVERT);
  else if (_conversion_mode == ConversionMode::OneShotNoWait)
    _writeRegister8(Reg::CR0, _cr0() | CR0Bit::ONESHOT);
}

// Writes MASK, the fault thresholds and CJTO (registers 0x02 to 0x09) in one transfer
void Driver::_writeLimits() {
  const uint16_t probe_high = static_cast<uint16_t>(_probe_high_raw);
  const uint16_t probe_low  = static_cast<uint16_t>(_probe_low_raw);

  const uint8_t buf[] = {
    _maskRegister(),
    static_cast<uint8_t>(_cj_high),
    static_cast<uint8_t>(_cj_low),
    static_cast<uint8_t>(probe_high >> 8),
    static_cast<uint8_t>(probe_high),
    static_cast<uint8_t>(probe_low >> 8),
    static_cast<uint8_t>(probe_low),
    static_cast<uint8_t>(_cj_offset_raw),
  };
  _writeRegisterN(Reg::MASK, buf, sizeof(buf));
}

// Writes two complementary patterns to CJTO and reads them back. Detects a missing chip or a broken
// SPI line, which typically reads as 0x00 or 0xFF
bool Driver::_checkCommunication() {
  static constexpr uint8_t patterns[] = {0xA5, 0x5A};

  bool ok = true;
  for (uint8_t pattern : patterns) {
    _writeRegister8(Reg::CJTO, pattern);
    if (_readRegister8(Reg::CJTO) != pattern) {
      ok = false;
      break;
    }
  }

  _writeRegister8(Reg::CJTO, static_cast<uint8_t>(_cj_offset_raw));
  return ok;
}

/* ---------------------------------------------------------------------------------------------- */
/*                                      Conversion helpers                                        */
/* ---------------------------------------------------------------------------------------------- */

// Worst-case duration of a one-shot (or first automatic) conversion with the current settings
uint32_t Driver::_conversionTimeMs() const {
  const bool hz50      = _noise_filter == NoiseFilter::Hz50;
  const uint32_t extra = (1u << static_cast<uint8_t>(_averaging)) - 1;
  const uint8_t* oc_ms = _cj_sensor_enabled ? OC_TEST_CJ_ON_MS : OC_TEST_CJ_OFF_MS;

  return (hz50 ? TCONV_MAX_50HZ_MS : TCONV_MAX_60HZ_MS) +
         extra * (hz50 ? AVG_SAMPLE_50HZ_MS : AVG_SAMPLE_60HZ_MS) +
         oc_ms[static_cast<uint8_t>(_open_circuit)];
}

uint32_t Driver::_conversionTimeoutMs() const {
  const uint32_t t = _conversionTimeMs();
  return t + t / 4;
}

bool Driver::_waitOneShotIdle(uint32_t timeout_ms) {
  const uint32_t start = millis();

  while (_readRegister8(Reg::CR0) & CR0Bit::ONESHOT) {
    if (millis() - start > timeout_ms) return false;
    delay(POLL_INTERVAL_MS);
  }
  return true;
}

// Makes sure the result registers hold a valid result for the current conversion mode.
// Returns false on timeout (OneShot) or if no conversion has finished yet (non-blocking modes)
bool Driver::_acquire() {
  if (!_initialized) return false;

  if (_conversion_mode == ConversionMode::OneShot) {
    triggerOneShot();
    return _waitOneShotIdle(_conversionTimeoutMs());
  }

  if (!_has_result) _has_result = conversionComplete();
  return _has_result;
}

// Called after reading results: in OneShotNoWait mode, starts the next conversion
void Driver::_release() {
  if (_conversion_mode == ConversionMode::OneShotNoWait) triggerOneShot();
}

Fault Driver::_toFault(uint8_t status) const {
  Fault fault = static_cast<Fault>(status);

  // The TC Range bit is meaningless in voltage mode
  if (isVoltageType(_type)) fault &= ~Fault::TCRange;
  return fault;
}

float Driver::_toVoltage(int32_t code) const {
  const float gain = (static_cast<uint8_t>(_type) & 0x04) ? 32.0f : 8.0f;
  return code / (gain * VOLTAGE_SCALE);
}

/* ---------------------------------------------------------------------------------------------- */
/*                                       SPI helpers                                              */
/* ---------------------------------------------------------------------------------------------- */
uint8_t Driver::_readRegister8(uint8_t addr) {
  uint8_t value = 0;
  _readRegisterN(addr, &value, 1);
  return value;
}

void Driver::_readRegisterN(uint8_t addr, uint8_t* buffer, uint8_t n) {
  addr &= 0x7F; // MSB=0 indicates read

  _spi->beginTransaction(_spi_settings);
  digitalWrite(_cs_pin, LOW);

  _spi->transfer(addr);
  for (uint8_t i = 0; i < n; ++i)
    buffer[i] = _spi->transfer(0x00);

  digitalWrite(_cs_pin, HIGH);
  _spi->endTransaction();
}

void Driver::_writeRegister8(uint8_t addr, uint8_t data) { _writeRegisterN(addr, &data, 1); }

// Multi-byte write: the address auto-increments while CS stays low
void Driver::_writeRegisterN(uint8_t addr, const uint8_t* data, uint8_t n) {
  addr |= 0x80; // MSB=1 indicates write

  _spi->beginTransaction(_spi_settings);
  digitalWrite(_cs_pin, LOW);

  _spi->transfer(addr);
  for (uint8_t i = 0; i < n; ++i)
    _spi->transfer(data[i]);

  digitalWrite(_cs_pin, HIGH);
  _spi->endTransaction();
}

} // namespace MAX31856
