#pragma once

#include <Arduino.h>
#include <SPI.h>

namespace MAX31856 {

/* ---------------------------------------------------------------------------------------------- */
/*                                            Enums                                               */
/* ---------------------------------------------------------------------------------------------- */

// Thermocouple type / voltage mode (4 LSB of CR1 register)
enum class Type : uint8_t {
  B             = 0b0000,
  E             = 0b0001,
  J             = 0b0010,
  K             = 0b0011,
  N             = 0b0100,
  R             = 0b0101,
  S             = 0b0110,
  T             = 0b0111,
  VoltageGain8  = 0b1000, // raw input voltage, ±78.125 mV full scale, no linearization
  VoltageGain32 = 0b1100, // raw input voltage, ±19.531 mV full scale, no linearization
};

// Power-line noise filter
enum class NoiseFilter : uint8_t {
  Hz60 = 0,
  Hz50 = 1,
};

// Samples averaged per conversion (CR1 bits 6:4). More samples reduce noise but lengthen each
// conversion by (samples - 1) x 40 ms at 50 Hz or x 33.33 ms at 60 Hz
enum class Averaging : uint8_t {
  Samples1  = 0,
  Samples2  = 1,
  Samples4  = 2,
  Samples8  = 3,
  Samples16 = 4,
};

// Open-circuit fault detection (CR0 bits 5:4). Choose the option matching the thermocouple lead
// resistance (Rs) and input filter time constant. It runs on every conversion in the one-shot modes
// and once every 16 conversions in Continuous mode
enum class OpenCircuitDetection : uint8_t {
  Disabled     = 0,
  Below5k      = 1, // Rs < 5 kΩ (adds up to 15 ms per test)
  Below40kFast = 2, // 5 kΩ < Rs < 40 kΩ, time constant < 2 ms (adds up to 37 ms per test)
  Below40kSlow = 3, // 5 kΩ < Rs < 40 kΩ, time constant > 2 ms (adds up to 125 ms per test)
};

// Conversion mode
enum class ConversionMode : uint8_t {
  OneShot,       // each read triggers a conversion and waits for it (blocking, ~200 ms by default)
  OneShotNoWait, // each read returns the last result and triggers the next conversion
  Continuous,    // the chip converts every ~100 ms; reads return the latest result
};

// Fault reporting mode (CR0 bit 2)
enum class FaultMode : uint8_t {
  Comparator = 0, // fault bits and FAULT pin follow the current condition (2 °C hysteresis)
  Interrupt  = 1, // fault bits and FAULT pin stay asserted until clearFaults()
};

// Fault register bits (SR, 0x0F). Multiple bits can be active simultaneously
enum class Fault : uint8_t {
  None      = 0x00,
  Open      = 0x01, // thermocouple open / disconnected
  OverUnder = 0x02, // over/under voltage on inputs (conversions are suspended while present)
  TCLow     = 0x04, // thermocouple temperature below threshold
  TCHigh    = 0x08, // thermocouple temperature above threshold
  CJLow     = 0x10, // cold-junction temperature below threshold
  CJHigh    = 0x20, // cold-junction temperature above threshold
  TCRange   = 0x40, // thermocouple out of its normal range (always cleared in voltage modes)
  CJRange   = 0x80, // cold junction out of range (-55 to +125 °C for most types)
  All       = 0xFF,
};

/* ---------------------------------------------------------------------------------------------- */
/*                                   Bitmask operators for Fault                                  */
/* ---------------------------------------------------------------------------------------------- */
constexpr Fault operator|(Fault a, Fault b) {
  return static_cast<Fault>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

constexpr Fault operator&(Fault a, Fault b) {
  return static_cast<Fault>(static_cast<uint8_t>(a) & static_cast<uint8_t>(b));
}

constexpr Fault operator^(Fault a, Fault b) {
  return static_cast<Fault>(static_cast<uint8_t>(a) ^ static_cast<uint8_t>(b));
}

constexpr Fault operator~(Fault a) { return static_cast<Fault>(~static_cast<uint8_t>(a)); }

inline Fault& operator|=(Fault& a, Fault b) { return a = a | b; }
inline Fault& operator&=(Fault& a, Fault b) { return a = a & b; }
inline Fault& operator^=(Fault& a, Fault b) { return a = a ^ b; }

// Free helpers
constexpr bool any(Fault f) { return f != Fault::None; }
constexpr bool has(Fault f, Fault flag) { return (f & flag) == flag; }

/* ---------------------------------------------------------------------------------------------- */
/*                                          toString()                                            */
/* ---------------------------------------------------------------------------------------------- */
const char* toString(Type type);

// Stringifies a single fault bit. If a value with multiple bits is passed, returns "Multiple"
const char* toString(Fault flag);

// Stringifies all active flags concatenated with " | ", e.g.: "Open | TCLow".
// If no faults are present, writes "None". Truncates if it does not fit in the buffer.
void toString(Fault f, char* buffer, size_t buffer_size);

/* ---------------------------------------------------------------------------------------------- */
/*                                          Measurement                                           */
/* ---------------------------------------------------------------------------------------------- */

// Results of a single conversion, as returned by Driver::read()
struct Measurement {
  float temperature;   // thermocouple temperature in °C (NAN in voltage modes)
  float voltage;       // thermocouple input voltage in V (NAN in thermocouple modes)
  float cold_junction; // cold-junction temperature in °C
  Fault fault;         // fault status after the conversion
};

/* ---------------------------------------------------------------------------------------------- */
/*                                            Driver                                              */
/* ---------------------------------------------------------------------------------------------- */

// Configuration setters can be called before begin(): the values are stored and begin() applies
// them all at once. After begin(), each setter is applied to the chip immediately; settings in the
// CR0/CR1 registers (type, mode, filter, averaging, open-circuit, fault mode, cold-junction sensor)
// stop conversions first, which can block for up to one conversion time (~200 ms by default).
// Calling begin() again re-applies the whole configuration (e.g. after the sensor lost power).
//
// Read behaviour depends on the conversion mode:
//   - OneShot:       triggers a conversion and waits for it. Returns NAN on timeout.
//   - OneShotNoWait: returns the previous result and triggers the next conversion. Never blocks.
//   - Continuous:    returns the latest result. Never blocks.
// In the non-blocking modes, reads return NAN until the first conversion after begin() or after a
// configuration change has finished.
class Driver {
  public:
  // Constructs the driver. The SPI bus must be initialized externally (SPI.begin()).
  // cs_pin: chip-select pin; spi: SPIClass instance to use
  // spi_freq_hz: SPI clock frequency in Hz (default 1 MHz, clamped to the 5 MHz maximum)
  Driver(int8_t cs_pin, SPIClass& spi = SPI, uint32_t spi_freq_hz = 1000000);

  // Checks communication with the chip and applies the configuration. Defaults:
  //   - Type K, OneShot mode, 50 Hz filter, no averaging
  //   - Open-circuit detection for Rs < 5 kΩ, comparator fault mode, all faults on the FAULT pin
  //   - Internal cold-junction sensor enabled, no cold-junction offset
  //   - Fault thresholds at the chip defaults (thermocouple: full range, CJ: -64 to 127 °C)
  // Returns false if the chip does not respond (missing, unpowered or miswired)
  bool begin();

  // ----- Configuration -----
  void setType(Type type);
  Type getType() const;

  void setConversionMode(ConversionMode mode);
  ConversionMode getConversionMode() const;

  void setNoiseFilter(NoiseFilter filter);
  NoiseFilter getNoiseFilter() const;

  void setAveraging(Averaging averaging);
  Averaging getAveraging() const;

  void setOpenCircuitDetection(OpenCircuitDetection detection);
  OpenCircuitDetection getOpenCircuitDetection() const;

  // Fault thresholds for thermocouple temperature (in °C, 0.0625 °C resolution).
  // Returns false (and changes nothing) if a value is NAN or low > high
  bool setProbeFaultThresholds(float low, float high);

  // Fault thresholds for cold-junction temperature (in °C, -128 to 127).
  // Returns false (and changes nothing) if low > high
  bool setColdJunctionFaultThresholds(int8_t low, int8_t high);

  // ----- Faults -----
  void setFaultMode(FaultMode mode);
  FaultMode getFaultMode() const;

  // Selects which faults assert the FAULT pin (Open, OverUnder, TCLow, TCHigh, CJLow, CJHigh).
  // The fault register reports every fault regardless. The range faults never assert the pin
  void setFaultOutput(Fault faults);
  Fault getFaultOutput() const;

  // Clears latched faults in Interrupt mode (no effect in Comparator mode). Faults that are still
  // present reassert immediately. In the one-shot modes it first waits for a running conversion
  void clearFaults();

  // ----- Cold junction -----

  // Offset added by the chip to the measured cold-junction temperature
  // (-8 to +7.9375 °C, 0.0625 °C resolution). Returns false if the value is NAN
  bool setColdJunctionOffset(float offset);
  float getColdJunctionOffset() const;

  // Enables or disables the internal cold-junction sensor. When disabled, provide the cold-junction
  // temperature from an external sensor with writeColdJunctionTemperature()
  void setColdJunctionSensorEnabled(bool enabled);
  bool isColdJunctionSensorEnabled() const;

  // Writes the cold-junction temperature (°C, -64 to +127.984375, clamped) used for compensation.
  // Only allowed after begin() with the internal sensor disabled; returns false otherwise
  bool writeColdJunctionTemperature(float temperature);

  // ----- Data-ready pin -----

  // Optional DRDY pin (active low). In Continuous mode it lets conversionComplete() report when a
  // new result is available. Pass -1 to stop using it
  void setDataReadyPin(int8_t drdy_pin);

  // ----- Readings -----

  // Thermocouple temperature, input voltage, cold-junction temperature and faults, all from the
  // same conversion. Returns false (fields set to NAN / Fault::None) if no result is available
  bool read(Measurement& measurement);

  // Probe (thermocouple tip) temperature in °C. NAN in voltage modes or if no result is available
  float readProbeTemperature();

  // Thermocouple input voltage in V. NAN in thermocouple modes or if no result is available
  float readVoltage();

  // Cold-junction (chip / PCB) temperature in °C. NAN if no result is available
  float readColdJunctionTemperature();

  // Reads the fault register. May contain multiple active bits. Does not trigger a conversion
  Fault readFault();

  // ----- Manual one-shot conversion -----

  // Starts a one-shot conversion. Does nothing in Continuous mode or if one is already running
  void triggerOneShot();

  // One-shot modes: true when no conversion is running.
  // Continuous mode: true when a new result is available (requires DRDY pin); without DRDY pin,
  // true once the first conversion has had time to finish
  bool conversionComplete();

  private:
  int8_t _cs_pin;
  int8_t _drdy_pin;
  SPIClass* _spi;
  SPISettings _spi_settings;

  // ----- Configuration (applied by begin()) -----
  Type _type;
  ConversionMode _conversion_mode;
  NoiseFilter _noise_filter;
  Averaging _averaging;
  OpenCircuitDetection _open_circuit;
  FaultMode _fault_mode;
  Fault _fault_output;
  bool _cj_sensor_enabled;
  int16_t _probe_low_raw;
  int16_t _probe_high_raw;
  int8_t _cj_low;
  int8_t _cj_high;
  int8_t _cj_offset_raw;

  // ----- State -----
  bool _initialized;
  bool _has_result;               // a conversion finished since the last (re)configuration
  uint32_t _conversion_start_ms;  // when conversions were (re)started
  uint32_t _active_conversion_ms; // worst-case conversion time of the settings in the chip

  // ----- Configuration helpers -----
  uint8_t _cr0() const;
  uint8_t _cr1() const;
  uint8_t _maskRegister() const;
  void _reconfigure();
  void _startConversions();
  void _writeLimits();
  bool _checkCommunication();

  // ----- Conversion helpers -----
  uint32_t _conversionTimeMs() const;
  uint32_t _conversionTimeoutMs() const;
  bool _waitOneShotIdle(uint32_t timeout_ms);
  bool _acquire();
  void _release();
  Fault _toFault(uint8_t status) const;
  float _toVoltage(int32_t code) const;

  // ----- Register access -----
  uint8_t _readRegister8(uint8_t addr);
  void _readRegisterN(uint8_t addr, uint8_t* buffer, uint8_t n);
  void _writeRegister8(uint8_t addr, uint8_t data);
  void _writeRegisterN(uint8_t addr, const uint8_t* data, uint8_t n);
};

} // namespace MAX31856
