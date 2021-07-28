/*
 * Copyright 2021 ElectroOptical Innovations, LLC
 * AD5273.h
 *
 * Features:
 *  - 64 positions
 *  - One-time programmable (OTP)1 set-and-forget
 *  - Resistance setting—low cost alternative over EEMEM
 *  - Unlimited adjustments prior to OTP activation
 *  - 1 kΩ, 10 kΩ, 50 kΩ, 100 kΩ end-to-end terminal resistance
 *
 *
 * Supports both reads and writes
 * Reads are the slave address, read bit and then the 2 fuse status bits with the 6 right justified data bits
 * Writes are either OTP or MTP. They are the slave address with write bit followed by the mode and then 2 DNC bits with the setting value right justified
 *
 * Addressing:
 *  - Has one address line
 *  - Default address 010110x
 *  - No global address
 */


#pragma once
#include <I2CBus/I2CInterface.h>
#include <cstdint>
#include <cmath>

namespace AD5273 {
const size_t kDataBits = 8;
const size_t kOtpValidationStart = 6;
const uint8_t kSlaveAddressBase = 0b0101100;  //  needs to be or'd with the address
inline constexpr size_t get_nsteps(void) {
  return (1 << kDataBits) - 1;
}
inline constexpr size_t get_lowest_resistance_step() {
  return 0;
}
inline constexpr size_t get_highest_resistance_step() {
  return get_nsteps() - get_lowest_resistance_step();
}

class Driver final : public I2CDeviceBase {
 private:
  uint8_t last_read_ = 0;
  const uint8_t kSlaveAddress;

  void write_(const uint8_t otp_byte, const uint8_t value) {
    InsertOperation(
        {I2COperationType::kWrite, MakeI2CSlaveWriteAddress(kSlaveAddress)});
    InsertOperation({I2COperationType::kStart});
    InsertOperation({I2COperationType::kWrite, otp_byte});
    InsertOperation({I2COperationType::kContinue});
    InsertOperation({I2COperationType::kWrite, value});
    InsertOperation({I2COperationType::kContinue});
  }

 public:
  explicit Driver(const bool a0 = false)
      : kSlaveAddress{static_cast<uint8_t>((kSlaveAddressBase << 1) |
                      static_cast<uint8_t>(a0))} {}
  virtual ~Driver(void) {}

  uint8_t get_value(void) {
    return last_read_ & ~(0x3 << kOtpValidationStart);
  }
  uint8_t get_otp_validation(void) {
    return last_read_ >> kOtpValidationStart;
  }
  void fuse_write(const uint8_t value) {
    write_(0xff, value);
  }
  void write(const uint8_t value) {
    write_(0x00, value);
  }
  void read(void) {
    InsertOperation(
        {I2COperationType::kWrite, MakeI2CSlaveReadAddress(kSlaveAddress)});
    InsertOperation({I2COperationType::kStart});
  }

  virtual void PushData(uint8_t value) {
    last_read_ = value;
  }
  virtual void Reset() {}

  virtual void Run(void) {}
};
}  //  namespace AD5273

