#pragma once
#include <cstdint>

// Scalar Kalman filter in Q16.16 fixed point (no FPU on Cortex-M3, so no floating point).
// State model: x[k] = x[k-1] + w (variance q), measurement: z = x + v (variance r).
class KalmanQ16 {
 public:
  constexpr KalmanQ16(std::int32_t q, std::int32_t r, std::int32_t p0)
      : q_(static_cast<std::int64_t>(q) << 16),
        r_(static_cast<std::int64_t>(r) << 16),
        p_(static_cast<std::int64_t>(p0) << 16) {}

  std::int32_t update(std::int32_t z) {
    if (!init_) {
      x_ = static_cast<std::int64_t>(z) << 16;
      init_ = true;
      return z;
    }
    p_ += q_;                                     // predict
    std::int64_t k = (p_ << 16) / (p_ + r_);      // Kalman gain, Q16.16
    x_ += (k * ((static_cast<std::int64_t>(z) << 16) - x_)) >> 16;  // correct
    p_ = ((static_cast<std::int64_t>(1) << 16) - k) * p_ >> 16;
    return static_cast<std::int32_t>(x_ >> 16);
  }

 private:
  std::int64_t q_, r_, p_;
  std::int64_t x_ = 0;
  bool init_ = false;
};
