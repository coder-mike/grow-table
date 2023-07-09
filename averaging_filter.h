#pragma once

#include <stdint.h>

template <int N>
class AveragingFilter
{
private:
  int32_t values_[N];
  uint32_t count_;
  uint32_t index_;
  int64_t sum_;

public:
  AveragingFilter() : index_(0), count_(0), sum_(0) {}

  /** Consume a new value and return an average of the most recent N values. */
  int32_t filter(int32_t value) {
    if (count_ == N) {
      // Buffer is full. Remove the oldest value.
      sum_ -= values_[index_];
    } else {
      count_++;
    }
    values_[index_] = value;
    index_ = (index_ + 1) % N;
    sum_ += value;
    if (count_ == N) {
      // Constant divide by N is optimized by the compiler.
      return sum_ / N;
    } else {
      return sum_ / count_;
    }
  }

  /**
   * The average slope of the most recent N values, represented as fixed-point
   * 32.32.
   */
  int64_t slope() {
    if (count_ == 0) {
      return 0;
    }
    if (count_ == N) { // Mode where buffer is full.
      int32_t first = values_[index_];
      int32_t last = values_[(index_ + N - 1) % N];
      return (((int64_t)last - first) << 32) / N;
    } else { // Mode where we're still filling up the buffer.
      int32_t first = values_[0];
      int32_t last = values_[count_ - 1];
      return (((int64_t)last - first) << 32) / count_;
    }
  }
};
