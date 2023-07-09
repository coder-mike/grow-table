#pragma once

#include <stdint.h>

class IntervalTimer
{
private:
  uint32_t intervalMs_;
  uint32_t nextExpire_;

public:
  IntervalTimer(uint32_t intervalMs) : intervalMs_(intervalMs) {
    nextExpire_ = millis();
  }

  /**
   * Call in main loop to check if the timer has expired.
   * Returns true once for every time it expires. If it's missed
   * one or more intervals, it will return true for each one.
   *
   * Note that the first expiry immediate. This is designed for periodic
   * processes that also need to run immediately on startup.
   */
  bool check() {
    uint32_t now = millis();
    while ((long)(now - nextExpire_) >= 0) {
      nextExpire_ += intervalMs_;
      return true;
    }
    return false;
  }

  uint32_t intervalMs() {
    return intervalMs_;
  }

  void reset() {
    nextExpire_ = millis() + intervalMs_;
  }
};

