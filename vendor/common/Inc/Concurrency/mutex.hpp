/**
 * @file mutex.hpp
 * @author Henrik Berg (henrik.berg@propulsentnu.no)
 *
 * @brief Cross-platform mutex and RAII lock guard.
 *
 * Provides a mutex abstraction with optional ThreadX support, together
 * with a scoped lock guard.
 *
 * @copyright Propulse NTNU (c) 2026
 */
#pragma once

#include <cstdint>

#ifdef USE_THREADX
#include "tx_api.h"
#endif

/**
 * Mutex wrapper.
 */
class Mutex {
public:
  static constexpr uint32_t NoWait = 0;
  static constexpr uint32_t WaitForever = 0xFFFFFFFFu;

  /**
   * Constructs and initializes the mutex.
   */
  Mutex() {
#ifdef USE_THREADX
    tx_mutex_create(&mutex_, const_cast<char*>("Mutex"), TX_INHERIT);
#endif
  }

  /**
   * Destroys the mutex and releases resources.
   */
  ~Mutex() {
#ifdef USE_THREADX
    tx_mutex_delete(&mutex_);
#endif
  }

  Mutex(const Mutex&) = delete;
  Mutex& operator=(const Mutex&) = delete;

  /**
   * Acquires the mutex, blocking indefinitely if necessary.
   *
   * @return True if the mutex was acquired.
   */
  bool lock() {
#ifdef USE_THREADX
    return tx_mutex_get(&mutex_, TX_WAIT_FOREVER) == TX_SUCCESS;
#else
    return true;
#endif
  }

  /**
   * Acquires the mutex with a timeout.
   *
   * @param wait_option Timeout in RTOS ticks.
   * @return True if the mutex was acquired.
   */
  bool lock(uint32_t wait_option) {
#ifdef USE_THREADX
    return tx_mutex_get(&mutex_, wait_option) == TX_SUCCESS;
#else
    return true;
#endif
  }

  /**
   * Releases the mutex.
   */
  void unlock() {
#ifdef USE_THREADX
    tx_mutex_put(&mutex_);
#endif
  }

private:
#ifdef USE_THREADX
  TX_MUTEX mutex_{};
#endif
};

/**
 * Scoped mutex helper.
 */
class LockGuard {
public:
  /**
   * Acquires the mutex, blocking until available.
   *
   * @param mutex Mutex to acquire.
   */
  explicit LockGuard(Mutex& mutex)
      : mutex_(mutex), acquired_(mutex_.lock()) {}

  /**
   * Acquires the mutex with a timeout.
   *
   * @param mutex Mutex to acquire.
   * @param timeout_ticks Timeout in RTOS ticks.
   */
  LockGuard(Mutex& mutex, const uint32_t timeout_ticks)
      : mutex_(mutex), acquired_(mutex_.lock(timeout_ticks)) {}

  /**
   * Releases the mutex if acquired.
   */
  ~LockGuard() {
    if (acquired_) {
      mutex_.unlock();
    }
  }

  LockGuard(const LockGuard&) = delete;
  LockGuard& operator=(const LockGuard&) = delete;

  /**
   * Indicates whether the mutex was acquired.
   *
   * @return True if the mutex is currently held by this guard.
   */
  bool acquired() const { return acquired_; }

private:
  Mutex& mutex_;
  bool acquired_;
};
