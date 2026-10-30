#pragma once
#include <cstdint>
#include <atomic>

extern "C" {
#include <cmsis_gcc.h>
}

template<typename T>
class SeqLock {
public:
  SeqLock() = default;

  void publish(const T& value) {
    // Oddetall = Skriving på gang
    seq_.fetch_add(1u, std::memory_order_relaxed);
    __DMB();

    data_ = value;

    __DMB();
    // Partall = skriving complete
    seq_.fetch_add(1u, std::memory_order_release);
  }

  T read() const {
    T copy{};
    uint32_t a = 0, b = 0;

    do {
      a = seq_.load(std::memory_order_acquire);
      __DMB();

      copy = data_;

      __DMB();
      b = seq_.load(std::memory_order_acquire);
    } while (a != b || (a & 1u));

    return copy;
  }

private:
  mutable std::atomic<uint32_t> seq_{0};
  T data_{};
};
