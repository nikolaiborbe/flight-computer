/**
 * @file ring_buffer.hpp
 * @author Henrik Berg (henrik.berg@propulsentnu.no)
 *
 * @brief Lock-free SPSC ring buffer.
 *
 * Provides a fixed-capacity FIFO queue for passing data between
 * a single producer and a single consumer.
 *
 * @copyright Propulse NTNU (c) 2026
 */
#pragma once

#include <array>

extern "C" {
#include <cmsis_gcc.h>
}

/**
 * Fixed-size SPSC ring buffer.
 *
 * @tparam T Element type.
 * @tparam N Buffer capacity in elements.
 */
template<typename T, std::size_t N>
class RingBuffer {
public:
  /**
   * Inserts an item into the buffer.
   *
   * @param item Item to insert.
   *
   * @return True if the item was inserted.
   */
  bool push(const T& item) {
    const uint32_t head = head_;

    if ((head - tail_) >= static_cast<uint32_t>(N)) {
      return false;
    }

    data_[head % N] = item;

    __DMB();
    head_ = head + 1;

    return true;
  }

  /**
   * Removes the oldest item from the buffer.
   *
   * @param out Destination for the retrieved item.
   *
   * @return True if an item was retrieved.
   */
  bool pop(T& out) {
    const uint32_t tail = tail_;

    if (tail == head_) {
      return false;
    }

    out = data_[tail % N];

    __DMB();
    tail_ = tail + 1;

    return true;
  }

private:
  std::array<T, N> data_{};

  volatile uint32_t head_{0};
  volatile uint32_t tail_{0};
};