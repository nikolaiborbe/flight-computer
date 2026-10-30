#pragma once

#include <array>
#include <cstddef>

#if defined(USE_THREADX)
#include "tx_api.h"
#endif

template<typename ItemT, std::size_t N>
class StreamBuffer {
public:
	struct Reader {
		uint32_t next_seq = 0;
	};

	struct ReadResult {
		std::size_t count = 0;
		bool overrun = false;
	};

	StreamBuffer() {
#if defined(USE_THREADX)
		tx_mutex_create(&tx_mutex_, const_cast<char*>("StreamBuffer Mutex"), TX_INHERIT);
#endif
	}

	Reader makeReader() const {
		lock();

		Reader reader{};
		reader.next_seq = write_seq_;

		unlock();
		return reader;
	}

	ReadResult read(Reader& reader, ItemT* out, std::size_t max) const {
		lock();

		ReadResult res{};

		if ((out == nullptr) || (max == 0)) {
			unlock();
			return res;
		}

		const uint32_t w = write_seq_;
		if (reader.next_seq == w) {
			unlock();
			return res;
		}

		if ((w - reader.next_seq) > static_cast<uint32_t>(N)) {
			res.overrun = true;
			reader.next_seq = w - static_cast<uint32_t>(N);
		}

		while ((res.count < max) && (reader.next_seq != w)) {
			const std::size_t idx = static_cast<std::size_t>(reader.next_seq % N);
			out[res.count++] = data_[idx];
			++reader.next_seq;
		}

		unlock();
		return res;
	}

	void pushItem(const ItemT& item) {
		lock();

		const std::size_t idx = static_cast<std::size_t>(write_seq_ % N);
		data_[idx] = item;
		++write_seq_;

		unlock();
	}

private:
	void lock() const {
#if defined(USE_THREADX)
		tx_mutex_get(&tx_mutex_, TX_WAIT_FOREVER);
#endif
	}

	void unlock() const {
#if defined(USE_THREADX)
		tx_mutex_put(&tx_mutex_);
#endif
	}

	std::array<ItemT, N> data_{};
	uint32_t write_seq_ = 0;

#if defined(USE_THREADX)
	mutable TX_MUTEX tx_mutex_{};
#endif
};
