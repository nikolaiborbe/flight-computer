#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>
#include <type_traits>

namespace algo {

inline constexpr std::size_t kCacheLine = 64;

template <typename T> class alignas(kCacheLine) Seqlock {
    static_assert(std::is_trivially_copyable_v<T>, "Seqlock<T> requires a trivially copyable T");

    std::atomic<uint32_t> m_seq {0};
    std::atomic<bool> m_writer_taken {false};
    alignas(kCacheLine) T m_data {};

public:
    constexpr Seqlock() noexcept = default;

    static Seqlock* create_at(void* mem) noexcept { return ::new (mem) Seqlock(); }
    static Seqlock* open_at(void* mem) noexcept { return std::launder(static_cast<Seqlock*>(mem)); }
    static constexpr std::size_t footprint() noexcept { return sizeof(Seqlock); }
    static constexpr std::size_t alignment() noexcept { return alignof(Seqlock); }

    class Writer {
        friend Seqlock;

        Seqlock* m_lock = nullptr;

        explicit Writer(Seqlock* lock) noexcept
            : m_lock(lock)
        {
        }

    public:
        constexpr Writer() noexcept = default;

        Writer(const Writer&)            = delete;
        Writer& operator=(const Writer&) = delete;

        Writer(Writer&& other) noexcept
            : m_lock(other.m_lock)
        {
            other.m_lock = nullptr;
        }
        Writer& operator=(Writer&& other) noexcept
        {
            if (this != &other) {
                m_lock       = other.m_lock;
                other.m_lock = nullptr;
            }
            return *this;
        }

        bool store(const T& v) noexcept
        {
            if (m_lock == nullptr)
                return false;
            m_lock->store(v);
            return true;
        }

        bool store_bytes(const void* src) noexcept
        {
            if (m_lock == nullptr)
                return false;
            m_lock->store_bytes(src);
            return true;
        }

        [[nodiscard]] bool valid() const noexcept { return m_lock != nullptr; }
        explicit operator bool() const noexcept { return valid(); }
    };

    [[nodiscard]] Writer acquire_writer_once() noexcept
    {
        bool expected = false;
        if (m_writer_taken.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
            return Writer(this);
        }
        return Writer(nullptr);
    }

    class Reader {
    private:
        const Seqlock<T>* m_src = nullptr;
        T m_value {};
        uint32_t m_last_gen = 0;

    public:
        constexpr Reader() noexcept = default;

        void bind(const Seqlock<T>* src) noexcept { m_src = src; }

        bool bound() const noexcept { return m_src != nullptr; }

        bool refresh() noexcept
        {
            if (m_src == nullptr)
                return false;
            uint32_t gen = 0;
            if (m_src->load(m_value, gen)) {
                const bool changed = (gen != m_last_gen);
                m_last_gen         = gen;
                return changed && gen != 0;
            }
            return false;
        }

        [[nodiscard]] const T& update_and_read() noexcept
        {
            refresh();
            return value();
        }

        [[nodiscard]] const T& value() const noexcept { return m_value; }
        [[nodiscard]] uint32_t generation() const noexcept { return m_last_gen; }
    };

private:
    void store(const T& v) noexcept
    {
        const uint32_t s = m_seq.load(std::memory_order_relaxed);
        m_seq.store(s + 1, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        m_data = v;
        std::atomic_thread_fence(std::memory_order_release);
        m_seq.store(s + 2, std::memory_order_release);
    }

    void store_bytes(const void* src) noexcept
    {
        const uint32_t s = m_seq.load(std::memory_order_relaxed);
        m_seq.store(s + 1, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        std::memcpy(&m_data, src, sizeof(T));
        std::atomic_thread_fence(std::memory_order_release);
        m_seq.store(s + 2, std::memory_order_release);
    }

    [[nodiscard]] T* begin_write() noexcept
    {
        const uint32_t s = m_seq.load(std::memory_order_relaxed);
        m_seq.store(s + 1, std::memory_order_relaxed); // odd
        std::atomic_thread_fence(std::memory_order_release);
        return &m_data;
    }

    void end_write() noexcept
    {
        std::atomic_thread_fence(std::memory_order_release);
        const uint32_t s = m_seq.load(std::memory_order_relaxed);
        m_seq.store(s + 1, std::memory_order_release);
    }

    [[nodiscard]] T* payload_ptr() noexcept { return &m_data; }
    [[nodiscard]] const T* payload_ptr() const noexcept { return &m_data; }
    static constexpr std::size_t payload_size() noexcept { return sizeof(T); }

public:
    [[nodiscard]] bool load(T& out, uint32_t& gen, int max_tries = 4) const noexcept
    {
        for (int i = 0; i < max_tries; ++i) {
            const uint32_t s1 = m_seq.load(std::memory_order_acquire);
            if (s1 & 1u)
                continue;

            T tmp = m_data;
            std::atomic_thread_fence(std::memory_order_acquire);
            const uint32_t s2 = m_seq.load(std::memory_order_relaxed);

            if (s1 == s2) {
                out = tmp;
                gen = s1;
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] bool try_read(T& out, int max_tries = 4) const noexcept
    {
        uint32_t gen;
        return load(out, gen, max_tries);
    }

    [[nodiscard]] uint32_t generation() const noexcept
    {
        return m_seq.load(std::memory_order_acquire);
    }
};
} // namespace algo