#pragma once

#include <atomic>

template <typename T>
class AtomicTuple
{
public:
    AtomicTuple(T atomic_1, T atomic_2)
        : atomic_tpl{atomic_1, atomic_2} {
          };

    ~AtomicTuple() = default;

    T load() const noexcept
    {
        return atomic_tpl[swap].load(std::memory_order_relaxed);
    }

    void store(T value) noexcept
    {
        swap = !swap;
        atomic_tpl[swap].store(value, std::memory_order_relaxed);
        return;
    }

private:
    std::atomic<T> atomic_tpl[2];

    // if false
    //      load from atomic_1, store to atomic_2
    // if true
    //      store to atomic_1, load to atomic_2
    bool swap = false;
};