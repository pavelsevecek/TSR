#pragma once

namespace Pvl {

struct Noncopyable {
    Noncopyable() = default;
    Noncopyable(const Noncopyable&) = delete;
    Noncopyable(Noncopyable&&) = delete;
    Noncopyable& operator=(const Noncopyable&) = delete;
    Noncopyable& operator=(Noncopyable&&) = delete;
};

struct ParallelTag {};
struct SequentialTag {};

template <typename Tag>
struct ParallelFor;

template <>
struct ParallelFor<SequentialTag> {
    template <typename Index, typename Func>
    void operator()(Index n1, Index n2, const Func& func) {
        for (Index n = n1; n < n2; ++n) {
            func(n);
        }
    }
};


template <typename Tag>
struct ParallelForEach;

template <>
struct ParallelForEach<SequentialTag> {
    template <typename Iter, typename Func>
    void operator()(Iter from, Iter to, const Func& func) {
        for (Iter iter = from; iter != to; ++iter) {
            func(*iter);
        }
    }
    template <typename Range, typename Func>
    void operator()(const Range& range, const Func& func) {
        for (auto&& value : range) {
            func(value);
        }
    }
};

} // namespace Pvl
