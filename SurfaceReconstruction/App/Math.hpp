#pragma once

#include <algorithm>
#include <cmath>

namespace Pvl {

const float PI = M_PI;

template <typename T>
T sqr(const T& value) {
    return value * value;
}
template <typename T>
int sign(const T& value) {
    if (value > 0) {
        return 1;
    } else if (value < 0) {
        return -1;
    } else {
        return 0;
    }
}

template <typename T, int N>
struct Pow;

template <typename T>
struct Pow<T, 2> {
    T operator()(const T& value) {
        return value * value;
    }
};
template <typename T>
struct Pow<T, 3> {
    T operator()(const T& value) {
        return value * value * value;
    }
};
template <typename T>
struct Pow<T, 4> {
    T operator()(const T& value) {
        return sqr(value * value);
    }
};

template <int N, typename T>
T pow(const T& value) {
    return Pow<T, N>()(value);
}

template <typename T>
T clamp(const T& value, const T& lower, const T& upper) {
    return std::min(std::max(value, lower), upper);
}

} // namespace Pvl
