#pragma once

#include "Vector.hpp"

namespace Pvl {

template <typename Vec>
class BoundingBox {
    Vec lower_;
    Vec upper_;

public:
    using Vector = Vec;
    using Type = typename Vec::Type;

    BoundingBox()
        : lower_(std::numeric_limits<Type>::max())
        , upper_(std::numeric_limits<Type>::lowest()) {}

    BoundingBox(const Vec& lower, const Vec& upper)
        : lower_(lower)
        , upper_(upper) {}

    Vec& lower() {
        return lower_;
    }

    const Vec& lower() const {
        return lower_;
    }

    Vec& upper() {
        return upper_;
    }

    const Vec& upper() const {
        return upper_;
    }

    Vec size() const {
        return upper_ - lower_;
    }

    Vec center() const {
        return Type(0.5) * (upper_ + lower_);
    }

    bool contains(const Vec& p) const {
        for (int i = 0; i < Vec::size(); ++i) {
            if (p[i] < lower_[i] || p[i] > upper_[i]) {
                return false;
            }
        }
        return true;
    }

    void extend(const Vec& p) {
        lower_ = min(lower_, p);
        upper_ = max(upper_, p);
    }
};

using Box3f = BoundingBox<Vec3f>;

/// \brief Splits the box along given coordinate.
///
/// The splitting plane must pass through the box.
template <typename Box, typename T>
std::pair<Box, Box> splitBox(const Box& box, const int dim, const T x) {
    /*ASSERT(isValid());*/
    PVL_ASSERT(dim < Box::Vector::size());
    PVL_ASSERT(x >= box.lower()[dim] && x <= box.upper()[dim]);
    Box b1 = box;
    Box b2 = box;
    b1.upper()[dim] = x;
    b2.lower()[dim] = x;
    return std::make_pair(b1, b2);
}


} // namespace Pvl
