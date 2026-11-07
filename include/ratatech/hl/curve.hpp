#pragma once

#include "ratatech/hl/curve_rank1.hpp"
#include <variant>

namespace detail {

using namespace ratatech::hl;

template <int Rank> struct curve_alternatives;

template <> struct curve_alternatives<1> {
    using type = std::variant<CurveStraight>;
};

} // namespace detail

namespace ratatech::hl {

template <int Rank>
class Curve : public detail::curve_alternatives<Rank>::type {
    using Base = detail::curve_alternatives<Rank>::type;
    using Base::Base;

  public:
    auto start_time() {
        return std::visit([](const auto &x) { return x.start_time(); }, *this);
    }
    auto end_time() {
        return std::visit([](const auto &x) { return x.end_time(); }, *this);
    }
    auto state(float time) {
        return std::visit([time](const auto &x) { return x.state(time); },
                          *this);
    }
};

} // namespace ratatech::hl
