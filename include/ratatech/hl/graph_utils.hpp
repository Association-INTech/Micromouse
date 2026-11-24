#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <ranges>

namespace ratatech::hl {

template <int Rank> struct Coord {
    size_t i, j;

    bool operator==(const Coord &other) const = default;
};

template <int Rank> struct Node {
    std::array<Coord<Rank>, Rank> coordinates;

    explicit Node() = default;
    Node(const std::array<Coord<Rank>, Rank> &coordinates)
        : coordinates{coordinates} {}

    static Node invalid_node() {
        Node node;

        size_t max{std::numeric_limits<size_t>::max()};
        node.coordinates.fill({max, max});

        return node;
    }

    bool operator==(const Node &other) const = default;
};

template <int Rank> struct Edge {
    std::array<Coord<Rank>, Rank + 1> coordinates;
    int cost;

    explicit Edge(const Node<Rank> &source, const Node<Rank> &destination,
                  int cost)
        : cost{cost} {
        // check that the Rank-1 last coordinates of source match teh Rank-1
        // first coordinates of destination just to be sure that the Edge is
        // valid to create
        assert(std::ranges::equal(source.coordinates | std::views::drop(1),
                                  destination.coordinates |
                                      std::views::take(Rank - 1)));

        std::copy(source.coordinates.begin(), source.coordinates.end(),
                  this->coordinates.begin());

        this->coordinates.back() = destination.coordinates.back();
    }

    explicit Edge(const Node<Rank> &source, const Coord<Rank> &last_coordinate,
                  int cost)
        : cost{cost} {
        std::copy(source.coordinates.begin(), source.coordinates.end(),
                  this->coordinates.begin());

        this->coordinates.back() = last_coordinate;
    }

    Node<Rank> source() const {
        std::array<Coord<Rank>, Rank> node_coordinates{};
        std::copy(coordinates.begin(), coordinates.end() - 1,
                  node_coordinates.begin());
        return node_coordinates;
    }

    Node<Rank> destination() const {
        std::array<Coord<Rank>, Rank> node_coordinates{};
        std::copy(coordinates.begin() + 1, coordinates.end(),
                  node_coordinates.begin());
        return node_coordinates;
    }
};

} // namespace ratatech::hl
