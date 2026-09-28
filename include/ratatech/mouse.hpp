#pragma once

#include "ratatech/hl/graph.hpp"
#include "ratatech/hl/maze.hpp"
#include "ratatech/hl/trajectory.hpp"

namespace ratatech {

class Mouse {
  public:
    explicit Mouse();

    void start();
    void update();

  private:
    hl::Maze maze_;
    hl::Graph graph_;
    hl::Trajectory trajectory_;
};

} // namespace ratatech
