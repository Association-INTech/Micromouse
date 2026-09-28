

#include "ratatech/hl/trajectory.hpp"
namespace ratatech::ll {

class Ramsete {
  public:
    explicit Ramsete(float b, float zeta);

    hl::StateVector update(hl::StateVector trajectory_state);

  private:
    float b, zeta;
};

} // namespace ratatech::ll
