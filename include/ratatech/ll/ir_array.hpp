#pragma once

#include "stm32f0xx.h"
#include "stm32f0xx_hal_tim.h"
#include <array>
#include <cstddef>
#include <cstdint>

namespace ratatech::ll {

// Abstraction for the IR LED/Phototransistor pair used to estimate
// distances to the walls.
class IRArray {
    static constexpr std::size_t n_sensors{5};

  public:
    explicit IRArray(TIM_HandleTypeDef *htim);

    const std::array<uint32_t, n_sensors> &readings() const;
    void enable_emitters();
    void disable_emitters();

  private:
    std::array<uint32_t, n_sensors> readings_;
    TIM_HandleTypeDef *htim_;
};

} // namespace ratatech::ll
