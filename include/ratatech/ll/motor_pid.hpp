#pragma once

#include "stm32f0xx.h"
#include "stm32f0xx_hal_tim.h"

namespace ratatech::ll {

class MotorPID {
    struct Velocity {
        float vx;
        float vy;
    };

  public:
    explicit MotorPID(TIM_HandleTypeDef *pwm_htim, float Kp, float Ki,
                      float Kd);
    float update(float current_speed, float target_speed, float dt);

  private:
    TIM_HandleTypeDef *pwm_htim_;
    float Kp_, Ki_, Kd_;
    float integral_error_, previous_error_;
};

} // namespace ratatech::ll
