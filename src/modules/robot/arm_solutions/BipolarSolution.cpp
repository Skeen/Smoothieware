#include "BipolarSolution.h"
#include "ActuatorCoordinates.h"

#include "checksumm.h"
#include "ConfigValue.h"
#include "libs/Kernel.h"
#include "libs/Config.h"

#define arm_length_checksum          CHECKSUM("arm_length")

BipolarSolution::BipolarSolution(Config* config)
{
    // arm_length is the distance between the two rotational centers
    arm_length = config->value(arm_length_checksum)->by_default(110.0f)->as_number();
}

void BipolarSolution::cartesian_to_actuator( const float cartesian_mm[], ActuatorCoordinates &actuator_mm )
{
    // Handle crossing 180degree barrier
    // https://github.com/unlimitedbacon/BipolarMarlin/commit/510602fa8ccf5816d4a16a956515c80dad9a8d80
    auto thetas = cartesian2bipolar((long double) cartesian_mm[X_AXIS], (long double) cartesian_mm[Y_AXIS]);

    auto theta1_deg = to_degrees(thetas.x);
    if(abs(theta1_deg - actuator_mm[ALPHA_STEPPER]) > 180)
    {
        if(theta1_deg > 0)
            theta1_deg -= 360;
        else
            theta1_deg += 360;
    }

    actuator_mm[ALPHA_STEPPER] = theta1_deg;
    actuator_mm[BETA_STEPPER ] = to_degrees(thetas.y);
    actuator_mm[GAMMA_STEPPER] = cartesian_mm[Z_AXIS];
}

void BipolarSolution::actuator_to_cartesian( const ActuatorCoordinates &actuator_mm, float cartesian_mm[] )
{
    auto carts = bipolar2cartesian(to_radians((long double) actuator_mm[X_AXIS]), to_radians((long double) actuator_mm[Y_AXIS]));

    cartesian_mm[ALPHA_STEPPER] = carts.x;
    cartesian_mm[BETA_STEPPER ] = carts.y;
    cartesian_mm[GAMMA_STEPPER] = actuator_mm[Z_AXIS];
}
