#include "BipolarSolution.h"
#include "ActuatorCoordinates.h"

#include "checksumm.h"
#include "ConfigValue.h"
#include "libs/Kernel.h"
#include "libs/Config.h"

#include "libs/Kernel.h"
#include "libs/StreamOutput.h"
#include "StreamOutputPool.h"

#define arm_length_checksum          CHECKSUM("arm_length")

BipolarSolution::BipolarSolution(Config* config)
{
    // arm_length is the distance between the two rotational centers
    arm_length = config->value(arm_length_checksum)->by_default(110.0f)->as_number();

    raw_mode = false;
}

void BipolarSolution::cartesian_to_actuator( const float cartesian_mm[], ActuatorCoordinates &actuator_mm )
{
    if(raw_mode)
    {
        actuator_mm[ALPHA_STEPPER] = cartesian_mm[X_AXIS];
        actuator_mm[BETA_STEPPER ] = cartesian_mm[Y_AXIS];
        actuator_mm[GAMMA_STEPPER] = cartesian_mm[Z_AXIS];
    }
    else
    {
        // Handle crossing 180degree barrier
        // https://github.com/unlimitedbacon/BipolarMarlin/commit/510602fa8ccf5816d4a16a956515c80dad9a8d80
        auto thetas = cartesian2bipolar((long double) cartesian_mm[X_AXIS], (long double) cartesian_mm[Y_AXIS]);

        auto theta1_deg = to_degrees(thetas.x);
        if(fabs(theta1_deg - actuator_mm[ALPHA_STEPPER]) > 180)
        {
            THEKERNEL->streams->printf("INSIDE LOOP!");
            if(theta1_deg > 0)
                theta1_deg -= 360;
            else
                theta1_deg += 360;
        }

        actuator_mm[ALPHA_STEPPER] = theta1_deg;
        actuator_mm[BETA_STEPPER ] = to_degrees(thetas.y);
        actuator_mm[GAMMA_STEPPER] = cartesian_mm[Z_AXIS];
    }
}

void BipolarSolution::actuator_to_cartesian( const ActuatorCoordinates &actuator_mm, float cartesian_mm[] )
{
    if(raw_mode)
    {
        cartesian_mm[ALPHA_STEPPER] = actuator_mm[X_AXIS];
        cartesian_mm[BETA_STEPPER ] = actuator_mm[Y_AXIS];
        cartesian_mm[GAMMA_STEPPER] = actuator_mm[Z_AXIS];
    }
    else
    {
        auto carts = bipolar2cartesian(to_radians((long double) actuator_mm[X_AXIS]), to_radians((long double) actuator_mm[Y_AXIS]));

        cartesian_mm[ALPHA_STEPPER] = carts.x;
        cartesian_mm[BETA_STEPPER ] = carts.y;
        cartesian_mm[GAMMA_STEPPER] = actuator_mm[Z_AXIS];
    }
}

bool BipolarSolution::set_optional(const arm_options_t& options)
{
    arm_options_t::const_iterator i;

    // Raw mode
    i = options.find('R');
    if(i != options.end()) {
        raw_mode = (i->second > 0.5);
    }
    return true;
}

bool BipolarSolution::get_optional(arm_options_t& options, bool force_all)
{
    if(raw_mode)
        options['R'] = 1;
    else
        options['R'] = 0;
    return true;
};
