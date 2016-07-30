#include "BipolarSolution.h"
#include "ActuatorCoordinates.h"

#include "checksumm.h"
#include "ConfigValue.h"
#include "libs/Kernel.h"
#include "libs/Config.h"

#define ARC_ANGULAR_TRAVEL_EPSILON 5E-7F // Float (radians)
#define PI 3.14159265358979323846F // force to be float, do not use M_PI
#define arm_length_checksum          CHECKSUM("arm_length")

BipolarSolution::BipolarSolution(Config* config)
{
    // arm_length is the distance between the two rotational centers
    arm_length = config->value(arm_length_checksum)->by_default(110.0f)->as_number();
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
		auto thetas = cartesian2bipolar((long double) cartesian_mm[X_AXIS], (long double) cartesian_mm[Y_AXIS]);

		actuator_mm[ALPHA_STEPPER] = to_degrees(thetas.x);
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
		auto carts = bipolar2cartesian((long double) actuator_mm[X_AXIS], (long double) actuator_mm[Y_AXIS]));

		cartesian_mm[ALPHA_STEPPER] = to_radians(carts.x);
		cartesian_mm[BETA_STEPPER ] = to_radians(carts.y);
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
