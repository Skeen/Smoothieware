#include "BipolarSolution.h"
#include "ActuatorCoordinates.h"
#include <math.h>

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
    actuator_mm[ALPHA_STEPPER] = cartesian_mm[X_AXIS];
    actuator_mm[BETA_STEPPER ] = cartesian_mm[Y_AXIS];
    actuator_mm[GAMMA_STEPPER] = cartesian_mm[Z_AXIS];
}

void BipolarSolution::actuator_to_cartesian( const ActuatorCoordinates &actuator_mm, float cartesian_mm[] )
{
    cartesian_mm[ALPHA_STEPPER] = actuator_mm[X_AXIS];
    cartesian_mm[BETA_STEPPER ] = actuator_mm[Y_AXIS];
    cartesian_mm[GAMMA_STEPPER] = actuator_mm[Z_AXIS];
}

float BipolarSolution::to_degrees(float radians)
{
    return radians * (180.0f / M_PI);
}

float BipolarSolution::to_radians(float degrees)
{
    return degrees * (M_PI / 180.0f);
}

Coord2D BipolarSolution::polar2cartesian(float theta, float r)
{
    float x = r * cos(theta);
    float y = r * sin(theta);
    return Coord2D(x,y);
}

Coord2D BipolarSolution::bipolar2cartesian(float theta1, float theta2)
{
    float theta = ((M_PI - theta2)/2) - theta1;
    float r = 2 * arm_length * sin(theta2/2);
    return polar2cartesian(theta,r);
}

Coord2D BipolarSolution::cartesian2polar(float x, float y)
{
    float r = sqrt(x*x + y*y);
    float theta = atan2(y,x);
    return Coord2D(theta,r);
}

Coord2D BipolarSolution::cartesian2bipolar(float x, float y)
{
    auto choord = cartesian2polar(x,y);

    float theta = choord.x;
    float r = choord.y;

    float th1 = acos( r / (2* arm_length) ) - theta;
    float th2 = 2 * asin( r / (2* arm_length) );
    return Coord2D(th1,th2);
}
