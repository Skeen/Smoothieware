#ifndef BIPOLARSOLUTION_H
#define BIPOLARSOLUTION_H
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "BaseSolution.h"
#include "libs/nuts_bolts.h"

#include "libs/Config.h"

#include <math.h>

template<typename floating>
struct Coord2D
{
    Coord2D(floating x, floating y)
        : x(x), y(y)
    {
    }

    floating x;
    floating y;
};

class BipolarSolution : public BaseSolution
{
public:
    BipolarSolution(){};
    BipolarSolution(Config*);
    void cartesian_to_actuator( const float millimeters[], ActuatorCoordinates &steps ) override;
    void actuator_to_cartesian( const ActuatorCoordinates &steps, float millimeters[] ) override;

    // Convert between degrees and radians
    template<typename floating>
    floating to_degrees(floating radians)
    {
        return radians * (floating(180) / M_PI);
    }

    template<typename floating>
    floating to_radians(floating degrees)
    {
        return degrees * (M_PI / floating(180));
    }

    /**
     * Convert polar coordinates
     * into cartesian coordinates.
     * All angles are measured in RADIANS
     */
    template<typename floating>
    Coord2D<floating> polar2cartesian(floating theta, floating r)
    {
        floating x = r * cos(theta);
        floating y = r * sin(theta);
        return Coord2D<floating>(x,y);
    }

    /*
     * Convert a set of bipolar coordinates
     * into cartesian coordinates.
     * Whereas polar coordinates are represented by an angle and a distance,
     * bipolar coordinates are represented by two angles. 
     */
    template<typename floating>
    Coord2D<floating> bipolar2cartesian(floating theta1, floating theta2)
    {
        floating theta = ((M_PI - theta2)/2) - theta1;
        floating r = 2 * arm_length * sin(theta2/2);
        return polar2cartesian(theta,r);
    }

    /*
     * Convert a set of cartesian coordinatesS
     * into polar coordinates
     */
    template<typename floating>
    Coord2D<floating> cartesian2polar(floating x, floating y)
    {
        floating r = sqrt(x*x + y*y);
        floating theta = atan2(y,x);
        return Coord2D<floating>(theta,r);
    }

    /*
     * Convert a set of cartesian coordinates (X,Y)
     * into bipolar coordinates (Theta_1,Theta_2) represented by two angles,
     * the angle of the platter and the angle of the arm.
     */
    template<typename floating>
    Coord2D<floating> cartesian2bipolar(floating x, floating y)
    {
        floating theta2 = 2 * asin( sqrt(x*x + y*y) / (2*arm_length) );
        floating theta1 = (M_PI-theta2)/2 - atan2(y,x);

        return Coord2D<floating>(theta1, theta2);
        /*
        auto choord = cartesian2polar(x,y);

        float theta = choord.x;
        float r = choord.y;

        float th1 = acos( r / (2* arm_length) ) - theta;
        float th2 = 2 * asin( r / (2* arm_length) );
        return Coord2D(th1,th2);
        */
    }

    bool set_optional(const arm_options_t& options) override;
    bool get_optional(arm_options_t& options, bool force_all) override;

    private:
        long double arm_length;
        bool raw_mode;
};

#endif
