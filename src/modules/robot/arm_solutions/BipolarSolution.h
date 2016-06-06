#ifndef BIPOLARSOLUTION_H
#define BIPOLARSOLUTION_H
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "BaseSolution.h"
#include "libs/nuts_bolts.h"

#include "libs/Config.h"

struct Coord2D
{
    Coord2D(float x, float y)
        : x(x), y(y)
    {
    }

    float x;
    float y;
};

class BipolarSolution : public BaseSolution {
    public:
        BipolarSolution(){};
        BipolarSolution(Config*);
        void cartesian_to_actuator( const float millimeters[], ActuatorCoordinates &steps ) override;
        void actuator_to_cartesian( const ActuatorCoordinates &steps, float millimeters[] ) override;

    // Convert between degrees and radians
    float to_degrees(float radians);
    float to_radians(float degrees);

    /**
     * Convert polar coordinates
     * into cartesian coordinates.
     * All angles are measured in RADIANS
     */
    Coord2D polar2cartesian(float theta, float r);

    /*
     * Convert a set of bipolar coordinates
     * into cartesian coordinates.
     * Whereas polar coordinates are represented by an angle and a distance,
     * bipolar coordinates are represented by two angles. 
     */
    Coord2D bipolar2cartesian(float theta1, float theta2);

    /*
     * Convert a set of cartesian coordinatesS
     * into polar coordinates
     */
    Coord2D cartesian2polar(float x, float y);

    /*
     * Convert a set of cartesian coordinates (X,Y)
     * into bipolar coordinates (Theta_1,Theta_2) represented by two angles,
     * the angle of the platter and the angle of the arm.
     */
    Coord2D cartesian2bipolar(float x, float y);

    private:
        float arm_length;
};

#endif
