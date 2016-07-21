/*
    Author: Emil Madsen (sovende@gmail.com)
    License: GPL3 or better see <http://www.gnu.org/licenses/>

    Summary
    -------
    Probes three user specified points on the bed and determines the average of those.

    Configuration
    -------------
    The strategy must be enabled in the config as well as zprobe.

    leveling-strategy.average-leveling.enable         true

    Three probe points must be defined, these are best if they are as far apart as possible.
    They can be defined in the config file as:-

    leveling-strategy.average-leveling.probe_points   3
    leveling-strategy.average-leveling.point1         100.0,0.0   # the first probe point (x,y)
    leveling-strategy.average-leveling.point2         200.0,200.0 # the second probe point (x,y)
    leveling-strategy.average-leveling.point3         0.0,200.0   # the third probe point (x,y)

    or they may be defined using M557 P0 X30 Y40.5  where P is 0,1,2,...,probe_points

    probe offsets from the nozzle or tool head can be defined with

    leveling-strategy.average-leveling.probe_offsets  0,0,0  # probe offsetrs x,y,z

    they may also be set with M565 X0 Y0 Z0

    Usage
    -----
    G32 probes the three probe points and defines the average
    G31 reports the status

    M557 defines the probe points
    M565 defines the probe offsets from the nozzle or tool head
*/

#include "AverageStrategy.h"
#include "Kernel.h"
#include "Config.h"
#include "Robot.h"
#include "StreamOutputPool.h"
#include "Gcode.h"
#include "checksumm.h"
#include "utils.h"
#include "ConfigValue.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "Conveyor.h"
#include "ZProbe.h"
#include "nuts_bolts.h"

#include <string>
#include <algorithm>
#include <cstdlib>
#include <sstream>
#include <cmath>

#define probe_points_checksum        CHECKSUM("probe_points")
#define probe_offsets_checksum       CHECKSUM("probe_offsets")
#define tolerance_checksum           CHECKSUM("tolerance")
#define point_tolerance_checksum     CHECKSUM("point_tolerance")
#define point_rejection_checksum     CHECKSUM("point_rejection")

AverageStrategy::AverageStrategy(ZProbe *zprobe) : LevelingStrategy(zprobe)
{
}

AverageStrategy::~AverageStrategy()
{
}

template<typename T>
std::string number_to_string(T Number)
{
    ostringstream ss;
    ss << Number;
    return ss.str();
}

bool AverageStrategy::handleConfig()
{
    std::string points = THEKERNEL->config->value(leveling_strategy_checksum, average_leveling_strategy_checksum, probe_points_checksum)->by_default("3")->as_string();

    unsigned num_probe_points = atoi(points.c_str());

    // Setup vector size, and allocate nans
    probe_points.resize(num_probe_points);
    //probe_points.resize(8);
    for(unsigned int i = 0; i < probe_points.size(); ++i) 
    {
        probe_points[i] = std::make_tuple(NAN, NAN);
    }
 
    // Read in the points
    for(unsigned int i = 0; i < probe_points.size(); ++i) 
    {
        std::string point_string = "point";
        point_string += number_to_string(i);
        // format is xxx,yyy for the probe points
        std::string point = THEKERNEL->config->value(leveling_strategy_checksum, average_leveling_strategy_checksum, get_checksum(point_string.c_str()))->by_default("")->as_string();
        if(!point.empty())
        {
            probe_points[i] = parseXY(point.c_str());
        }
        else
        {
            THEKERNEL->streams->printf("Configuration error; Missing zprobe point: %d", i);
        }
    }

    // Probe offsets xxx,yyy,zzz
    std::string po = THEKERNEL->config->value(leveling_strategy_checksum, average_leveling_strategy_checksum, probe_offsets_checksum)->by_default("0,0,0")->as_string();
    this->probe_offsets= parseXYZ(po.c_str());

    std::string pt = THEKERNEL->config->value(leveling_strategy_checksum, average_leveling_strategy_checksum, point_tolerance_checksum)->by_default("0.1")->as_string();
    this->point_tolerance= atof(pt.c_str());

    std::string prj = THEKERNEL->config->value(leveling_strategy_checksum, average_leveling_strategy_checksum, point_rejection_checksum)->by_default("3")->as_string();
    this->point_rejection= atoi(prj.c_str());

    std::string tol = THEKERNEL->config->value(leveling_strategy_checksum, average_leveling_strategy_checksum, tolerance_checksum)->by_default("0.1")->as_string();
    this->tolerance= atof(tol.c_str());

    return true;
}

bool AverageStrategy::handleGcode(Gcode *gcode)
{
    if(gcode->has_g) {
        if( gcode->g == 31 ) { // report status
            gcode->stream->printf("Probe is %s\n", zprobe->getProbeStatus() ? "Triggered" : "Not triggered");
            gcode->stream->printf("Average: %f\n", average);
            return true;

        } else if( gcode->g == 32 ) { // three point probe
            // first wait for an empty queue i.e. no moves left
            THEKERNEL->conveyor->wait_for_idle();

            if(!doProbing(gcode->stream)) {
                gcode->stream->printf("Probe failed to complete, probe not triggered or other error\n");
            } else {
                gcode->stream->printf("Probe completed, average found\n");
            }
            return true;
        }
    } else if(gcode->has_m) {
        if(gcode->m == 557) { // M557 - set probe points eg M557 P0 X30 Y40.5  where P is 0,1,2
            int idx = 0;
            float x = NAN, y = NAN;
            if(gcode->has_letter('P')) idx = gcode->get_value('P');
            if(gcode->has_letter('X')) x = gcode->get_value('X');
            if(gcode->has_letter('Y')) y = gcode->get_value('Y');
            if(idx >= 0 && idx < (int) probe_points.size()) {
                probe_points[idx] = std::make_tuple(x, y);
            }else{
                 gcode->stream->printf("Only %d probe points configured!\n", probe_points.size());
            }
            return true;

        } else if(gcode->m == 558) { // M558 - get probe points eg M558 P0 where P is 0,1,2
            int idx = 0;
            if(gcode->has_letter('P')) idx = gcode->get_value('P');
 
            if(idx >= 0 && idx < (int) probe_points.size()) {
                auto point = probe_points[idx];

                gcode->stream->printf("%f, %f\n",
                    std::get<X_AXIS>(point),
                    std::get<Y_AXIS>(point));
            }else{
                 gcode->stream->printf("Only %d probe points configured!\n", probe_points.size());
            }
            return true;
        } else if(gcode->m == 559) { // M559 - set number of probe points
            int idx = -1;
            if(gcode->has_letter('N')) idx = gcode->get_value('N');
 
            if(idx >= 0) {
                int size = probe_points.size();
                probe_points.resize(idx);
                for(unsigned int i = size; i < probe_points.size(); ++i) 
                {
                    probe_points[i] = std::make_tuple(NAN, NAN);
                }
            }else{
                 gcode->stream->printf("Unable to configure number of probe points to %d\n", probe_points.size());
            }
            return true;

        } else if(gcode->m == 560) { // M560 - get number of probe points
            gcode->stream->printf("%d\n", probe_points.size());
            return true;
     
        } else if(gcode->m == 565) { // M565: Set Z probe offsets
            float x= 0, y= 0, z= 0;
            if(gcode->has_letter('X')) x = gcode->get_value('X');
            if(gcode->has_letter('Y')) y = gcode->get_value('Y');
            if(gcode->has_letter('Z')) z = gcode->get_value('Z');
            probe_offsets = std::make_tuple(x, y, z);
            return true;
        } else if(gcode->m == 566) { // M566: Get Z probe offsets
            gcode->stream->printf("%f, %f, %f\n",
                std::get<X_AXIS>(this->probe_offsets),
                std::get<Y_AXIS>(this->probe_offsets),
                std::get<Z_AXIS>(this->probe_offsets));
          
            return true;
        }
    }

    return false;
}

bool AverageStrategy::doProbing(StreamOutput *stream)
{
    float x, y;
    // check the probe points have been defined
    for (unsigned int i = 0; i < probe_points.size(); ++i) {
        std::tie(x, y) = probe_points.at(i);
        if(isnan(x) || isnan(y)) {
            stream->printf("Probe point P%d has not been defined, use M557 P%d Xnnn Ynnn to define it\n", i, i);
            return false;
        }
    }

    // move to the first probe point
    std::tie(x, y) = probe_points.at(0);
    // offset by the probe XY offset
    x -= std::get<X_AXIS>(this->probe_offsets);
    y -= std::get<Y_AXIS>(this->probe_offsets);
    zprobe->coordinated_move(x, y, NAN, zprobe->getFastFeedrate());

    // for now we use probe to find bed and not the Z min endstop
    // the first probe point becomes Z == 0 effectively so if we home Z or manually set z after this, it needs to be at the first probe point

    // TODO this needs to be configurable to use min z or probe

    // find bed via probe
    float mm;
    if(!zprobe->run_probe(mm)) return false;

    // TODO if using probe then we probably need to set Z to 0 at first probe point, but take into account probe offset from head
    THEROBOT->reset_axis_position(/*std::get<Z_AXIS>(this->probe_offsets)*/0, Z_AXIS);

    // move up to specified probe start position
    zprobe->coordinated_move(NAN, NAN, zprobe->getProbeHeight(), zprobe->getSlowFeedrate()); // move to probe start position

    // probe the three points
    std::vector<float> z_values;
    for (unsigned int i = 0; i < probe_points.size(); ++i) {
        //for(unsigned int q = 0; q < 8; q++)
        {
            std::tie(x, y) = probe_points[i];
            // offset moves by the probe XY offset
            float z = zprobe->probeDistance(x-std::get<X_AXIS>(this->probe_offsets), y-std::get<Y_AXIS>(this->probe_offsets));
            if(isnan(z)) return false; // probe failed
            z = zprobe->getProbeHeight() - z; // relative distance between the probe points, lower is negative z
            stream->printf("DEBUG: P%d:%1.4f\n", i, z);
            z_values.push_back(z);
        }
    }
    
    // TODO: Meanian and bad values
    /*
    stream->printf("Got: %d readings\n", z_values.size());
    z_values.erase(std::remove_if(z_values.begin(), z_values.end(), [&stream, this](float f)
    {
        if(fabs(f) > point_tolerance)
        {
            stream->printf("Removed: %1.4f\n", f);
            return true;
        }
        else
        {
            return false;
        }
    }), z_values.end());
    stream->printf("Got: %d readings\n", z_values.size());

    if(z_values.size() < probe_points.size() - point_rejection)
    {
        stream->printf("Too many bad points!\n");
        stream->printf("%d < %d - %d\n", z_values.size(), probe_points.size(), point_rejection);
        return false;
    }
*/
    for (float element : z_values)
    {
        if(fabs(element) > point_tolerance)
        {
            stream->printf("Dangerous outlier\n");
            return false;
        }
    }

    //average = std::accumulate(z_values.begin(), z_values.end(), 0.0) / z_values.size();
    //stream->printf("Average: %f\n", average);
    
    average = median(z_values);
    stream->printf("Average: %f\n", average);

    if(fabs(average) > tolerance)
    {
        stream->printf("Average higher than tolerance: %f\n", tolerance);
        return false;
    }

    THEROBOT->reset_axis_position(std::get<Z_AXIS>(this->probe_offsets)+average, Z_AXIS);
    stream->printf("Z position: %f\n", THEROBOT->get_axis_position(Z_AXIS));
    return true;
}

// parse a "X,Y" string return x,y
std::tuple<float, float> AverageStrategy::parseXY(const char *str)
{
    float x = NAN, y = NAN;
    char *p;
    x = strtof(str, &p);
    if(p + 1 < str + strlen(str)) {
        y = strtof(p + 1, nullptr);
    }
    return std::make_tuple(x, y);
}

// parse a "X,Y,Z" string return x,y,z tuple
std::tuple<float, float, float> AverageStrategy::parseXYZ(const char *str)
{
    float x = 0, y = 0, z= 0;
    char *p;
    x = strtof(str, &p);
    if(p + 1 < str + strlen(str)) {
        y = strtof(p + 1, &p);
        if(p + 1 < str + strlen(str)) {
            z = strtof(p + 1, nullptr);
        }
    }
    return std::make_tuple(x, y, z);
}
