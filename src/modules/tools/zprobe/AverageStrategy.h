#ifndef _AVERAGESTRATEGY
#define _AVERAGESTRATEGY

#include "LevelingStrategy.h"

#include <string.h>
#include <tuple>
#include <vector>

#define average_leveling_strategy_checksum CHECKSUM("average-leveling")

class StreamOutput;
class Plane3D;

template<typename T>
T median(std::vector<T>& v)
{
    size_t n = v.size() / 2;
    nth_element(v.begin(), v.begin()+n, v.end());
    return v[n];
}

class AverageStrategy : public LevelingStrategy
{
public:
    AverageStrategy(ZProbe *zprobe);
    ~AverageStrategy();
    bool handleGcode(Gcode* gcode);
    bool handleConfig();

private:
    bool doProbing(StreamOutput *stream);
    std::tuple<float, float> parseXY(const char *str);
    std::tuple<float, float, float> parseXYZ(const char *str);

    // The points we need to probe
    std::vector<std::tuple<float, float>> probe_points;
    // The offset of the probe
    std::tuple<float, float, float> probe_offsets;

    float average;

    float tolerance;
    float point_tolerance;

    int point_rejection;
};

#endif
