#ifndef VIRTUAL_MAP_H_
#define VIRTUAL_MAP_H_

#include <map>
#include <vector>

#include "ros/ros.h"

#include "tf/tf.h"
#include "geometry_msgs/Polygon.h"
#include "nav_msgs/OccupancyGrid.h"

#include "autolabor_simulation_stage/Obstacle.h"

#define ERROR          -2
#define UNKNOWN_SPACE  -1
#define FREE_SPACE     0
#define OBS_SPACE      100


struct Point
{
  float x;
  float y;
};

typedef vector<Point> footprint;
typedef vector<footprint> footprint_list;
typedef map<string, footprint_list> footprints_map;
typedef map<string, footprint> obstacle_points_map;

class VritualMap
{
public:
  VritualMap();
  ~VritualMap();
};




#endif
