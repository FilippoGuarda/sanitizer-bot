#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <chrono>

#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include <uv_visualization/lowestIrradiation.h>
#include <uv_visualization/subMapCoords.h>

#include "uv_utility.h"

#define Power 0.5
#define cleanTreshold 10.0
#define MAP_ODOM_X_DISPLACEMENT -0.0
#define MAP_ODOM_Y_DISPLACEMENT -0.0

using namespace grid_map;

/* Globals variables */
grid_map::GridMap map({"irradiation"});
grid_map::GridMap map1({"irradiation"});
nav_msgs::Odometry turtle_odom;
float map_size_x;
float map_size_y;
float map_position_x;
float map_position_y;
float map_resolution;
float subMap_x1 = 0;
float subMap_y1 = 0;
float subMap_x2 = 0;
float subMap_y2 = 0;
float sub_lowest_irradiation_x;
float sub_lowest_irradiation_y;
float lowest_irradiation_x;
float lowest_irradiation_y;
bool roomDone = true;
bool mapOk = false;
bool tempGoalReached = false;

/*** Callbacks ***/
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  // Update position of the robot
  turtle_odom.pose.pose.position.x = msg->pose.pose.position.x;
  turtle_odom.pose.pose.position.y = msg->pose.pose.position.y;
  turtle_odom.pose.pose.position.z = 0; // always zero
}

void gridMapCallback(const nav_msgs::OccupancyGrid &msg)
{
  ROS_INFO("Received Grid Map");
  // Save a copy of the received map
  grid_map::GridMapRosConverter::fromOccupancyGrid(msg, {"irradiation"}, map);

  mapOk = true;
  map_size_x = map.getSize()(0);
  map_size_y = map.getSize()(1);
  map_position_x = map.getPosition().x();
  map_position_y = map.getPosition().y();
  map_resolution = map.getLength().x() / map.getSize()(0);
}

void subMapCallback(const uv_visualization::subMapCoords::ConstPtr &msg)
{
  if ((subMap_x1 != msg->x1) or (subMap_y1 != msg->y1))
  {
    ROS_INFO("Uv_map received Sub map");
    roomDone = false;
    subMap_x1 = msg->x1;
    subMap_y1 = msg->y1;
    subMap_x2 = msg->x2;
    subMap_y2 = msg->y2;
    ROS_INFO("Sub map size is %f, %f", subMap_x2-subMap_x1, subMap_y2-subMap_y1);
  }
}

/***/

int main(int argc, char **argv)
{
  // Initialize node, publisher and subscriber.
  ros::init(argc, argv, "uv_visualization");
  ros::NodeHandle nh("~");
  ros::Subscriber grid_map_sub_ = nh.subscribe("/map", 1, gridMapCallback);
  ros::Subscriber odom_sub_ = nh.subscribe("/odom", 1, odomCallback);
  ros::Subscriber sub_map_sub = nh.subscribe("/submap", 1, subMapCallback);
  ros::Publisher grid_map_pub_ = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  ros::Publisher lowest_irradiation_pub_ = nh.advertise<uv_visualization::lowestIrradiation>("/lowestIrradiation", 10);

  // Define new map
  map1.setFrameId("map");
  map1.setGeometry(Length(19.2, 19.2), 0.05, Position(-0.4, -0.4));

  // Initialize grid map with all zeros
  grid_map::Matrix &data = map1["irradiation"];
  for (grid_map::GridMapIterator iterator(map1); !iterator.isPastEnd(); ++iterator)
  {
    const int i = iterator.getLinearIndex();
    data(i) = 0;
  }

  Position currentPosition;  
  Position currentSubPosition;   
  Position miniPosition;          // store current robot position for the circles and sub iterator
  const double radius = 19.2;     // radius for the circle iterator
  const int obstacle_value = 100;  // value for a cell obstructed by a wall

  float distance;
  bool obstacleFound = false; // if true an obstacle is found between two cell
  float tempPower = 0;
  float minPower = 0;
  float newMinPower = cleanTreshold;
  long int deltaT; // loop time in [ms]
  float deltaT_s;  // loop time in [s]

  auto End = std::chrono::steady_clock::now();
  auto Start = std::chrono::steady_clock::now();

  ros::Rate rate(20);

  while (nh.ok())
  {

    if (mapOk == true)
    {
      End = std::chrono::steady_clock::now();

      deltaT = std::chrono::duration_cast<std::chrono::milliseconds>(End - Start).count();
      deltaT_s = float(deltaT) / 1000;

      Start = std::chrono::steady_clock::now();

      // ROS_INFO("Delta T: %f", deltaT_s);
      // Circle iterator specifications
      Position center(turtle_odom.pose.pose.position.x - MAP_ODOM_X_DISPLACEMENT, turtle_odom.pose.pose.position.y - MAP_ODOM_Y_DISPLACEMENT);

      for (grid_map::CircleIterator iterator(map, center, radius); !iterator.isPastEnd(); ++iterator)
      {

        // get cell position
        map.getPosition(*iterator, currentPosition);

        obstacleFound = false;

        if (sqrt(pow(center.x() - currentPosition.x(), 2) + pow(center.y() - currentPosition.y(), 2)) >= 0.11)
        {

          // These are in the gridmap reference frame because we check obstacles in a map transleted of 0.4 0.4 wrt the map reference frame.
          Index start(round(-(center.x() - map_position_x) / map_resolution) + map_size_x / 2, round(-(center.y() - map_position_y) / map_resolution) + map_size_y / 2);
          Index end(round(-(currentPosition.x() - map_position_x) / map_resolution) + map_size_x / 2, round(-(currentPosition.y() - map_position_y) / map_resolution) + map_size_y / 2);

          // Line iterator
          for (grid_map::LineIterator iterator1(map, start, end); !iterator1.isPastEnd(); ++iterator1)
          {

            // Obstacle Check
            if (map.at("irradiation", *iterator1) == obstacle_value)
              obstacleFound = true;

            // If obstacle is found exit the line iterator
            if (obstacleFound == true)
              break;
          }

          // Update Energy
          if ((obstacleFound == false) && (map.at("irradiation", *iterator) != obstacle_value))
          {

            distance = pow(turtle_odom.pose.pose.position.x - currentPosition.x(), 2) + pow(turtle_odom.pose.pose.position.y - currentPosition.y(), 2);

            tempPower = map1.atPosition("irradiation", currentPosition) + (Power * (deltaT_s)) / distance;

            if (tempPower >= cleanTreshold)
            {
              map1.atPosition("irradiation", currentPosition) = cleanTreshold;
            }
            else
            {
              map1.atPosition("irradiation", currentPosition) = tempPower;
            }
          }
        }
      } 

      if(not roomDone)
      {
        // find less irradiated point in the room
        float BufferSize_x = (subMap_x2 - subMap_x1);
        float BufferSize_y = (subMap_y2 - subMap_y1);
        int iterationCounter = 0;
        bool feasibleGoal = true;
        float distanceFromCentre = 0;
        Index submapStartIndex(round(-(subMap_x2-map_position_x)/map_resolution)+ map_size_x / 2, round(-(subMap_y2-map_position_y)/map_resolution)+map_size_y / 2);
        Index submapBufferSize(round(BufferSize_x/map_resolution), round(BufferSize_y/map_resolution));
        newMinPower = cleanTreshold;
        for (grid_map::SubmapIterator subIterator(map1, submapStartIndex, submapBufferSize); !subIterator.isPastEnd(); ++subIterator)
        {
          map1.getPosition(*subIterator, currentSubPosition);
          //map1.atPosition("irradiation", currentSubPosition) = map1.atPosition("irradiation", currentSubPosition) + 0.01;

          feasibleGoal = true;
          Position miniCenter(currentSubPosition);
          //Position miniCentre((-(subMap_x2-map_position_x)/map_resolution+ map_size_x / 2, -(subMap_y2-map_position_y)/map_resolution+map_size_y / 2));
          for (grid_map::CircleIterator miniCircle(map1, miniCenter, 0.40); !miniCircle.isPastEnd(); ++miniCircle)
          {
            map1.getPosition(*miniCircle, miniPosition);
            distanceFromCentre = (sqrt(pow(center.x() - miniPosition.x(), 2) + pow(center.y() - miniPosition.y(), 2)));
            if((distanceFromCentre <= 0.11) or (map.atPosition("irradiation", miniPosition) == obstacle_value))
            {
              feasibleGoal = false;
              //ROS_INFO("too close");
              break;
            }
            // else
            // {
            //   map1.atPosition("irradiation", miniPosition) += 0.1;
            // }
          }

          // if (feasibleGoal)
          // {
          //   map1.atPosition("irradiation", currentSubPosition) = map1.atPosition("irradiation", currentSubPosition) + 0.1;
          // }
          //ROS_INFO("current power %f", (map1.atPosition("irradiation", currentPosition)));
          if ((map1.atPosition("irradiation", currentSubPosition) < newMinPower) and feasibleGoal)
          {
            newMinPower = map1.atPosition("irradiation", currentSubPosition);
            sub_lowest_irradiation_x = currentSubPosition.x();
            sub_lowest_irradiation_y = currentSubPosition.y();
          }
        }
        if (minPower < newMinPower)
        {
            minPower = newMinPower;
            // lowest_irradiation_x = sub_lowest_irradiation_x*map_resolution - (subMap_x1+subMap_x2)/2;
            // lowest_irradiation_y = sub_lowest_irradiation_y*map_resolution - (subMap_y1+subMap_y2)/2;
            lowest_irradiation_x = sub_lowest_irradiation_x;
            lowest_irradiation_y = sub_lowest_irradiation_y;
            ROS_INFO("lowest irradiation updated = %f. at %f,%f", minPower, lowest_irradiation_x, lowest_irradiation_y);
            ROS_INFO("%d", iterationCounter);
        }
        if (minPower >= cleanTreshold)
        {
          ROS_INFO("this room is done");
          roomDone = true;
          minPower = 0;
        }
      }
    }

    ros::Time time = ros::Time::now();
    map1.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map1, message);
    grid_map_pub_.publish(message);
    uv_visualization::lowestIrradiation irCoords;
    irCoords.lowest_x = lowest_irradiation_x;
    irCoords.lowest_y = lowest_irradiation_y;
    irCoords.room_done = roomDone;
    lowest_irradiation_pub_.publish(irCoords);
    // SpinOnce to run callbacks
    ros::spinOnce();
    // Wait for next cycle.
    rate.sleep();
  }

  return 0;
}


// #include <ros/ros.h>
// #include <grid_map_ros/grid_map_ros.hpp>
// #include <grid_map_msgs/GridMap.h>
// #include <cmath>

// using namespace grid_map;

// int main(int argc, char** argv)
// {
//   // Initialize node and publisher.
//   ros::init(argc, argv, "grid_map_simple_demo");
//   ros::NodeHandle nh("~");
//   ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

//   // Create grid map.
//   GridMap map({"irradiation"});
//   map.setFrameId("map");
//   map.setGeometry(Length(1.2, 2.0), 0.03);
//   ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
//     map.getLength().x(), map.getLength().y(),
//     map.getSize()(0), map.getSize()(1));

//   // Work with grid map in a loop.
//   ros::Rate rate(30.0);
//   while (nh.ok()) {

//     // Add data to grid map.
//     ros::Time time = ros::Time::now();
//     for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
//       Position position;
//       map.getPosition(*it, position);
//       map.at("irradiation", *it) = -0.04 + 0.2 * std::sin(3.0 * time.toSec() + 5.0 * position.y()) * position.x();
//     }

//     // Publish grid map.
//     map.setTimestamp(time.toNSec());
//     grid_map_msgs::GridMap message;
//     GridMapRosConverter::toMessage(map, message);
//     publisher.publish(message);
//     ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

//     // Wait for next cycle.
//     rate.sleep();
//   }

//   return 0;
// }
