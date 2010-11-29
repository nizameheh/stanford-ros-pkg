/*
 * Project 3
 * Team Win: Arne Bech, Brian Chung, Robert Kanter, Pierre Kreitmann
 */

#ifndef PUBLISHER_H
#define PUBLISHER_H

#include "pose.h"
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "LinearMath/btTransform.h"
#include "ros/time.h"
#include "ros/console.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "map.h"
#include "string.h"


/*
 * This class contains functions used for publishing data to rviz
 */
class Publisher
{
    public:
        static void publishPoint(ros::Publisher marker_pub, Pose* pose, std::string frameID, std::string nameS, int id);
        static void publishPoint2(ros::Publisher marker_pub, Pose* pose, std::string frameID, std::string nameS, int id);
        static void publishPDF(ros::Publisher marker_pub, std::vector<Pose*> poses, std::string frameID);
        static void publishArrowsPDF(ros::Publisher posePublisher, std::vector<Pose*> poses, std::string frameID, float offset);
        static void publishRobot(ros::Publisher marker_pub);
        static void publishMap(const Map & map, const ros::Publisher & publisher, const char* nameSpace, std::string frameID);
        static std_msgs::ColorRGBA colorScale(double x);
        

};

#endif
