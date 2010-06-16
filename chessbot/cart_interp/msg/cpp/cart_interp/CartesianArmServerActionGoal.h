/* auto-generated by genmsg_cpp from /home/davidmandle/rospackages/cart_interp/msg/CartesianArmServerActionGoal.msg.  Do not edit! */
#ifndef CART_INTERP_CARTESIANARMSERVERACTIONGOAL_H
#define CART_INTERP_CARTESIANARMSERVERACTIONGOAL_H

#include <string>
#include <vector>
#include "ros/message.h"
#include "ros/debug.h"
#include "ros/assert.h"
#include "ros/time.h"

#include "roslib/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "cart_interp/CartesianArmServerGoal.h"

namespace cart_interp
{

//! \htmlinclude CartesianArmServerActionGoal.msg.html

class CartesianArmServerActionGoal : public ros::Message
{
public:
  typedef boost::shared_ptr<CartesianArmServerActionGoal> Ptr;
  typedef boost::shared_ptr<CartesianArmServerActionGoal const> ConstPtr;

  typedef roslib::Header _header_type;
  typedef actionlib_msgs::GoalID _goal_id_type;
  typedef cart_interp::CartesianArmServerGoal _goal_type;

  roslib::Header header;
  actionlib_msgs::GoalID goal_id;
  cart_interp::CartesianArmServerGoal goal;

  CartesianArmServerActionGoal() : ros::Message()
  {
  }
  CartesianArmServerActionGoal(const CartesianArmServerActionGoal &copy) : ros::Message(),
    header(copy.header),
    goal_id(copy.goal_id),
    goal(copy.goal)
  {
    (void)copy;
  }
  CartesianArmServerActionGoal &operator =(const CartesianArmServerActionGoal &copy)
  {
    if (this == &copy)
      return *this;
    header = copy.header;
    goal_id = copy.goal_id;
    goal = copy.goal;
    return *this;
  }
  virtual ~CartesianArmServerActionGoal() 
  {
  }
  inline static std::string __s_getDataType() { return std::string("cart_interp/CartesianArmServerActionGoal"); }
  inline static std::string __s_getMD5Sum() { return std::string("8188ffc0ebe10e91044960a5c85b373a"); }
  inline static std::string __s_getMessageDefinition()
  {
    return std::string(
    "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
    "\n"
    "Header header\n"
    "actionlib_msgs/GoalID goal_id\n"
    "CartesianArmServerGoal goal\n"
    "\n"
    "================================================================================\n"
    "MSG: roslib/Header\n"
    "# Standard metadata for higher-level stamped data types.\n"
    "# This is generally used to communicate timestamped data \n"
    "# in a particular coordinate frame.\n"
    "# \n"
    "# sequence ID: consecutively increasing ID \n"
    "uint32 seq\n"
    "#Two-integer timestamp that is expressed as:\n"
    "# * stamp.secs: seconds (stamp_secs) since epoch\n"
    "# * stamp.nsecs: nanoseconds since stamp_secs\n"
    "# time-handling sugar is provided by the client library\n"
    "time stamp\n"
    "#Frame this data is associated with\n"
    "# 0: no frame\n"
    "# 1: global frame\n"
    "string frame_id\n"
    "\n"
    "================================================================================\n"
    "MSG: actionlib_msgs/GoalID\n"
    "# The stamp should store the time at which this goal was requested.\n"
    "# It is used by an action server when it tries to preempt all\n"
    "# goals that were requested before a certain time\n"
    "time stamp\n"
    "\n"
    "# The id provides a way to associate feedback and\n"
    "# result message with specific goal requests. The id\n"
    "# specified must be unique.\n"
    "string id\n"
    "\n"
    "\n"
    "================================================================================\n"
    "MSG: cart_interp/CartesianArmServerGoal\n"
    "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
    "# goal definition\n"
    "geometry_msgs/PoseStamped setpoint\n"
    "\n"
    "================================================================================\n"
    "MSG: geometry_msgs/PoseStamped\n"
    "# A Pose with reference coordinate frame and timestamp\n"
    "Header header\n"
    "Pose pose\n"
    "\n"
    "================================================================================\n"
    "MSG: geometry_msgs/Pose\n"
    "# A representation of pose in free space, composed of postion and orientation. \n"
    "Point position\n"
    "Quaternion orientation\n"
    "\n"
    "================================================================================\n"
    "MSG: geometry_msgs/Point\n"
    "# This contains the position of a point in free space\n"
    "float64 x\n"
    "float64 y\n"
    "float64 z\n"
    "\n"
    "================================================================================\n"
    "MSG: geometry_msgs/Quaternion\n"
    "# This represents an orientation in free space in quaternion form.\n"
    "\n"
    "float64 x\n"
    "float64 y\n"
    "float64 z\n"
    "float64 w\n"
    "\n"
    "\n"
    );
  }
  inline virtual const std::string __getDataType() const { return __s_getDataType(); }
  inline virtual const std::string __getMD5Sum() const { return __s_getMD5Sum(); }
  inline virtual const std::string __getMessageDefinition() const { return __s_getMessageDefinition(); }
  inline uint32_t serializationLength() const
  {
    unsigned __l = 0;
    __l += header.serializationLength(); // header
    __l += goal_id.serializationLength(); // goal_id
    __l += goal.serializationLength(); // goal
    return __l;
  }
  virtual uint8_t *serialize(uint8_t *write_ptr,
                             uint32_t seq) const
  {
    roslib::Header _ser_header = header;
    bool __reset_seq = (header.seq == 0);
    if (__reset_seq) _ser_header.seq = seq;
    write_ptr = _ser_header.serialize(write_ptr, seq);
    write_ptr = goal_id.serialize(write_ptr, seq);
    write_ptr = goal.serialize(write_ptr, seq);
    return write_ptr;
  }
  virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    read_ptr = header.deserialize(read_ptr);
    read_ptr = goal_id.deserialize(read_ptr);
    read_ptr = goal.deserialize(read_ptr);
    return read_ptr;
  }
};

typedef boost::shared_ptr<CartesianArmServerActionGoal> CartesianArmServerActionGoalPtr;
typedef boost::shared_ptr<CartesianArmServerActionGoal const> CartesianArmServerActionGoalConstPtr;


}

#endif