#ifndef GAZEBO_PLUGINS_LSBOT_GAZEBO_PLUGINS_HPP_
#define GAZEBO_PLUGINS_LSBOT_GAZEBO_PLUGINSE_HPP_

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

// #include <gazebo_ros/conversions/builtin_interfaces.hpp>
//#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <ros/ros.h>
#include <ros/node_handle.h>


// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include "trajectory_msgs/JointTrajectory.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include "lsbot_msgs/Angle.h"
#include "lsbot_msgs/GoalRotaryServo.h"
#include "lsbot_msgs/StateRotaryServo.h"

#include "lsbot_msgs/SpecsRotaryServo.h"
#include "lsbot_msgs/SetAngle.h"

#include <string>
#include <vector>
#include <memory>
#include <chrono>

namespace gazebo_plugins
{
  class LsbotGazeboPluginRosPrivate
  {
  public:

    /// Defines the moving Axis
    enum
    {
      AXIS1 = 0,
    };

    /// Callback to be called at every simulation iteration.
    /// \param[in] _info Updated simulation info.
    void OnUpdate(const gazebo::common::UpdateInfo & _info);

    float to_radians(float deg);

    /// A pointer to the GazeboROS node.
    boost::shared_ptr<ros::NodeHandle> ros_node_;

    /// Connection to event called at every world iteration.
    gazebo::event::ConnectionPtr update_connection_;

    /// Pointers to movable joints.
    std::vector<gazebo::physics::JointPtr> joints_;

    /// Pointer to model.
    gazebo::physics::ModelPtr model_;

    /// Protect variables accessed on callbacks.
    std::mutex vel_mutex_;
    std::mutex odom_mutex_;

    /// Update period in seconds.
    double update_period_ = 5;

    /// Last update time.
    gazebo::common::Time last_update_time_;

    /// Last time the encoder was updated
    gazebo::common::Time last_encoder_update_;

    float log_throttle_ = 3; /* secs */
    float current_velocity_ = 0;
    float desired_velocity_ = 0;
    float floorscan_velocity_low_range_ = 0.4;
    float floorscan_velocity_middle_range_ = 0.7;

    int rate_ = 5;
    int floorscan_angle_low_ = 60; // degrees
    int floorscan_angle_middle_ = 45; // degrees
    int floorscan_angle_high_ = 30; // degrees
    int floorscan_last_desired_angle = 0;

    /// Current Angle configuration
    lsbot_msgs::Angle floorscan_angle_msg;

    ros::Publisher motor_state_axis1_pub;
    ros::Publisher angle_pub;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& message);
    void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg);

    ros::Subscriber odom_sub_;
    ros::Subscriber cmdvel_sub_;

    ros::Timer timer_status_;

    ros::ServiceServer specs_srv_;

    /// Status publishers
    void timer_status_msgs();

    void execute_trajectory_axis1(float goal_angle);

    /// Change Angle based on velocity
    void checkFloorScanWithVelocity(float velocity);
    void publishSlantedLidarFrame(float angle);

    std::vector<float> trajectories_position_axis1;
    std::vector<float> trajectories_velocities_axis1;
    bool executing_axis1 = false;
    bool debug = false;
    bool floorscan1_dynamic_frame_ = true;
    unsigned int index_trajectory_axis1 = 0;
    float goal_position_axis1_rad = 0;
    float floorscan1_dynamic_default_angle_ = 0.68;

    std::string type_motor;
    std::string floorscan1_frame_base_ = "floorscan";
    std::string floorscan1_frame_child_ = "floorscan_dynamic";
    std::string topic_odom_ = "/odom_shaft";
    std::string topic_cmdvel_ = "/cmd_vel";
    std::string topic_name_state_angle_ = "/floorscan/angle";
  };

  /// A plugin for gazebo.
  /*
   *
   * \author  Luciano Silveira (ladrians <at> gmail.com)
   *
   */
  class LsbotGazeboPluginRos : public gazebo::ModelPlugin
  {
  public:
    /// Constructor
    LsbotGazeboPluginRos();

    /// Destructor
    ~LsbotGazeboPluginRos();

    void createGenericTopics(std::string node_name);

  protected:
    // Documentation inherited
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    // Documentation inherited
    void Reset() override;

  private:
    /// Private data pointer
    boost::shared_ptr<LsbotGazeboPluginRosPrivate> impl_;
  };
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS_LSBOT_GAZEBO_PLUGINSE_HPP_