#ifndef GAZEBO_PLUGINS_LSBOT_GAZEBO_PLUGINS_HPP_
#define GAZEBO_PLUGINS_LSBOT_GAZEBO_PLUGINSE_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

// #include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "lsbot_msgs/msg/angle.hpp"
#include "lsbot_msgs/msg/goal_rotary_servo.hpp"
#include "lsbot_msgs/msg/state_rotary_servo.hpp"

#include "lsbot_msgs/srv/specs_rotary_servo.hpp"
#include "lsbot_msgs/srv/set_angle.hpp"

#include <string>
#include <vector>
#include <memory>
#include <chrono>

#include "rclcpp_action/rclcpp_action.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

static const double NSEC_PER_SECOND = 1e+9;

using namespace std::chrono_literals;

namespace gazebo_plugins
{
  class LsbotGazeboPluginRosPrivate
  {
  public:

    /// Indicates which wheel
    enum
    {
      AXIS1 = 0,
    };

    /// Callback to be called at every simulation iteration.
    /// \param[in] _info Updated simulation info.
    void OnUpdate(const gazebo::common::UpdateInfo & _info);

    void timer_motor_state_msgs();
    std::shared_ptr<rclcpp::TimerBase> timer_motor_state_;

    /// A pointer to the GazeboROS node.
    gazebo_ros::Node::SharedPtr ros_node_;

    /// Connection to event called at every world iteration.
    gazebo::event::ConnectionPtr update_connection_;

    /// Pointers to wheel joints.
    std::vector<gazebo::physics::JointPtr> joints_;

    /// Pointer to model.
    gazebo::physics::ModelPtr model_;

    /// Protect variables accessed on callbacks.
    std::mutex lock_;
    std::mutex odom_mutex_;

    /// Update period in seconds.
    double update_period_;

    /// Last update time.
    gazebo::common::Time last_update_time_;

    /// Last time the encoder was updated
    gazebo::common::Time last_encoder_update_;

    /// Robot base frame ID
    std::string robot_base_frame_;
    std::string floorscan_angle_error_message_ = "";
    std::string floorscan_angle_service_name_ = "/set_angle";

    float current_velocity = 0;
    float floorscan1_tilt_angle_ = 0.6435;
    float floorscan1_head_height_ = 0.07;
    float floorscan1_height_ = 0.6;
    float floorscan1_corrected_height_ = floorscan1_height_;
    float floorscan_velocity_low_range_ = 0.4;
    float floorscan_velocity_middle_range_ = 0.7;
    int floorscan_angle_counter_ = 0;
    int floorscan_angle_low_ = 60; // degrees
    int floorscan_angle_middle_ = 45; // degrees
    int floorscan_angle_high_ = 30; // degrees
    int floorscan_last_desired_angle = 0;

    lsbot_msgs::msg::Angle floorscan_angle_msg;

    void SpecsRotaryServoService(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<lsbot_msgs::srv::SpecsRotaryServo::Request> req,
        std::shared_ptr<lsbot_msgs::srv::SpecsRotaryServo::Response> res);

    std::shared_ptr<rclcpp::Publisher<lsbot_msgs::msg::StateRotaryServo>> motor_state_axis1_pub;
    std::shared_ptr<rclcpp::Publisher<lsbot_msgs::srv::SpecsRotaryServo>> specs_pub;

    std::shared_ptr<rclcpp::Subscription<lsbot_msgs::msg::GoalRotaryServo>> command_sub_axis1_;

    void commandCallback_axis1(const lsbot_msgs::msg::GoalRotaryServo::SharedPtr msg);

    std::shared_ptr<rclcpp::Publisher<lsbot_msgs::msg::Angle>> angle_pub;
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> odom_sub_;

    std::shared_ptr<rclcpp::TimerBase> timer_status_;
    std::shared_ptr<rclcpp::TimerBase> timer_comm_;

    rclcpp::Service<lsbot_msgs::srv::SpecsRotaryServo>::SharedPtr specs_srv_;

    void timer_status_msgs();

    void execute_trajectory_axis1(float goal_angle);

    bool setFloorScanAngle(float ang);
    void processFloorScanAngle(const lsbot_msgs::msg::Angle::SharedPtr msg);
    void checkFloorScanWithVelocity(float velocity);

    std::vector<float> trajectories_position_axis1;
    std::vector<float> trajectories_velocities_axis1;
    bool executing_axis1;
    unsigned int index_trajectory_axis1;
    float goal_position_axis1_rad;

    std::string type_motor;
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
    std::unique_ptr<LsbotGazeboPluginRosPrivate> impl_;
  };
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS_LSBOT_GAZEBO_PLUGINSE_HPP_