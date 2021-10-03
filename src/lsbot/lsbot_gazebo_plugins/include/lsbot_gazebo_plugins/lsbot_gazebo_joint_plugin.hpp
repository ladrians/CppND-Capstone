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

    /// Update period in seconds.
    double update_period_;

    /// Last update time.
    gazebo::common::Time last_update_time_;

    /// Last time the encoder was updated
    gazebo::common::Time last_encoder_update_;

    /// Robot base frame ID
    std::string robot_base_frame_;

    float floorscan_velocity_low_range_ = 0.4;
    float floorscan_velocity_middle_range_ = 0.7;
    int floorscan_angle_counter_ = 0;
    int floorscan_angle_low_ = 60; // degrees
    int floorscan_angle_middle_ = 45; // degrees
    int floorscan_angle_high_ = 30; // degrees
    int floorscan_last_desired_angle = 0;

    lsbot_msgs::msg::Angle floorscan_angle_msg;

    /*
    void SpecsCommunicationService(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<hrim_generic_srvs::srv::SpecsCommunication::Request> req,
        std::shared_ptr<hrim_generic_srvs::srv::SpecsCommunication::Response> res);

    void SpecsRotaryServoService(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<hrim_actuator_rotaryservo_srvs::srv::SpecsRotaryServo::Request> req,
        std::shared_ptr<hrim_actuator_rotaryservo_srvs::srv::SpecsRotaryServo::Response> res);

    void IDService(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<hrim_generic_srvs::srv::ID::Request> req,
        std::shared_ptr<hrim_generic_srvs::srv::ID::Response> res);

    void Sim3DService(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<hrim_generic_srvs::srv::Simulation3D::Request> req,
        std::shared_ptr<hrim_generic_srvs::srv::Simulation3D::Response> res);

    void URDFService(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<hrim_generic_srvs::srv::SimulationURDF::Request> req,
        std::shared_ptr<hrim_generic_srvs::srv::SimulationURDF::Response> res);

    std::shared_ptr<rclcpp::Publisher<hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo>> motor_state_axis1_pub;
    std::shared_ptr<rclcpp::Publisher<hrim_actuator_rotaryservo_srvs::srv::SpecsRotaryServo>> specs_pub;

    std::shared_ptr<rclcpp::Subscription<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>> command_sub_axis1_;

    void commandCallback_axis1(const hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo::SharedPtr msg);
    */
    std::shared_ptr<rclcpp::Publisher<lsbot_msgs::msg::Angle>> angle_pub;
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> odom_sub_;

    std::shared_ptr<rclcpp::TimerBase> timer_status_;
    std::shared_ptr<rclcpp::TimerBase> timer_comm_;

    /*
    std::shared_ptr<rclcpp::Publisher<hrim_generic_msgs::msg::Status>> status_pub;
    std::shared_ptr<rclcpp::Publisher<hrim_generic_msgs::msg::Power>> power_pub;
    std::shared_ptr<rclcpp::Publisher<hrim_generic_msgs::msg::StateCommunication>> state_comm_pub;

    rclcpp::Service<hrim_generic_srvs::srv::ID>::SharedPtr id_srv_;
    rclcpp::Service<hrim_generic_srvs::srv::Simulation3D>::SharedPtr sim_3d_srv_;
    rclcpp::Service<hrim_generic_srvs::srv::SimulationURDF>::SharedPtr sim_urdf_srv_;
    rclcpp::Service<hrim_generic_srvs::srv::SpecsCommunication>::SharedPtr specs_comm_srv_;
    rclcpp::Service<hrim_actuator_rotaryservo_srvs::srv::SpecsRotaryServo>::SharedPtr specs_srv_;
    */

    void timer_status_msgs();
    void timer_comm_msgs();

    /*
    rclcpp_action::Server<hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory>::SharedPtr action_server_trajectory_axis1_;

    std::shared_ptr<rclcpp_action::ServerGoalHandle<hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory>> goal_handle_axis1_;
    */

    /*
    void handle_trajectory_axis1_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory>> goal_handle);
    void execute_trajectory_axis1(const std::shared_ptr<rclcpp_action::ServerGoalHandle<hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory>> goal_handle);
    rclcpp_action::GoalResponse handle_trajectory_axis1_goal(
            const std::array<uint8_t, 16> & uuid,
            std::shared_ptr<const hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory::Goal> goal);
    rclcpp_action::CancelResponse handle_trajectory_axis1_cancel(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory>> goal_handle);
    */
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

    std::mutex odom_mutex_;

  private:
    /// Private data pointer
    std::unique_ptr<LsbotGazeboPluginRosPrivate> impl_;
  };
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS_LSBOT_GAZEBO_PLUGINSE_HPP_