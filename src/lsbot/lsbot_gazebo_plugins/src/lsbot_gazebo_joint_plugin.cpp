#include <lsbot_gazebo_plugins/lsbot_gazebo_joint_plugin.hpp>

namespace gazebo_plugins
{

LsbotGazeboPluginRos::LsbotGazeboPluginRos()
: impl_(std::make_unique<LsbotGazeboPluginRosPrivate>())
{
}

LsbotGazeboPluginRos::~LsbotGazeboPluginRos()
{
}

void LsbotGazeboPluginRosPrivate::execute_trajectory_axis1(float goal_angle)
{
  if ( goal_angle == floorscan_angle_msg.angle)
  {
      return;
  }

  printf("Executing trajectory...\n");

  struct timespec initial_time;
  clock_gettime(CLOCK_REALTIME, &initial_time);
  double initial_time_secs = (double)(initial_time.tv_sec) + (double)(initial_time.tv_nsec/1e+9);
  struct timespec current_time;
  double diff_time_secs = 0;

  while(executing_axis1)
  {
    clock_gettime(CLOCK_REALTIME, &current_time);
    double current_time_secs = (double)(current_time.tv_sec) + (double)(current_time.tv_nsec/1e+9);

    diff_time_secs = current_time_secs - initial_time_secs;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.push_back(joints_[LsbotGazeboPluginRosPrivate::AXIS1]->Position(0));
    point.velocities.push_back(joints_[LsbotGazeboPluginRosPrivate::AXIS1]->GetVelocity(0));
    point.time_from_start.sec = (int)diff_time_secs;
    point.time_from_start.nanosec = (diff_time_secs - (int)diff_time_secs)*1e+9;

    usleep(10000);
  }

  RCLCPP_INFO(rclcpp::get_logger("server"), "Angle Goal Suceeded");
}

void LsbotGazeboPluginRosPrivate::timer_motor_state_msgs()
{
  gazebo::common::Time cur_time = model_->GetWorld()->SimTime();

  lsbot_msgs::msg::StateRotaryServo motor_state_msg;
  motor_state_msg.header.stamp.sec = cur_time.sec;
  motor_state_msg.header.stamp.nanosec = cur_time.nsec;
  motor_state_msg.position = joints_[LsbotGazeboPluginRosPrivate::AXIS1]->Position(0);
  motor_state_msg.velocity = joints_[LsbotGazeboPluginRosPrivate::AXIS1]->GetVelocity(0);
  motor_state_msg.effort = joints_[LsbotGazeboPluginRosPrivate::AXIS1]->GetForce(0);
  motor_state_msg.goal = goal_position_axis1_rad;
  motor_state_msg.error = (goal_position_axis1_rad - motor_state_msg.position);
  motor_state_msg.load = 0;
  motor_state_msg.moving = executing_axis1;
  motor_state_msg.fault = lsbot_msgs::msg::StateRotaryServo::FAULT_NONE;
  motor_state_axis1_pub->publish(motor_state_msg);
}

void LsbotGazeboPluginRos::createGenericTopics(std::string node_name)
{
  // Creating sim topic name
  std::string service_name_specs = std::string(node_name) + "/specs";
  std::string topic_name_state_angle = "/floorscan/angle";
  auto rmw_qos_profile_default = rclcpp::SensorDataQoS().reliable();

  std::function<void( std::shared_ptr<rmw_request_id_t>,
                      const std::shared_ptr<lsbot_msgs::srv::SpecsRotaryServo::Request>,
                      std::shared_ptr<lsbot_msgs::srv::SpecsRotaryServo::Response>)> cb_SpecsRotaryServo_function = std::bind(
        &LsbotGazeboPluginRosPrivate::SpecsRotaryServoService, impl_.get(), std::placeholders::_1,  std::placeholders::_2,  std::placeholders::_3);
  impl_->specs_srv_ = impl_->ros_node_->create_service<lsbot_msgs::srv::SpecsRotaryServo>(service_name_specs, cb_SpecsRotaryServo_function);
  RCUTILS_LOG_INFO_NAMED(impl_->ros_node_->get_name(), "creating service called: %s ", service_name_specs.c_str());

  impl_->angle_pub = impl_->ros_node_->create_publisher<lsbot_msgs::msg::Angle>(topic_name_state_angle,
                rmw_qos_profile_default);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "creating %s publisher topic", topic_name_state_angle.c_str());

  impl_->timer_status_ = impl_->ros_node_->create_wall_timer(
      1s, std::bind(&LsbotGazeboPluginRosPrivate::timer_status_msgs, impl_.get()));
}

void LsbotGazeboPluginRosPrivate::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  checkFloorScanWithVelocity(msg->twist.twist.linear.x);
}

void LsbotGazeboPluginRosPrivate::commandCallback_axis1(const lsbot_msgs::msg::GoalRotaryServo::SharedPtr msg)
{
  if (!executing_axis1)
  {
    if (msg->velocity!=0.0)
    {
      trajectories_position_axis1.clear();
      trajectories_velocities_axis1.clear();
      std::vector<double> X(2), Y_vel(2), Y_pos(2);

      float current_pose_rad = joints_[LsbotGazeboPluginRosPrivate::AXIS1]->Position(0);

      double start_time = 0;
      double end_time = fabs(current_pose_rad-msg->position)/msg->velocity;

      Y_vel[0] = 0;
      Y_pos[0] = current_pose_rad;
      X[0] = start_time;

      Y_vel[1] = 0;
      Y_pos[1] = msg->position;
      X[1] = end_time;
      /*tk::spline interpolation_vel, interpolation_pos;
      if (!interpolation_vel.set_points(X, Y_vel))
        return;
      if (!interpolation_pos.set_points(X, Y_pos))
        return;

      for (double t = start_time; t < end_time; t+= 0.001 )
      {
        trajectories_position_axis1.push_back(interpolation_pos(t));
        trajectories_velocities_axis1.push_back(interpolation_vel(t));
      }*/
    }
  }
}

void LsbotGazeboPluginRos::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  impl_->trajectories_position_axis1.clear();
  impl_->trajectories_velocities_axis1.clear();
  impl_->executing_axis1 = false;
  impl_->index_trajectory_axis1 = 0;
  impl_->goal_position_axis1_rad = 0;

  std::string node_name = _sdf->Get<std::string>("name");
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "name %s\n", node_name.c_str());

  // Get joints
  impl_->joints_.resize(1);

  auto motor1 = _sdf->Get<std::string>("axis1", "axis1").first;
  impl_->joints_[LsbotGazeboPluginRosPrivate::AXIS1] = _model->GetJoint(motor1);

  impl_->type_motor = _sdf->Get<std::string>("type", "None").first;
  gzmsg << "type_motor " << impl_->type_motor << std::endl;

  if (!impl_->joints_[LsbotGazeboPluginRosPrivate::AXIS1])
  {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(),
      "Joint [%s] not found, plugin will not work.", motor1.c_str());
    impl_->ros_node_.reset();
    return;
  }

  createGenericTopics(node_name);
  auto rmw_qos_profile_sensor_data = rclcpp::SensorDataQoS().reliable();

  // Creating motor state topic name
  std::string topic_name_motor_state = std::string(node_name) + "/state_axis1";
  impl_->motor_state_axis1_pub = impl_->ros_node_->create_publisher<lsbot_msgs::msg::StateRotaryServo>(topic_name_motor_state,
                        rmw_qos_profile_sensor_data);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Creating topic %s", topic_name_motor_state.c_str() );

  // Creating command topic name
  std::string topic_odom = "/odom_shaft";
  impl_->odom_sub_ = impl_->ros_node_->create_subscription<nav_msgs::msg::Odometry>(topic_odom,
                                rmw_qos_profile_sensor_data,
                                std::bind(&LsbotGazeboPluginRosPrivate::odomCallback, impl_.get(), std::placeholders::_1)
                                );
  std::string topic_command_state = std::string(node_name) + "/goal_axis1";
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Creating topic %s", topic_odom.c_str() );
  impl_->command_sub_axis1_ = impl_->ros_node_->create_subscription<lsbot_msgs::msg::GoalRotaryServo>(topic_command_state,
                                rmw_qos_profile_sensor_data,
                                std::bind(&LsbotGazeboPluginRosPrivate::commandCallback_axis1, impl_.get(), std::placeholders::_1)
                                );
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Creating topic %s", topic_command_state.c_str() );

  // Update rate
  auto update_rate = _sdf->Get<double>("update_rate", 1000.0).first;
  if (update_rate > 0.0)
  {
    impl_->update_period_ = 1.0 / update_rate;
  }
  else
  {
    impl_->update_period_ = 0.0;
  }
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Update rate: %.4f\n", impl_->update_period_);

  impl_->last_update_time_ = _model->GetWorld()->SimTime();

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&LsbotGazeboPluginRosPrivate::OnUpdate, impl_.get(), std::placeholders::_1));

  impl_->timer_motor_state_ = impl_->ros_node_->create_wall_timer(
        50ms, std::bind(&LsbotGazeboPluginRosPrivate::timer_motor_state_msgs, impl_.get()));
}

void LsbotGazeboPluginRos::Reset()
{
  impl_->trajectories_position_axis1.clear();
  impl_->trajectories_velocities_axis1.clear();
  impl_->executing_axis1 = false;
  impl_->index_trajectory_axis1 = 0;
  impl_->goal_position_axis1_rad = 0;
}

void LsbotGazeboPluginRosPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{

  if (!executing_axis1 && trajectories_position_axis1.size()>0)
  {
    index_trajectory_axis1 = 0;
    executing_axis1 = true;
  }

  if (executing_axis1)
  {
    goal_position_axis1_rad = trajectories_position_axis1[index_trajectory_axis1];
    index_trajectory_axis1++;
    if (index_trajectory_axis1==trajectories_position_axis1.size())
    {
      executing_axis1 = false;
      trajectories_position_axis1.clear();
      index_trajectory_axis1 = 0;
    }
  }

  joints_[LsbotGazeboPluginRosPrivate::AXIS1]->SetPosition(0, goal_position_axis1_rad, false);

  last_update_time_ = _info.simTime;
}

void LsbotGazeboPluginRosPrivate::timer_status_msgs()
{
  lsbot_msgs::msg::Angle angle_msg;
  gazebo::common::Time cur_time = model_->GetWorld()->SimTime();
  angle_msg.header.stamp.sec = cur_time.sec;
  angle_msg.header.stamp.nanosec = cur_time.nsec;
  angle_msg.status = angle_msg.STABLE;
  angle_msg.angle = floorscan_angle_msg.angle;
  angle_pub->publish(angle_msg);
}

void LsbotGazeboPluginRosPrivate::SpecsRotaryServoService(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<lsbot_msgs::srv::SpecsRotaryServo::Request>,
    std::shared_ptr<lsbot_msgs::srv::SpecsRotaryServo::Response> res)
{
  res->control_type = (uint8_t)lsbot_msgs::srv::SpecsRotaryServo::Response::CONTROL_TYPE_POSITION_VELOCITY;
  res->range_min = joints_[LsbotGazeboPluginRosPrivate::AXIS1]->LowerLimit();
  res->range_max = joints_[LsbotGazeboPluginRosPrivate::AXIS1]->UpperLimit();
  res->precision = 0.00008722222; // 0.005º

  res->rated_speed = 1.46607657; // 14 RPM
  res->reachable_speed = 1.46607657; // 14 RPM
  res->rated_torque = 9; // 9-Nm
  res->reachable_torque = 13; // 13-Nm

  res->temperature_range_min  = -10.0; // -10º
  res->temperature_range_max  = +50.0; // 50º
}

bool LsbotGazeboPluginRosPrivate::setFloorScanAngle(float ang)
{
  /*ros::ServiceClient client = n->serviceClient<diffrobot2_msgs::SetAngle>(floorscan_angle_service_name_);
  diffrobot2_msgs::SetAngle srv;
  srv.request.angle = ang;
  bool ret = client.call(srv);
  floorscan_angle_error_success_ = srv.response.success;
  floorscan_angle_error_message_ = srv.response.message;
  */
  execute_trajectory_axis1(ang);
  bool ret = true;
  if (!ret)
  {
    RCLCPP_INFO( ros_node_->get_logger(), "Failed to call %s service: %s", floorscan_angle_service_name_.c_str(), floorscan_angle_error_message_.c_str());
  }
  return ret;
}

void LsbotGazeboPluginRosPrivate::processFloorScanAngle(const lsbot_msgs::msg::Angle::SharedPtr msg)
{
  floorscan_angle_msg = *msg;
  if (msg->status != floorscan_angle_msg.STABLE)
    return;

  floorscan1_tilt_angle_ = floorscan_angle_msg.angle;
  floorscan1_corrected_height_ = floorscan1_height_ + floorscan1_head_height_ * cos( floorscan1_tilt_angle_ );
  checkFloorScanWithVelocity(current_velocity);
}

void LsbotGazeboPluginRosPrivate::checkFloorScanWithVelocity(float velocity)
{
  int desired_angle = 90;
  if (velocity < floorscan_velocity_low_range_)
  {
    desired_angle = floorscan_angle_low_;
  }
  else if  (velocity >= floorscan_velocity_low_range_ && velocity < floorscan_velocity_middle_range_)
  {
    desired_angle = floorscan_angle_middle_;
  }
  else if  (velocity >= floorscan_velocity_middle_range_)
  {
    desired_angle = floorscan_angle_high_;
  }
  if (desired_angle != floorscan_last_desired_angle)
  {
    setFloorScanAngle(desired_angle * M_PI / 180);
    floorscan_last_desired_angle = desired_angle;
    floorscan_angle_msg.status = floorscan_angle_msg.UNSTABLE;
  }
}

GZ_REGISTER_MODEL_PLUGIN(LsbotGazeboPluginRos)
}  // namespace gazebo_plugins
