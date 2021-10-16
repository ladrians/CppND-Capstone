#include <lsbot_gazebo_plugins/lsbot_gazebo_joint_plugin.hpp>
#include <memory>

#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>

namespace gazebo_plugins
{

LsbotGazeboPluginRos::LsbotGazeboPluginRos()
{
  impl_.reset(new gazebo_plugins::LsbotGazeboPluginRosPrivate());
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

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(joints_[LsbotGazeboPluginRosPrivate::AXIS1]->Position(0));
    point.velocities.push_back(joints_[LsbotGazeboPluginRosPrivate::AXIS1]->GetVelocity(0));
    point.time_from_start.sec = (int)diff_time_secs;
    point.time_from_start.nsec = (diff_time_secs - (int)diff_time_secs)*1e+9;

    usleep(10000);
  }
  floorscan_angle_msg.status = floorscan_angle_msg.STABLE;
  if (debug)
  {
    ROS_INFO("Angle Goal Suceeded");
  }
}

void LsbotGazeboPluginRos::createGenericTopics(std::string node_name)
{
  // Creating sim topic name
  std::string service_name_specs = std::string(node_name) + "/specs";
  std::string topic_name_state_angle = impl_->topic_name_state_angle_;

  /*std::function<void( std::shared_ptr<int>,
                      const std::shared_ptr<lsbot_msgs::SpecsRotaryServo::Request>,
                      std::shared_ptr<lsbot_msgs::SpecsRotaryServo::Response>)> cb_SpecsRotaryServo_function = std::bind(
        &LsbotGazeboPluginRosPrivate::SpecsRotaryServoService, impl_.get(), std::placeholders::_1,  std::placeholders::_2,  std::placeholders::_3);
  impl_->specs_srv_ = impl_->ros_node_->advertiseService<lsbot_msgs::SpecsRotaryServo>(service_name_specs, cb_SpecsRotaryServo_function);*/

  impl_->angle_pub = impl_->ros_node_->advertise<lsbot_msgs::Angle>(topic_name_state_angle, 1);

  impl_->timer_status_ = impl_->ros_node_->createTimer(
      ros::Duration(1), std::bind(&LsbotGazeboPluginRosPrivate::timer_status_msgs, impl_.get()));
}

void LsbotGazeboPluginRosPrivate::odomCallback(const nav_msgs::Odometry::ConstPtr&  msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  current_velocity_ = msg->twist.twist.linear.x;
}

void LsbotGazeboPluginRosPrivate::cmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(vel_mutex_);
  desired_velocity_ = msg->linear.x;
}

void LsbotGazeboPluginRos::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;

  // Initialize ROS node
  impl_->ros_node_.reset(new ros::NodeHandle("~"));

  impl_->trajectories_position_axis1.clear();
  impl_->trajectories_velocities_axis1.clear();
  impl_->executing_axis1 = false;
  impl_->index_trajectory_axis1 = 0;
  impl_->goal_position_axis1_rad = 0;

  std::string node_name = _sdf->Get<std::string>("name");
  ROS_INFO("name %s", node_name.c_str());

  // Get joints
  impl_->joints_.resize(1);

  auto motor1 = _sdf->Get<std::string>("axis1", "axis1").first;
  impl_->joints_[LsbotGazeboPluginRosPrivate::AXIS1] = _model->GetJoint(motor1);

  impl_->type_motor = _sdf->Get<std::string>("type", "None").first;
  gzmsg << "type_motor " << impl_->type_motor << std::endl;

  if (!impl_->joints_[LsbotGazeboPluginRosPrivate::AXIS1])
  {
    ROS_INFO(
      "Joint [%s] not found, plugin will not work.", motor1.c_str());
    impl_->ros_node_.reset();
    return;
  }

  impl_->ros_node_->param<std::string>("topic_name_state_angle", impl_->topic_name_state_angle_, impl_->topic_name_state_angle_);
  impl_->ros_node_->param<std::string>("topic_odom", impl_->topic_odom_, impl_->topic_odom_);
  impl_->ros_node_->param<std::string>("topic_cmdvel", impl_->topic_cmdvel_, impl_->topic_cmdvel_);

  createGenericTopics(node_name);

  // Creating motor state topic name
  std::string topic_name_motor_state = std::string(node_name) + "/state_axis1";
  impl_->motor_state_axis1_pub = impl_->ros_node_->advertise<lsbot_msgs::StateRotaryServo>(topic_name_motor_state, 1);

  // Creating subscriptions
  impl_->odom_sub_ = impl_->ros_node_->subscribe<nav_msgs::Odometry>(impl_->topic_odom_, 1, &LsbotGazeboPluginRosPrivate::odomCallback, impl_.get());
  impl_->cmdvel_sub_ = impl_->ros_node_->subscribe<geometry_msgs::Twist>(impl_->topic_cmdvel_, 1, &LsbotGazeboPluginRosPrivate::cmdvelCallback, impl_.get());

  // Update rate
  auto update_rate = _sdf->Get<double>("update_rate", 3.0).first;
  if (update_rate > 0.0)
  {
    impl_->update_period_ = 1.0 / update_rate;
  }
  else
  {
    impl_->update_period_ = 0.0;
  }
  ROS_INFO("\tupdate_period: %.4f", impl_->update_period_);
  ROS_INFO("\tlog_throttle: %f", impl_->log_throttle_);
  ROS_INFO("\tfloorscan1_dynamic_default_angle_: %.4f", impl_->floorscan1_dynamic_default_angle_);
  ROS_INFO("\tfloorscan1_dynamic_frame: %d", impl_->floorscan1_dynamic_frame_);
  ROS_INFO("\tfloorscan_velocity_low_range: %.4f", impl_->floorscan_velocity_low_range_);
  ROS_INFO("\tfloorscan_velocity_middle_range: %.4f", impl_->floorscan_velocity_middle_range_);
  ROS_INFO("\trate: %d", impl_->rate_);
  ROS_INFO("\tfloorscan_angle_low: %d", impl_->floorscan_angle_low_);
  ROS_INFO("\tfloorscan_angle_middle: %d", impl_->floorscan_angle_middle_);
  ROS_INFO("\tfloorscan_angle_high: %d", impl_->floorscan_angle_high_);
  ROS_INFO("\tdebug: %d", impl_->debug);
  ROS_INFO("\tfloorscan1_frame_base: %s", impl_->floorscan1_frame_base_.c_str());
  ROS_INFO("\tfloorscan1_frame_child_: %s", impl_->floorscan1_frame_child_.c_str());

  impl_->last_update_time_ = _model->GetWorld()->SimTime();

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&LsbotGazeboPluginRosPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
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
  if (!executing_axis1 && trajectories_position_axis1.size() > 0)
  {
    index_trajectory_axis1 = 0;
    executing_axis1 = true;
  }

  float rads = 0;
  if (executing_axis1)
  {
    goal_position_axis1_rad = trajectories_position_axis1[index_trajectory_axis1];
    rads = to_radians(goal_position_axis1_rad);
    index_trajectory_axis1++;
    if (index_trajectory_axis1 == trajectories_position_axis1.size())
    {
      executing_axis1 = false;
      trajectories_position_axis1.clear();
      index_trajectory_axis1 = 0;
    }
  }

  rads = to_radians(goal_position_axis1_rad);
  bool ret = joints_[LsbotGazeboPluginRosPrivate::AXIS1]->SetPosition(0, rads, false);

  checkFloorScanWithVelocity(desired_velocity_);
  last_update_time_ = _info.simTime;
  if (ret)
  {
    floorscan_angle_msg.status = floorscan_angle_msg.STABLE;
    floorscan_angle_msg.angle = rads;
  }
}

float LsbotGazeboPluginRosPrivate::to_radians(float deg)
{
  return deg * M_PI / 180;
}

void LsbotGazeboPluginRosPrivate::timer_status_msgs()
{
  lsbot_msgs::Angle angle_msg;
  gazebo::common::Time cur_time = model_->GetWorld()->SimTime();
  angle_msg.header.stamp.sec = cur_time.sec;
  angle_msg.header.stamp.nsec = cur_time.nsec;
  angle_msg.status = floorscan_angle_msg.status;
  angle_msg.angle = floorscan_angle_msg.angle;
  angle_pub.publish(angle_msg);
}

void LsbotGazeboPluginRosPrivate::publishSlantedLidarFrame(float angle)
{
  ros::Time now = ros::Time::now();
  float lidar_orientation = floorscan1_dynamic_default_angle_;
  if (floorscan1_dynamic_frame_)
  {
    lidar_orientation = angle;
  }

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = now;
  transformStamped.header.frame_id = floorscan1_frame_base_;
  transformStamped.child_frame_id = floorscan1_frame_child_;
  transformStamped.transform.translation.x = 0.0;
  transformStamped.transform.translation.y = 0.0;
  transformStamped.transform.translation.z = 0.0;
  tf::Quaternion q;
  q = tf::createQuaternionFromRPY(0, lidar_orientation, 0);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  br.sendTransform(transformStamped);
}

void LsbotGazeboPluginRosPrivate::checkFloorScanWithVelocity(float velocity)
{
  int desired_angle = 90;
  if (velocity < floorscan_velocity_low_range_)
  {
    desired_angle = floorscan_angle_low_;
  }
  else if (velocity >= floorscan_velocity_low_range_ && velocity < floorscan_velocity_middle_range_)
  {
    desired_angle = floorscan_angle_middle_;
  }
  else if (velocity >= floorscan_velocity_middle_range_)
  {
    desired_angle = floorscan_angle_high_;
  }
  else
  {
    ROS_WARN("Velocity %.2f Angle:%d", velocity, desired_angle);
  }
  if (desired_angle != floorscan_last_desired_angle)
  {
    floorscan_last_desired_angle = desired_angle;
    floorscan_angle_msg.status = floorscan_angle_msg.UNSTABLE;
    trajectories_position_axis1.push_back(floorscan_last_desired_angle);
  }
  float desired_angle_rads = desired_angle * M_PI / 180;
  publishSlantedLidarFrame(desired_angle_rads);
}

GZ_REGISTER_MODEL_PLUGIN(LsbotGazeboPluginRos)
}  // namespace gazebo_plugins
