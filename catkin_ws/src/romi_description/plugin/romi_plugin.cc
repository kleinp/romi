#ifndef _ROMI_PLUGIN_HH_
#define _ROMI_PLUGIN_HH_

// Gazebo includes
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// ROS includes
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32MultiArray.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"

namespace gazebo
{
/// \brief A plugin to control a Velodyne sensor.
class RomiPlugin : public ModelPlugin
{
  /// \brief Constructor
public:
  RomiPlugin() {}

  /**
     * The load function is called by Gazebo when the plugin is
     * inserted into simulation
     */
public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Store important components
    this->model = _model;
    this->model_name = _model->GetName();

    // Output a welcome message
    std::cout << "The romi plugin is attach to model[" << _model << "] (" << this->model_name << ")\n";

    // Set up left wheel PID controller
    this->left_wheel = _model->GetJoints()[0];
    this->left_pid = common::PID(0.1, 0, 0);
    this->model->GetJointController()->SetVelocityPID(this->left_wheel->GetScopedName(), this->left_pid);
    //std::cout << "Left wheel joint: " << this->left_wheel->GetScopedName() << "\n";

    // Set up right wheel PID controller
    this->right_wheel = _model->GetJoints()[1];
    this->right_pid = common::PID(0.1, 0, 0);
    this->model->GetJointController()->SetVelocityPID(this->right_wheel->GetScopedName(), this->right_pid);
    //std::cout << "Right wheel joint: " << this->right_wheel->GetScopedName() << "\n";

    // --- ROS integration ---

    // Initialize ROS
    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, this->model->GetScopedName(), ros::init_options::NoSigintHandler);
    }

    // create ROS node
    this->rosNode.reset(new ros::NodeHandle(this->model_name));

    // --- Static transforms for distance sensors --------------------------------------------------------------
    frame_dist_fwd = staticTransform("frame_base", "frame_dist_fwd", 0.06, 0, 0.033, 0);
    frame_dist_n45 = staticTransform("frame_base", "frame_dist_n45", 0.06, 0, 0.033, 0.7854);
    frame_dist_p45 = staticTransform("frame_base", "frame_dist_p45", 0.06, 0, 0.033, -0.7854);

    // --- Velocity commands (input to Gazebo) -----------------------------------------------------------------

    // create named topic and subscribe to it
    std::string cmd_vel_topic = "/" + this->model_name + "/cmd_vel";
    std::cout << "Velocity topic: " << cmd_vel_topic << "\n";
    std::cout << "This topic accepts a geometry_msgs/Twist message type\n";

    ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
        cmd_vel_topic, 1, boost::bind(&RomiPlugin::OnRosVelMsg, this, _1), ros::VoidPtr(), &this->rosQueue);

    this->vel_sub = this->rosNode->subscribe(so);

    // --- Odometry (output from Gazebo) -----------------------------------------------------------------------

    // set up odometry publisher
    this->odom_pub = this->rosNode->advertise<nav_msgs::Odometry>("odom", 50);

    // --- Sensors (output from Gazebo) ------------------------------------------------------------------------

    // --- Finalize --------------------------------------------------------------------------------------------

    // Start queue helper thread for topic subscriber
    this->rosQueueThread = std::thread(std::bind(&RomiPlugin::QueueThread, this));

    // Start
    this->gazeboUpdate = event::Events::ConnectWorldUpdateBegin(std::bind(&RomiPlugin::UpdateTask, this));
  }

  // Helper functions
public:
  void SetVelocity(const double &_left, const double &_right)
  {
    //std::cout << "Setting velocity (m/s): left=" << _left << " right=" << _right << "\n";
    this->model->GetJointController()->SetVelocityTarget(this->left_wheel->GetScopedName(), _left);
    // right wheel is backwards so negate command
    this->model->GetJointController()->SetVelocityTarget(this->right_wheel->GetScopedName(), -_right);
  }

  void OnRosVelMsg(const geometry_msgs::Twist::ConstPtr &msg)
  {
    // Convert the twist message into left and right wheel commands, which are expecting (rad/s)
    // The robot can't move linearly in Y or Z, and can't rotate in X or Y. These are going to be ignored
    // Distance between wheels is 141mm and wheel diameter is 70mm

    double vx = msg->linear.x;
    double wz = msg->angular.z;

    double left = (vx + wz * 0.141 / 2) / .07;
    double right = (vx - wz * 0.141 / 2) / .07;

    this->SetVelocity(left, right);
  }

private:
  void UpdateTask()
  {
    // have a counter to run things every once in a while
    static int i = 0;
    i++;

    ros::Time current_time = ros::Time::now();

    // get model pose and velocities
    ignition::math::Pose3d pose = this->model->RelativePose();
    ignition::math::Vector3d vel = this->model->RelativeLinearVel();
    ignition::math::Vector3d rot = this->model->RelativeAngularVel();

    // Build and send out transform
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "frame_base";

    odom_trans.transform.translation.x = pose.Pos().X();
    odom_trans.transform.translation.y = pose.Pos().Y();
    odom_trans.transform.translation.z = pose.Pos().Z();
    odom_trans.transform.rotation = quaternionIgnition2geometryMsg(pose.Rot());

    tf_broadcaster.sendTransform(odom_trans);

    // Build and send out odometry
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "map";
    odom.child_frame_id = "frame_base";

    odom.pose.pose.position.x = odom_trans.transform.translation.x;
    odom.pose.pose.position.y = odom_trans.transform.translation.y;
    odom.pose.pose.position.z = odom_trans.transform.translation.z;
    odom.pose.pose.orientation = odom_trans.transform.rotation;

    odom.twist.twist.linear.x = vel.X();
    odom.twist.twist.linear.y = vel.Y();
    odom.twist.twist.angular.z = rot.Z();

    odom_pub.publish(odom);

    //std::cout << "Pose (x, y, z): (" << pose.Pos().X() << ", " << pose.Pos().Y() << ", " << pose.Pos().Z() << ")\n";

    // Every 10 loops, publish static transforms for lasers relative to frame_base
    if (i % 10 == 0)
    {
      frame_dist_fwd.header.stamp = current_time;
      frame_dist_n45.header.stamp = current_time;
      frame_dist_p45.header.stamp = current_time;
      tf_broadcaster.sendTransform(frame_dist_fwd);
      tf_broadcaster.sendTransform(frame_dist_n45);
      tf_broadcaster.sendTransform(frame_dist_p45);
    }
  }

  void QueueThread()
  {
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

private:
  // Gazebo private variables
  physics::ModelPtr model;
  std::string model_name;
  physics::JointPtr left_wheel;
  common::PID left_pid;
  physics::JointPtr right_wheel;
  common::PID right_pid;

  // ROS private variables
  std::unique_ptr<ros::NodeHandle> rosNode;
  ros::Subscriber vel_sub;
  ros::CallbackQueue rosQueue;
  std::thread rosQueueThread;
  event::ConnectionPtr gazeboUpdate;
  ros::Publisher odom_pub;
  tf::TransformBroadcaster tf_broadcaster;

  // static transform frames
  geometry_msgs::TransformStamped frame_dist_fwd;
  geometry_msgs::TransformStamped frame_dist_n45;
  geometry_msgs::TransformStamped frame_dist_p45;

  geometry_msgs::Quaternion quaternionIgnition2geometryMsg(ignition::math::Quaterniond q)
  {
    geometry_msgs::Quaternion gq;
    gq.x = q.X();
    gq.y = q.Y();
    gq.z = q.Z();
    gq.w = q.W();

    return (gq);
  }

  geometry_msgs::TransformStamped staticTransform(std::string base_frame,
                                                  std::string child_frame, double x, double y, double z, double w)
  {
    geometry_msgs::TransformStamped tmpT;
    ignition::math::Quaterniond tmpQ(0, 0, w);

    tmpT.header.frame_id = base_frame;
    tmpT.child_frame_id = child_frame;
    tmpT.transform.translation.x = x;
    tmpT.transform.translation.y = y;
    tmpT.transform.translation.z = z;
    tmpT.transform.rotation.x = tmpQ.X();
    tmpT.transform.rotation.y = tmpQ.Y();
    tmpT.transform.rotation.z = tmpQ.Z();
    tmpT.transform.rotation.w = tmpQ.W();

    return tmpT;
  }
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(RomiPlugin)
}
#endif