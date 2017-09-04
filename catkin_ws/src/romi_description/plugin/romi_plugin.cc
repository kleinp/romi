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

    // Output a welcome message
    std::cout << "The romi plugin is attach to model[" << _model << "] (" << _model->GetName() << ")\n";

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
      ros::init(argc, argv, "romi", ros::init_options::NoSigintHandler);
    }

    // create ROS node
    this->rosNode.reset(new ros::NodeHandle("romi"));

    // --- Velocity commands ----------------------------------------------------------------------------------

    // create named topic and subscribe to it
    std::string raw_vel_topic = "/" + this->model->GetName() + "/raw_vel_cmd";
    std::cout << "Raw velocity topic: " << raw_vel_topic << "\n";

    ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
        raw_vel_topic,
        1,
        boost::bind(&RomiPlugin::OnRosVelMsg, this, _1),
        ros::VoidPtr(), &this->rosQueue);

    this->vel_sub = this->rosNode->subscribe(so);

    // --- Odometry --------------------------------------------------------------------------------------------

    // set up odometry publisher
    this->odom_pub = this->rosNode->advertise<nav_msgs::Odometry>("odom", 50);

    // --- Sensors ---------------------------------------------------------------------------------------------

    // --- Finalize --------------------------------------------------------------------------------------------

    // Start queue helper thread for topic subscriber
    this->rosQueueThread = std::thread(std::bind(&RomiPlugin::QueueThread, this));

    // Start
    this->gazeboUpdate = event::Events::ConnectWorldUpdateBegin(std::bind(&RomiPlugin::UpdateTask, this));
  }

  // Helper functions
public:
  void SetRawVelocity(const double &_left, const double &_right)
  {
    std::cout << "Setting velocity: left=" << _left << " right=" << _right << "\n";
    this->model->GetJointController()->SetVelocityTarget(this->left_wheel->GetScopedName(), _left);
    // right wheel is backwards so negate command
    this->model->GetJointController()->SetVelocityTarget(this->right_wheel->GetScopedName(), -_right);
  }

  void OnRosVelMsg(const std_msgs::Float32MultiArrayConstPtr &_msg)
  {
    this->SetRawVelocity(_msg->data[0], _msg->data[1]);
  }

private:
  void UpdateTask()
  {
    ros::Time current_time = ros::Time::now();
    // get model pose
    ignition::math::Pose3d pose = this->model->RelativePose();
    ignition::math::Vector3d vel = this->model->RelativeLinearVel();
    ignition::math::Vector3d rot = this->model->RelativeAngularVel();

    // Build and send out transform
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = pose.Pos().X();
    odom_trans.transform.translation.y = pose.Pos().Y();
    odom_trans.transform.translation.z = pose.Pos().Z();
    odom_trans.transform.rotation = quaternionIgnition2geometryMsg(pose.Rot());

    odom_broadcaster.sendTransform(odom_trans);

    // Build and send out odometry
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = odom_trans.transform.translation.x;
    odom.pose.pose.position.y = odom_trans.transform.translation.y;
    odom.pose.pose.position.z = odom_trans.transform.translation.z;
    odom.pose.pose.orientation = odom_trans.transform.rotation;

    odom.twist.twist.linear.x = vel.X();
    odom.twist.twist.linear.y = vel.Y();
    odom.twist.twist.angular.z = rot.Z();

    odom_pub.publish(odom);

    //std::cout << "Pose (x, y, z): (" << pose.Pos().X() << ", " << pose.Pos().Y() << ", " << pose.Pos().Z() << ")\n";
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
  tf::TransformBroadcaster odom_broadcaster;

  geometry_msgs::Quaternion quaternionIgnition2geometryMsg(ignition::math::Quaterniond q)
  {
    geometry_msgs::Quaternion gq;
    gq.x = q.X();
    gq.y = q.Y();
    gq.z = q.Z();
    gq.w = q.W();

    return(gq);
  }
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(RomiPlugin)
}
#endif