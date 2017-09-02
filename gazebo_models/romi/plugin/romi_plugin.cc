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
     * 
     * param[in] _model A pointer to the model that this plugin is
     * attached to.
     * param[in] _sdf A pointer to the plugin's SDF element.
     * 
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

    // Create node for gazebo API
    this->node = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION < 8
    this->node->Init(this->model->GetWorld()->GetName());
#else
    this->node->Init(this->model->GetWorld()->Name());
#endif

    // Create topic names
    std::string velTopicName = "~/" + this->model->GetName() + "/vel_cmd";
    std::cout << "Velocity topic: " << velTopicName << "\n";

    // Subscribe to velocity topic and register callback
    this->sub = this->node->Subscribe(velTopicName, &RomiPlugin::OnVelMsg, this);

    // --- ROS integration ---

    // Initialize ROS
    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
    }

    // create ROS node
    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    // create named topic and subscribe to it
    std::string rosVelTopicName = "/" + this->model->GetName() + "/vel_cmd";
    std::cout << "ROS Velocity topic: " << rosVelTopicName << "\n";

    ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
        rosVelTopicName,
        1,
        boost::bind(&RomiPlugin::OnRosVelMsg, this, _1),
        ros::VoidPtr(), &this->rosQueue);
    
    this->rosSub = this->rosNode->subscribe(so);

    // Spin up queue helper thread
    this->rosQueueThread = std::thread(std::bind(&RomiPlugin::QueueThread, this));
  }

  // Helper functions
public:
  void SetVelocity(const double &_left, const double &_right)
  {
    std::cout << "Setting velocity: left=" << _left << " right=" << _right << "\n";
    this->model->GetJointController()->SetVelocityTarget(this->left_wheel->GetScopedName(), _left);
    // right wheel is backwards so negate command
    this->model->GetJointController()->SetVelocityTarget(this->right_wheel->GetScopedName(), -_right);
  }

  void OnRosVelMsg(const std_msgs::Float32MultiArrayConstPtr &_msg)
  {
    std::cout << "Got a ROS message: " << _msg << "\n";
    this->SetVelocity(_msg->data[0], _msg->data[1]);
  }

private:
  void OnVelMsg(ConstVector3dPtr &_msg)
  {
    this->SetVelocity(_msg->x(), _msg->y());
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
  transport::NodePtr node;
  transport::SubscriberPtr sub;

  // ROS private variables
  std::unique_ptr<ros::NodeHandle> rosNode;
  ros::Subscriber rosSub;
  ros::CallbackQueue rosQueue;
  std::thread rosQueueThread;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(RomiPlugin)
}
#endif