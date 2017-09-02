#ifndef _ROMI_PLUGIN_HH_
#define _ROMI_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

/* Notes:
 * http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5
 * http://gazebosim.org/tutorials?tut=plugins_hello_world&cat=write_plugin
 *
 * Compile with:
 * # cmake
 * # make
 * export export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/path/to/library/build/
 * gazebo --verbose // to get more info
 *
 * gazebo libraries at /usr/include/gazebo/

*/

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
    std::cout << "Left wheel joint: " << this->left_wheel->GetScopedName() << "\n";

    // Set up right wheel PID controller
    this->right_wheel = _model->GetJoints()[1];
    this->right_pid = common::PID(0.1, 0, 0);
    this->model->GetJointController()->SetVelocityPID(this->right_wheel->GetScopedName(), this->right_pid);
    std::cout << "Right wheel joint: " << this->right_wheel->GetScopedName() << "\n";

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
  }

public:
  void SetVelocity(const double &_left, const double &_right)
  {
    this->model->GetJointController()->SetVelocityTarget(this->left_wheel->GetScopedName(), _left);
    // right wheel is backwards so negate command
    this->model->GetJointController()->SetVelocityTarget(this->right_wheel->GetScopedName(), -_right);
  }

private:
  void OnVelMsg(ConstVector3dPtr &_msg)
  {
    this->SetVelocity(_msg->x(), _msg->y());
  }

private:
  physics::ModelPtr model;
  physics::JointPtr left_wheel;
  common::PID left_pid;
  physics::JointPtr right_wheel;
  common::PID right_pid;
  transport::NodePtr node;
  transport::SubscriberPtr sub;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(RomiPlugin)
}
#endif