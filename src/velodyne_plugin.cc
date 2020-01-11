#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float32.h>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <thread>

namespace gazebo {
/// \brief A plugin to control a Velodyne sensor.
class VelodynePlugin : public ModelPlugin {
 public:
  /// \brief Constructor
  VelodynePlugin() {}

  /// \brief The load function is called by Gazebo when the plugin is
  /// inserted into simulation
  /// \param[in] _model A pointer to the model that this plugin is
  /// attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    // Safety check
    if (_model->GetJointCount() == 0) {
      std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
      return;
    }

    // Store the model pointer for convenience.
    this->model = _model;

    // Get the first joint. We are making an assumption about the model
    // having one joint that is the rotational joint.
    this->joint = _model->GetJoints()[0];

    // Setup a P-controller, with a gain of 0.1.
    this->pid = common::PID(0.1, 0, 0);

    // Apply the P-controller to the joint.
    this->model->GetJointController()->SetVelocityPID(
        this->joint->GetScopedName(), this->pid);

    this->SetVelocity(-20 * M_PI);

    // Create the node
    this->node = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION < 8
    this->node->Init(this->model->GetWorld()->GetName());
#else
    this->node->Init(this->model->GetWorld()->Name());
#endif

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized()) {
      int argc = 0;
      char** argv = NULL;
      ros::init(argc, argv, "gazebo_client",
                ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
  }

 private:
  /// \brief Set the velocity of the Velodyne
  /// \param[in] _vel New target velocity
  void SetVelocity(const double& _vel) {
    // Set the joint's target velocity.
    this->model->GetJointController()->SetVelocityTarget(
        this->joint->GetScopedName(), _vel);
  }

 private:
  /// \brief A node used for transport
  transport::NodePtr node;

  /// \brief A subscriber to a named topic.
  transport::SubscriberPtr sub;

  /// \brief Pointer to the model.
  physics::ModelPtr model;

  /// \brief Pointer to the joint.
  physics::JointPtr joint;

  /// \brief A PID controller for the joint.
  common::PID pid;

  /// \brief A node use for ROS transport
  std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief A ROS subscriber
  ros::Subscriber rosSub;

  /// \brief A ROS callbackqueue that helps process messages
  ros::CallbackQueue rosQueue;

  /// \brief A thread the keeps running the rosQueue
  std::thread rosQueueThread;  /// \brief Handle an incoming message from ROS
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}  // namespace gazebo
#endif