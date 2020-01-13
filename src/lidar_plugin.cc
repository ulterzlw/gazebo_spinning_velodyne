//
// Created by Linwei Zheng on 11/1/2020.
// Copyright (c) 2020 Linwei Zheng. All rights reserved.
//

#ifndef LIDAR_PLUGIN_H
#define LIDAR_PLUGIN_H

#include <algorithm>
#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <ros/advertise_options.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo/common/Exception.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>
#include <sdf/sdf.hh>

#include <gazebo_plugins/PubQueue.h>

#include "point_types.h"

namespace gazebo {

class GazeboRosLidar : public RayPlugin {
 public:
  GazeboRosLidar() = default;
  ~GazeboRosLidar() {
    rosnode_->shutdown();
    delete rosnode_;
  }

  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
    RayPlugin::Load(_parent, sdf);  // todo: whey sdf
    std::string worldName = _parent->WorldName();
    world_ = physics::get_world(worldName);
    sdf = _sdf;
    GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
    parent_ray_sensor_ =
        dynamic_pointer_cast<sensors::RaySensor>(_parent);
    if (!parent_ray_sensor_) gzthrow(
        "GazeboRosLaser controller requires a Ray Sensor as its parent");
    if (sdf->HasElement("robotNamespace")) {
      robot_namespace_ = sdf->Get<std::string>("robotNamespace") + "/";
      ROS_INFO_STREAM("<robotNamespace> set to: " << robot_namespace_);
    } else {
      std::string scoped_name = parent_ray_sensor_->ParentName();
      std::size_t it = scoped_name.find("::");

      robot_namespace_ = "/" + scoped_name.substr(0, it) + "/";
      ROS_WARN_STREAM(
          "missing <robotNamespace>, set to default: " << robot_namespace_);
    }
    if (!sdf->HasElement("frameName")) {
      ROS_INFO_NAMED("lidar",
                     "Lider plugin missing <frameName>, default to /world");
      frame_name_ = "/world";
    } else {
      frame_name_ = sdf->Get<std::string>("frameName");
    }
    if (!sdf->HasElement("topicName")) {
      ROS_INFO_NAMED("lidar",
                     "Lidar plugin missing <topicNmae>, default to /world");
      topic_name_ = "/world";
    } else {
      topic_name_ = sdf->Get<std::string>("topicName");
    }
    laser_connect_count_ = 0;
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM_NAMED(
          "lidar",
          "A ROS node has not been initialized, unable to load lidar plugin.");
      return;
    }
    ROS_INFO_NAMED("laser", "Starting Laser Plugin (namespace = %s",
                   robot_namespace_.c_str());
    deferred_load_thread_ =
        boost::thread(boost::bind(&GazeboRosLidar::LoadThread, this));
  }

 private:
  void LoadThread() {
    gazebo_node_ =
        gazebo::transport::NodePtr(new gazebo::transport::Node());
    gazebo_node_->Init(world_name_);
    pmq.startServiceThread();
    rosnode_ = new ros::NodeHandle(robot_namespace_);
    tf_prefix_ = tf::getPrefixParam(*rosnode_);
    if (tf_prefix_.empty()) {
      tf_prefix_ = robot_namespace_;
      boost::trim_right_if(tf_prefix_, boost::is_any_of("/"));
    }
    ROS_INFO_NAMED("lidar",
                   "Lidar Plugin (namespace = %s) <tf_prefix_>, set t \"%s\"",
                   robot_namespace_.c_str(), tf_prefix_.c_str());
    if (topic_name_ != "") {
      ros::AdvertiseOptions ao =
          ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
              topic_name_, 1,
              boost::bind(&GazeboRosLidar::LaserConnect, this),
              boost::bind(&GazeboRosLidar::LaserDisconnect, this),
              ros::VoidPtr(), NULL);
      pub_ = rosnode_->advertise(ao);
      pub_queue_ = pmq.addPub<sensor_msgs::PointCloud2>();
    }
    parent_ray_sensor_->SetActive(false);
  }
  void LaserConnect() {
    laser_connect_count_++;
    if (laser_connect_count_ == 1)
      laser_scan_sub_ = gazebo_node_->Subscribe(
          parent_ray_sensor_->Topic(), &GazeboRosLidar::OnScan, this);
  }
  void LaserDisconnect() {
    laser_connect_count_--;
    if (laser_connect_count_ == 0) laser_scan_sub_.reset();
  }
  void OnScan(ConstLaserScanStampedPtr& _msg) {
    static float angle_min = parent_ray_sensor_->AngleMin().Radian();
    static float angle_max = parent_ray_sensor_->AngleMax().Radian();
    static float angle_resolution = parent_ray_sensor_->AngleResolution();
    static int ray_count = parent_ray_sensor_->RayCount();
    float
        measure_time =
        parent_ray_sensor_->LastMeasurementTime().Float() - delay_time_;
    measure_time = fmod(measure_time, 1 / frequency_);
    bool new_frame = !pc_.empty()
        && measure_time < 5e-3
        && pc_.back().azimuth > measure_time;
    float angle_revolution = -measure_time * frequency_ * 2 * M_PI + M_PI;
    for (int ring = 0; ring < ray_count; ++ring) {
      float range = parent_ray_sensor_->LaserShape()->GetRange(ring);
      if (range < parent_ray_sensor_->RangeMax()
          && range > parent_ray_sensor_->RangeMin()) {
        float angle = (float) ring * angle_resolution + angle_min;
        float
            intensity = parent_ray_sensor_->LaserShape()->GetRetro(ring);
        float x = range * cosf(angle) * cosf(angle_revolution);
        float y = range * cosf(angle) * sinf(angle_revolution);
        float z = range * sinf(angle);
        pc_.push_back(PointXYZIRT(x,
                                  y,
                                  z,
                                  intensity,
                                  ring,
                                  measure_time));
      }
    }
    if (new_frame) {
      sensor_msgs::PointCloud2 pc2_msg;
      pcl::toROSMsg(pc_, pc2_msg);
      pc2_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec())
          - ros::Duration(delay_time_);
      pc2_msg.header.frame_id = frame_name_;
      pub_queue_->push(pc2_msg, pub_);
      pc_.clear();
    }
  }

 private:
  int laser_connect_count_;
  GazeboRosPtr gazebo_ros_;
  std::string world_name_;
  physics::WorldPtr world_;
  sensors::RaySensorPtr parent_ray_sensor_;

  ros::NodeHandle* rosnode_;
  ros::Publisher pub_;
  PubQueue<sensor_msgs::PointCloud2>::Ptr pub_queue_;
  pcl::PointCloud<PointXYZIRT> pc_;

  std::string topic_name_;
  std::string frame_name_;
  std::string tf_prefix_;
  std::string robot_namespace_;

  sdf::ElementPtr sdf;
  boost::thread deferred_load_thread_;

  gazebo::transport::NodePtr gazebo_node_;
  gazebo::transport::SubscriberPtr laser_scan_sub_;

  PubMultiQueue pmq;

  float frequency_ = 10;
  float delay_time_ = 0.0009260680220259; // for unknown gazebo raysensor delay
};
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosLidar)
}  // namespace gazebo

#endif