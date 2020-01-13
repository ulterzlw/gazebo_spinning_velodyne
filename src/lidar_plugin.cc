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
    this->rosnode_->shutdown();
    delete this->rosnode_;
  }

  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
    RayPlugin::Load(_parent, this->sdf);  // todo: whey sdf
    std::string worldName = _parent->WorldName();
    this->world_ = physics::get_world(worldName);
    this->sdf = _sdf;
    GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
    this->parent_ray_sensor_ =
        dynamic_pointer_cast<sensors::RaySensor>(_parent);
    if (!this->parent_ray_sensor_) gzthrow(
        "GazeboRosLaser controller requires a Ray Sensor as its parent");
    this->robot_namespace_ = GetRobotNamespace(_parent, _sdf, "Lidar");
    if (!this->sdf->HasElement("frameName")) {
      ROS_INFO_NAMED("lidar",
                     "Lider plugin missing <frameName>, default to /world");
      this->frame_name_ = "/world";
    } else {
      this->frame_name_ = this->sdf->Get<std::string>("frameName");
    }
    if (!this->sdf->HasElement("topicName")) {
      ROS_INFO_NAMED("lidar",
                     "Lidar plugin missing <topicNmae>, default to /world");
      this->topic_name_ = "/world";
    } else {
      this->topic_name_ = this->sdf->Get<std::string>("topicName");
    }
    this->laser_connect_count_ = 0;
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM_NAMED(
          "lidar",
          "A ROS node has not been initialized, unable to load lidar plugin.");
      return;
    }
    ROS_INFO_NAMED("laser", "Starting Laser Plugin (namespace = %s",
                   this->robot_namespace_.c_str());
    this->deferred_load_thread_ =
        boost::thread(boost::bind(&GazeboRosLidar::LoadThread, this));
  }

 private:
  void LoadThread() {
    this->gazebo_node_ =
        gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->gazebo_node_->Init(this->world_name_);
    this->pmq.startServiceThread();
    this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);
    this->tf_prefix_ = tf::getPrefixParam(*this->rosnode_);
    if (this->tf_prefix_.empty()) {
      this->tf_prefix_ = this->robot_namespace_;
      boost::trim_right_if(this->tf_prefix_, boost::is_any_of("/"));
    }
    ROS_INFO_NAMED("lidar",
                   "Lidar Plugin (namespace = %s) <tf_prefix_>, set t \"%s\"",
                   this->robot_namespace_.c_str(), this->tf_prefix_.c_str());
    this->frame_name_ = tf::resolve(this->tf_prefix_, this->frame_name_);
    if (this->topic_name_ != "") {
      ros::AdvertiseOptions ao =
          ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
              this->topic_name_, 1,
              boost::bind(&GazeboRosLidar::LaserConnect, this),
              boost::bind(&GazeboRosLidar::LaserDisconnect, this),
              ros::VoidPtr(), NULL);
      this->pub_ = this->rosnode_->advertise(ao);
      this->pub_queue_ = this->pmq.addPub<sensor_msgs::PointCloud2>();
    }
    this->parent_ray_sensor_->SetActive(false);
  }
  void LaserConnect() {
    this->laser_connect_count_++;
    if (this->laser_connect_count_ == 1)
      this->laser_scan_sub_ = this->gazebo_node_->Subscribe(
          this->parent_ray_sensor_->Topic(), &GazeboRosLidar::OnScan, this);
  }
  void LaserDisconnect() {
    this->laser_connect_count_--;
    if (this->laser_connect_count_ == 0) this->laser_scan_sub_.reset();
  }
  void OnScan(ConstLaserScanStampedPtr& _msg) {
    static float angle_min = this->parent_ray_sensor_->AngleMin().Radian();
    static float angle_max = this->parent_ray_sensor_->AngleMax().Radian();
    static float angle_resolution = this->parent_ray_sensor_->AngleResolution();
    static int ray_count = this->parent_ray_sensor_->RayCount();
    float
        measure_time = this->parent_ray_sensor_->LastMeasurementTime().Float();
    measure_time = fmod(measure_time, 0.1f);
    bool new_frame(false);
    if (!pc_.empty() && measure_time < 1e-3
        && measure_time < this->pc_.back().azimuth)
      new_frame = true;
    float angle_revolution = -measure_time / 0.1 * 2 * M_PI + M_PI;
    for (int ring = 0; ring < ray_count; ++ring) {
      float range = this->parent_ray_sensor_->LaserShape()->GetRange(ring);
      if (range < this->parent_ray_sensor_->RangeMax()
          && range > this->parent_ray_sensor_->RangeMin()) {
        float angle = (float) ring * angle_resolution + angle_min;
        float
            intensity = this->parent_ray_sensor_->LaserShape()->GetRetro(ring);
        float x = range * cosf(angle) * cosf(angle_revolution);
        float y = range * cosf(angle) * sinf(angle_revolution);
        float z = range * sinf(angle);
        this->pc_.push_back(PointXYZIRT(x,
                                        y,
                                        z,
                                        intensity,
                                        ring,
                                        measure_time));
      }
    }
    if (new_frame) {
      sensor_msgs::PointCloud2 pc2_msg;
      pcl::toROSMsg(this->pc_, pc2_msg);
      pc2_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
      pc2_msg.header.frame_id = this->frame_name_;
      this->pub_queue_->push(pc2_msg, this->pub_);
      this->pc_.clear();
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
};
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosLidar)
}  // namespace gazebo

#endif