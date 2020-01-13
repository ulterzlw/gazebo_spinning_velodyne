//
// Created by Linwei Zheng on 11/1/2020.
// Copyright (c) 2020 Linwei Zheng. All rights reserved.
//

#ifndef POINT_TYPES_H
#define POINT_TYPES_H

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>

namespace gazebo {

struct PointXYZIRT {
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  uint8_t ring;
  float azimuth;

  PointXYZIRT() {
    x = y = z = 0;
    intensity = 0;
    ring = 0;
    azimuth = 0;
  }

  PointXYZIRT(float _x, float _y, float _z, float _i, uint8_t _r, float _t)
      : x(_x), y(_y), z(_z), intensity(_i), ring(_r), azimuth(_t) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
}  // namespace gazebo

POINT_CLOUD_REGISTER_POINT_STRUCT(gazebo::PointXYZIRT,
                                  (float, x, x)
                                      (float, y, y)
                                      (float, z, z)
                                      (float, intensity, intensity)
                                      (uint8_t, ring, ring)
                                      (float, azimuth, azimuth))

#endif  // POINT_TYPES_H
