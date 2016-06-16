/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/
//Author: 荆明轩
//Source code: https://github.com/id9502/Quadrotor

#include "attitude.h"

quater attitude_Q;
float_vect3 attitude_velocity;
float_vect3 attitude_position;
float_vect3 attitude_gravity;
float_vect3 attitude_acceleration;

void attitudeUpdate(float dt)
{
	////计算四元数姿态
	quater current_Q = convert2Q(GYRO_v3f.x*dt, GYRO_v3f.y*dt, GYRO_v3f.z*dt);
	attitude_Q = qmult_v(attitude_Q, current_Q);
	attitude_acceleration = vsub(attitude_gravity,qrotate(attitude_Q,ACC_v3f));//相对地面的加速度
	////计算积分相对地面位置
	attitude_position = vadd(attitude_position,vadd(vmultn(attitude_velocity,dt),
			vmultn(attitude_acceleration,0.5f*dt*dt)));	//零阶保持积分
	////计算积分相对地面速度（减去重力）；
	attitude_velocity = vadd(attitude_velocity,vmultn(attitude_acceleration,dt));	//零阶保持积分
}

void attitudeAdjust()
{
	attitude_Q.r = 1.0f;
	attitude_Q.i = attitude_Q.j = attitude_Q.k = 0.0f;
	attitude_velocity.x = attitude_velocity.y = attitude_velocity.z = 0.0f;
	attitude_position.x = attitude_position.y = attitude_position.z = 0.0f;
	attitude_acceleration.x = attitude_acceleration.y = attitude_acceleration.z = 0.0f;
}
