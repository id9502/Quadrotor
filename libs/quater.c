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
//Author: ¾£Ã÷Ðù
//Source code: https://github.com/id9502/Quadrotor

#include "quater.h"

#define r0	r.r
#define r1	r.i
#define r2	r.j
#define r3	r.k
#define p0	p.r
#define p1	p.i
#define p2	p.j
#define p3	p.k
#define q0	q.r
#define q1	q.i
#define q2	q.j
#define q3	q.k

inline quater qmult_v(quater p, quater q)
{
	quater r;
	r0 = p0*q0 - p1*q1 - p2*q2 - p3*q3;
	r1 = p0*q1 + p1*q0 + p2*q3 - p3*q2;
	r2 = p0*q2 + p2*q0 + p3*q1 - p1*q3;
	r3 = p0*q3 + p3*q0 + p1*q2 - p2*q1;
	return r;
}

inline quater qmult_n(quater p, float n)
{
	quater r;
	r0 = n*p0;
	r1 = n*p1;
	r2 = n*p2;
	r3 = n*p3;
	return r;
}

inline float qmult_dot(quater p, quater q)
{
	float r = 0;
	r += p0*q0 + p1*q1 + p2*q2 + p3*q3;
	return r;
}

inline float qabs(quater p)
{
	return sqrtf(qmult_dot(p,p));
}

inline quater qadd(quater p, quater q)
{
	quater r;
	r0 = p0 + q0;
	r1 = p1 + q1;
	r2 = p2 + q2;
	r3 = p3 + q3;
	return r;
}

inline quater qsub(quater p, quater q)
{
	quater r;
	r0 = p0 - q0;
	r1 = p1 - q1;
	r2 = p2 - q2;
	r3 = p3 - q3;
	return r;
}

inline quater qinv(quater p)
{
	quater r;
	r0 = -p0;
	r1 = -p1;
	r2 = -p2;
	r3 = -p3;
	return r;
}

inline quater qconj(quater p)
{
	quater r;
	r0 = p0;
	r1 = -p1;
	r2 = -p2;
	r3 = -p3;
	return r;
}

inline float qL2(quater p)
{
	return qmult_dot(p,p);
}

inline quater qreci(quater p)
{
	float div = qL2(p);
	quater r;
	r0 = p0/div;
	r1 = -p1/div;
	r2 = -p2/div;
	r3 = -p3/div;
	return r;
}

inline quater convert2Q(float wx, float wy, float wz)
{
	quater r;
	float dtheta = sqrtf(wx*wx + wy*wy + wz*wz);
	if(dtheta == 0.0){
		r0 = 1.0;
		r1 = r2 = r3 = 0.0;
		return r;
	}else{
		r0 = cosf(dtheta/2.0);
		r1 = -sinf(dtheta/2.0) * wx / dtheta;
		r2 = -sinf(dtheta/2.0) * wy / dtheta;
		r3 = -sinf(dtheta/2.0) * wz / dtheta;
		return r;
	}
}

inline quater qfromv3f(float_vect3 u)
{
	quater r;
	r0 = 0.0f;
	r1 = u.x;
	r2 = u.y;
	r3 = u.z;
	return r;
}

inline float_vect3 qtov3f(quater u)
{
	float_vect3 v;
	v.x = u.i;
	v.y = u.j;
	v.z = u.k;
	return v;
}

float_vect3 qrotate(quater q, float_vect3 v)//////rotate from current coordinate system to global system.
{
	quater forkq = qfromv3f(v);
	forkq = qmult_v(qmult_v(qconj(q),forkq),q);
	return qtov3f(forkq);
}

float_vect3 vcross(float_vect3 a, float_vect3 b)
{
	float_vect3 ret;
	ret.x = a.y*b.z - a.z*b.y;
	ret.y = a.z*b.x - a.x*b.z;
	ret.z = a.x*b.y - a.y*b.x;
	return ret;
}
inline float vdot(float_vect3 a, float_vect3 b)
{
	return a.x*b.x + a.y*b.y + a.z*b.z;
}

inline float vabs(float_vect3 a)
{
	return sqrtf(vdot(a,a));
}

inline float_vect3 vmultn(float_vect3 a, float b)
{
	float_vect3 ret;
	ret.x = a.x * b;
	ret.y = a.y * b;
	ret.z = a.z * b;
	return ret;
}

inline float_vect3 vdivn(float_vect3 a, float b)
{
	float_vect3 ret;
	ret.x = a.x / b;
	ret.y = a.y / b;
	ret.z = a.z / b;
	return ret;
}

inline float_vect3 vsub(float_vect3 a, float_vect3 b)
{
	float_vect3 ret;
	ret.x = a.x - b.x;
	ret.y = a.y - b.y;
	ret.z = a.z - b.z;
	return ret;
}

inline float_vect3 vadd(float_vect3 a, float_vect3 b)
{
	float_vect3 ret;
	ret.x = a.x + b.x;
	ret.y = a.y + b.y;
	ret.z = a.z + b.z;
	return ret;
}
