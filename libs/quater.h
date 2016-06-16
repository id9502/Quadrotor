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

#ifndef __QUATER_H__
#define __QUATER_H__

#include <math.h>
#include "sys.h"

quater qmult_v(quater p, quater q);		// p * q
quater qmult_n(quater p, float n);		// n * p
float qmult_dot(quater p, quater q);	// p .* q
float qL2(quater p);					// ||p|| = (p .* p`)
float qabs(quater p);					// |p| = ¡Ì(p .* p`)
quater qadd(quater p, quater q);		// p + q
quater qsub(quater p, quater q);		// p - q
quater qinv(quater p);					// -p
quater qconj(quater p);					// p`
quater qreci(quater p);					// p^-1

float_vect3 vcross(float_vect3 a, float_vect3 b);//a x b
float vdot(float_vect3 a, float_vect3 b);		//a * b
float vabs(float_vect3 a);						//||a||
float_vect3 vmultn(float_vect3 a, float b);		//a * b
float_vect3 vdivn(float_vect3 a, float b); 		//a / b
float_vect3 vsub(float_vect3 a, float_vect3 b);	//a - b
float_vect3 vadd(float_vect3 a, float_vect3 b);	//a + b

quater qfromv3f(float_vect3 u);
float_vect3 qtov3f(quater u);
quater convert2Q(float wx, float wy, float wz);
float_vect3 qrotate(quater q, float_vect3 v);//q*v*q'

#endif // __QUATER_H__
