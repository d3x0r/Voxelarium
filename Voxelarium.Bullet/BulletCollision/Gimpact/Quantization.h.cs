#if ! BT_GIMPACT_QUANTIZATION_H_INCLUDED
#define BT_GIMPACT_QUANTIZATION_H_INCLUDED

/*! \file btQuantization.h
*\author Francisco Leon Najera

*/
/*
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "LinearMath/btTransform.h"






public void bt_calc_quantization_parameters(
	btVector3  outMinBound,
	btVector3  outMaxBound,
	btVector3  bvhQuantization,
	ref btVector3 srcMinBound,ref btVector3 srcMaxBound,
	double quantizationMargin)
{
	//enlarge the AABB to avoid division by zero when initializing the quantization values
	btVector3 clampValue(quantizationMargin,quantizationMargin,quantizationMargin);
	outMinBound = srcMinBound - clampValue;
	outMaxBound = srcMaxBound + clampValue;
	btVector3 aabbSize = outMaxBound - outMinBound;
	bvhQuantization = btVector3((double)(65535.0),
								(double)(65535.0),
								(double)(65535.0)) / aabbSize;
}


public void bt_quantize_clamp(
	ushort* out,
	ref btVector3 point,
	btVector3  min_bound,
	btVector3  max_bound,
	btVector3  bvhQuantization)
{

	btVector3 clampedPoint(point);
	clampedPoint.setMax(min_bound);
	clampedPoint.setMin(max_bound);

	btVector3 v = (clampedPoint - min_bound) * bvhQuantization;
	out[0] = (ushort)(v.x+0.5f);
	out[1] = (ushort)(v.y+0.5f);
	out[2] = (ushort)(v.z+0.5f);
}


public btVector3 bt_unquantize(
	string nsigned short* vecIn,
	btVector3  offset,
	btVector3  bvhQuantization)
{
	btVector3	vecOut;
	vecOut.setValue(
		(double)(vecIn[0]) / (bvhQuantization.x),
		(double)(vecIn[1]) / (bvhQuantization.y),
		(double)(vecIn[2]) / (bvhQuantization.z));
	vecOut += offset;
	return vecOut;
}



#endif // BT_GIMPACT_QUANTIZATION_H_INCLUDED
