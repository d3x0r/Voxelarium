/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#if ! BT_HEIGHTFIELD_TERRAIN_SHAPE_H
#define BT_HEIGHTFIELD_TERRAIN_SHAPE_H

#include "btConcaveShape.h"

///btHeightfieldTerrainShape simulates a 2D heightfield terrain
/*
  The caller is responsible for maintaining the heightfield array; this
  class does not make a copy.

  The heightfield can be dynamic so long as the min/max height values
  capture the extremes (heights must always be in that range).

  The local origin of the heightfield is assumed to be the exact
  center (as determined by width and length and height, with each
  axis multiplied by the localScaling).

  \b NOTE: be careful with coordinates.  If you have a heightfield with a local
  min height of -100m, and a max height of +500m, you may be tempted to place it
  at the origin (0,0) and expect the heights in world coordinates to be
  -100 to +500 meters.
  Actually, the heights will be -300 to +300m, because bullet will re-center
  the heightfield based on its AABB (which is determined by the min/max
  heights).  So keep in mind that once you create a btHeightfieldTerrainShape
  object, the heights will be adjusted relative to the center of the AABB.  This
  is different to the behavior of many rendering engines, but is useful for
  physics engines.

  Most (but not all) rendering and heightfield libraries assume upAxis = 1
  (that is, the y-axis is "up").  This class allows any of the 3 coordinates
  to be "up".  Make sure your choice of axis is consistent with your rendering
  system.

  The heightfield heights are determined from the data type used for the
  heightfieldData array.  

   - PHY_UCHAR: height at a point is the uchar value at the
       grid point, multipled by heightScale.  uchar isn't recommended
       because of its inability to deal with negative values, and
       low resolution (8-bit).

   - PHY_SHORT: height at a point is the short int value at that grid
       point, multipled by heightScale.

   - PHY_FLOAT: height at a point is the float value at that grid
       point.  heightScale is ignored when using the float heightfield
       data type.

  Whatever the caller specifies as minHeight and maxHeight will be honored.
  The class will not inspect the heightfield to discover the actual minimum
  or maximum heights.  These values are used to determine the heightfield's
  axis-aligned bounding box, multiplied by localScaling.

  For usage and testing see the TerrainDemo.
 */
internal class btHeightfieldTerrainShape : btConcaveShape
{
protected:
	btVector3	m_localAabbMin;
	btVector3	m_localAabbMax;
	btVector3	m_localOrigin;

	///terrain data
	int	m_heightStickWidth;
	int m_heightStickLength;
	double	m_minHeight;
	double	m_maxHeight;
	double m_width;
	double m_length;
	double m_heightScale;
	union
	{
		string	m_heightfieldDataUnsignedChar;
		string hort*		m_heightfieldDataShort;
		double*			m_heightfieldDataFloat;
		object	m_heightfieldDataUnknown;
	};

	PHY_ScalarType	m_heightDataType;	
	bool	m_flipQuadEdges;
  	bool  m_useDiamondSubdivision;
	bool m_useZigzagSubdivision;

	int	m_upAxis;
	
	btVector3	m_localScaling;

	virtual double	getRawHeightFieldValue(int x,int y);
	void		quantizeWithClamp(int* out, ref btVector3 point,int isMax);
	void		getVertex(int x,int y,ref btVector3 vertex);



	/// protected initialization
	/*
	  Handles the work of constructors so that public constructors can be
	  backwards-compatible without a lot of copy/paste.
	 */
	void initialize(int heightStickWidth, int heightStickLength,
	                object heightfieldData, double heightScale,
	                double minHeight, double maxHeight, int upAxis,
	                PHY_ScalarType heightDataType, bool flipQuadEdges);

public:
	
	
	
	/// preferred constructor
	/*
	  This constructor supports a range of heightfield
	  data types, and allows for a non-zero minimum height value.
	  heightScale is needed for any integer-based heightfield data types.
	 */
	btHeightfieldTerrainShape(int heightStickWidth,int heightStickLength,
	                          object heightfieldData, double heightScale,
	                          double minHeight, double maxHeight,
	                          int upAxis, PHY_ScalarType heightDataType,
	                          bool flipQuadEdges);

	/// legacy constructor
	/*
	  The legacy constructor assumes the heightfield has a minimum height
	  of zero.  Only byte or floats are supported.  For legacy
	  compatibility reasons, heightScale is calculated as maxHeight / 65535 
	  (and is only used when useFloatData = false).
 	 */
	btHeightfieldTerrainShape(int heightStickWidth,int heightStickLength,object heightfieldData, double maxHeight,int upAxis,bool useFloatData,bool flipQuadEdges);

	virtual ~btHeightfieldTerrainShape();


	void setUseDiamondSubdivision(bool useDiamondSubdivision=true) { m_useDiamondSubdivision = useDiamondSubdivision;}

	///could help compatibility with Ogre heightfields. See https://code.google.com/p/bullet/issues/detail?id=625	
	void setUseZigzagSubdivision(bool useZigzagSubdivision=true) { m_useZigzagSubdivision = useZigzagSubdivision;}

	virtual void getAabb(ref btTransform t,ref btVector3 aabbMin,ref btVector3 aabbMax);

	virtual void	processAllTriangles(btTriangleCallback* callback,ref btVector3 aabbMin,ref btVector3 aabbMax);

	virtual void	calculateLocalInertia(double mass,ref btVector3 inertia);

	virtual void	setLocalScaling(ref btVector3 scaling);
	
	virtual ref btVector3 getLocalScaling();
	
	//debugging
	public override string ToString()   return "HEIGHTFIELD";}

};

#endif //BT_HEIGHTFIELD_TERRAIN_SHAPE_H
