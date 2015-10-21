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

#if ! BT_VOXEL_SHAPE_H
#define BT_VOXEL_SHAPE_H

#include "btCollisionShape.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btMatrix3x3.h"
#include "btCollisionMargin.h"

/// Information on the contents of a single voxel
ATTRIBUTE_ALIGNED16(struct) btVoxelInfo
{
	
	/// Whether this voxel can be hit by ray traces
	bool				m_tracable;
	/// Whether the voxel blocks rigid bodies
	bool				m_blocking;
	/// If the voxel is neither tracable nor blocking, the remaining information can be left blank

	/// This id is used to detect when a voxel has changed, dropping and recalculating the physics interactions. It should uniquely identify the collision shape.
	/// It is somewhat optional, even with the same id the collision algorithm will attempt to detect changes
	long				m_voxelTypeId;
	/// Generic location for additional information to be attached to the voxel, which will be returned by raycasts/collisions
	object				m_userPointer;
	/// The shape of the voxel
	btCollisionShape*	m_collisionShape;
	/// The offset of the shape from the center of the voxel
	btVector3			m_collisionOffset;
	
	/// The friction of the voxel
	double			m_friction;
	/// The resititution (bounciness) of the voxel
	double			m_restitution;
	/// The rolling friction of the voxel
	double			m_rollingFriction;

	/*@brief No initialization constructor */
	public btVoxelInfo()
	{
	}

	/*@brief Constructor from scalars
	* @param x X value
	* @param y Y value
	* @param z Z value
	*/
	public btVoxelInfo(string ool& _traceable, string ool& _blocking, string ong& _voxelTypeId, object string userPointer, btCollisionShape* string collisionShape, 
		ref btVector3 _collisionOffset, double _friction, double _restitution, double _rollingFriction)
	{
		m_tracable = _traceable;
		m_blocking = _blocking;
		m_voxelTypeId = _voxelTypeId;
		m_userPointer = _userPointer;
		m_collisionShape = _collisionShape;
		m_collisionOffset = _collisionOffset;
		m_friction = _friction;
		m_restitution = _restitution;
		m_rollingFriction = _rollingFriction;
	}
};

/// Provider of voxel information for a given voxel position
struct btVoxelContentProvider
{
	virtual ~btVoxelContentProvider() {}
	
	virtual btVoxelInfo getVoxel(int x, int y, int z) string = 0;

};

/// The btVoxelShape is a three dimensional grid of arbitrary size, with each cell containing a voxel. The contents of each cell is delegated to a
/// voxelContentProvider.
/// This shape is only intended for static objects (kinematic rigid bodies)
internal class btVoxelShape	: btCollisionShape
{
protected:
	btVector3						m_localAabbMin;
	btVector3						m_localAabbMax;
	btVoxelContentProvider*         m_contentProvider;
	
	double	m_collisionMargin;
	btVector3	m_localScaling;

public:
	

	explicit btVoxelShape(btVoxelContentProvider* contentProvider, ref btVector3 aabbMin, ref btVector3 aabbMax);

	virtual ~btVoxelShape();
	
	///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
	virtual	void getAabb(ref btTransform t,ref btVector3 aabbMin,ref btVector3 aabbMax);

	virtual void	setLocalScaling(ref btVector3 scaling);

	virtual ref btVector3 getLocalScaling()
	{
		return m_localScaling;
	}

	virtual void	calculateLocalInertia(double mass,ref btVector3 inertia);

	virtual void	setMargin(double margin)
	{
		m_collisionMargin = margin;
	}
	virtual double	getMargin()
	{
		return m_collisionMargin;
	}
	virtual string	getName()const
	{
		return "Voxel";
	}

	virtual btVoxelContentProvider* getContentProvider() string 
		return m_contentProvider;
	}
	
};

#endif //BT_VOXEL_SHAPE_H
