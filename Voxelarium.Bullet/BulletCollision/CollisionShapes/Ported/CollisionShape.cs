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

using Bullet.Collision.BroadPhase;
using Bullet.LinearMath;
using System;

namespace Bullet.Collision.Shapes
{


	///The btCollisionShape class provides an interface for collision shapes that can be shared among btCollisionObjects.
	public abstract class btCollisionShape
	{
		protected BroadphaseNativeTypes m_shapeType;
		protected object m_userPointer;
		protected int m_userIndex;
		internal Type bvhIndexType;
		internal Type bvhDataType;

		public btCollisionShape()
		{
			m_shapeType = BroadphaseNativeTypes.INVALID_SHAPE_PROXYTYPE;
			m_userPointer = null;
			m_userIndex = -1;
		}

		///getAabb returns the axis aligned bounding box in the coordinate frame of the given transform t.
		public abstract void getAabb( ref btTransform t, out btVector3 aabbMin, out btVector3 aabbMax );
		//public abstract void getAabb( btITransform t, out btVector3 aabbMin, out btVector3 aabbMax );

		///getAngularMotionDisc returns the maximus radius needed for Conservative Advancement to handle time-of-impact with rotations.



		public bool isPolyhedral()
		{
			return btBroadphaseProxy.isPolyhedral( getShapeType() );
		}

		public bool isConvex2d()
		{
			return btBroadphaseProxy.isConvex2d( getShapeType() );
		}

		public bool isConvex()
		{
			return btBroadphaseProxy.isConvex( getShapeType() );
		}
		public bool isNonMoving()
		{
			return btBroadphaseProxy.isNonMoving( getShapeType() );
		}
		public bool isConcave()
		{
			return btBroadphaseProxy.isConcave( getShapeType() );
		}
		public bool isCompound()
		{
			return btBroadphaseProxy.isCompound( getShapeType() );
		}

		public bool isSoftBody()
		{
			return btBroadphaseProxy.isSoftBody( getShapeType() );
		}

		///isInfinite is used to catch simulation error (aabb check)
		public bool isInfinite()
		{
			return btBroadphaseProxy.isInfinite( getShapeType() );
		}

		public abstract void setLocalScaling( ref btVector3 scaling );
		public abstract void getLocalScaling( out btVector3 scaling );
		public abstract void calculateLocalInertia( double mass, out btVector3 inertia );

		//debugging support
		//public abstract string	getName() ;


		public BroadphaseNativeTypes getShapeType() { return m_shapeType; }

		///the getAnisotropicRollingFrictionDirection can be used in combination with setAnisotropicFriction
		///See Bullet/Demos/RollingFrictionDemo for an example
		public virtual void getAnisotropicRollingFrictionDirection( out btVector3 result )
		{
			result = new btVector3( 1, 1, 1 );
		}
		public abstract void setMargin( double margin );
		public abstract double getMargin();


		///optional user data pointer
		void setUserPointer( object userPtr )
		{
			m_userPointer = userPtr;
		}

		object getUserPointer()
		{
			return m_userPointer;
		}
		void setUserIndex( int index )
		{
			m_userIndex = index;
		}

		int getUserIndex()
		{
			return m_userIndex;
		}


#if asdfaasdf
		///fills the dataBuffer and returns the struct name (and 0 on failure)
		virtual	string	serialize(object dataBuffer, btSerializer* serializer);

	virtual void	serializeSingleShape(btSerializer* serializer);
#endif

		public virtual void getBoundingSphere( out btVector3 center, out double radius )
		{
			btVector3 aabbMin, aabbMax;

			getAabb( ref btTransform.Identity, out aabbMin, out aabbMax );

			btVector3 tmp;
			aabbMax.Sub( ref aabbMin, out tmp );
			radius = tmp.length() * (double)( 0.5 );
			aabbMax.Add( ref aabbMin, out tmp );
			tmp.Mult( 0.5, out center );
		}


		public virtual double getContactBreakingThreshold( double defaultContactThreshold )
		{
			return getAngularMotionDisc() * defaultContactThreshold;
		}

		public virtual double getAngularMotionDisc()
		{
			///@todo cache this value, to improve performance
			btVector3 center;
			double disc;
			getBoundingSphere( out center, out disc );
			disc += ( center ).length();
			return disc;
		}

		///calculateTemporalAabb calculates the enclosing aabb for the moving object over interval [0..timeStep)
		///result is conservative
		public void calculateTemporalAabb( ref btTransform curTrans, ref btVector3 linvel, ref btVector3 angvel, double timeStep
			, out btVector3 temporalAabbMin, out btVector3 temporalAabbMax )
		{
			//start with static aabb
			getAabb( ref curTrans, out temporalAabbMin, out temporalAabbMax );

			double temporalAabbMaxx = temporalAabbMax.x;
			double temporalAabbMaxy = temporalAabbMax.y;
			double temporalAabbMaxz = temporalAabbMax.z;
			double temporalAabbMinx = temporalAabbMin.x;
			double temporalAabbMiny = temporalAabbMin.y;
			double temporalAabbMinz = temporalAabbMin.z;

			// add linear motion
			btVector3 linMotion;
			linvel.Mult( timeStep, out linMotion );
			///@todo: simd would have a vector max/min operation, instead of per-element access
			if( linMotion.x > btScalar.BT_ZERO )
				temporalAabbMaxx += linMotion.x;
			else
				temporalAabbMinx += linMotion.x;
			if( linMotion.y > btScalar.BT_ZERO )
				temporalAabbMaxy += linMotion.y;
			else
				temporalAabbMiny += linMotion.y;
			if( linMotion.z > btScalar.BT_ZERO )
				temporalAabbMaxz += linMotion.z;
			else
				temporalAabbMinz += linMotion.z;

			//add conservative angular motion
			double angularMotion = angvel.length() * getAngularMotionDisc() * timeStep;
			btVector3 angularMotion3d = new btVector3( angularMotion, angularMotion, angularMotion );
			temporalAabbMin = new btVector3( temporalAabbMinx, temporalAabbMiny, temporalAabbMinz );
			temporalAabbMax = new btVector3( temporalAabbMaxx, temporalAabbMaxy, temporalAabbMaxz );
			temporalAabbMin.Sub( ref angularMotion3d, out temporalAabbMin );
			temporalAabbMax.Add( ref angularMotion3d, out temporalAabbMax );
		}

#if asdfaasdf
///fills the dataBuffer and returns the struct name (and 0 on failure)
const char* btCollisionShape::serialize( void* dataBuffer, btSerializer* serializer ) const
{
	btCollisionShapeData* shapeData = (btCollisionShapeData*)dataBuffer;
char* name = (char*)serializer.findNameForPointer( this );
shapeData.m_name = (char*)serializer.getUniquePointer( name);
	if (shapeData.m_name)
	{
		serializer.serializeName( name);
	}
	shapeData.m_shapeType = m_shapeType;
	//shapeData.m_padding//??
	return "btCollisionShapeData";
}

void btCollisionShape::serializeSingleShape( btSerializer* serializer ) const
{
	int len = calculateSerializeBufferSize();
btChunk* chunk = serializer.allocate( len, 1 );
const char* structType = serialize( chunk.m_oldPtr, serializer );
serializer.finalizeChunk( chunk, structType, BT_SHAPE_CODE,(void*)this);
}
};	
#endif
		/*
		///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
		struct btCollisionShapeData
		{
			string m_name;
			int m_shapeType;
			//char m_padding[4];
		};

		public virtual int calculateSerializeBufferSize()
		{
			return sizeof( btCollisionShapeData );
		}
		*/

	}
}
