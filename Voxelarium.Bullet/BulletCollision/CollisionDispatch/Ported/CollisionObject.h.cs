/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

using System.Collections.Generic;
using Bullet.Collision.BroadPhase;
using Bullet.Collision.Shapes;
using Bullet.LinearMath;
using Bullet.Types;
using Bullet.Dynamics.ConstraintSolver;

namespace Bullet.Collision.Dispatch
{

	//island management, m_activationState1
	public enum ActivationState
	{
		ACTIVE_TAG = 1,
		ISLAND_SLEEPING = 2,
		WANTS_DEACTIVATION = 3,
		DISABLE_DEACTIVATION = 4,
		DISABLE_SIMULATION = 5
	};

	//struct	btBroadphaseProxy;
	//class	btCollisionShape;
	//struct btCollisionShapeData;

	public class btCollisionObjectArray : btList<btCollisionObject>
	{
	}
	/*
	# ifdef BT_USE_DOUBLE_PRECISION
	#define btCollisionObjectData btCollisionObjectDoubleData
	#define btCollisionObjectDataName "btCollisionObjectDoubleData"
	#else
	#define btCollisionObjectData btCollisionObjectFloatData
	#define btCollisionObjectDataName "btCollisionObjectFloatData"
	#endif
	*/

	/// btCollisionObject can be used to manage collision detection objects. 
	/// btCollisionObject maintains all information that is needed for a collision detection: Shape, Transform and AABB proxy.
	/// They can be added to the btCollisionWorld.
	public partial class btCollisionObject
	{



		public btTransform m_worldTransform;

		///m_interpolationWorldTransform is used for CCD and interpolation
		///it can be either previous or future (predicted) transform
		public btTransform m_interpolationWorldTransform;
		//those two are experimental: just added for bullet time effect, so you can still apply impulses (directly modifying velocities) 
		//without destroying the continuous interpolated motion (which uses this interpolation velocities)
		protected btVector3 m_interpolationLinearVelocity;
		protected btVector3 m_interpolationAngularVelocity;

		public btVector3 m_anisotropicFriction;
		protected AnisotropicFrictionFlags m_hasAnisotropicFriction;
		protected double m_contactProcessingThreshold;

		protected btBroadphaseProxy m_broadphaseHandle;
		protected btCollisionShape m_collisionShape;
		///m_extensionPointer is used by some internal low-level Bullet extensions.
		protected object m_extensionPointer;

		///m_rootCollisionShape is temporarily used to store the original collision shape
		///The m_collisionShape might be temporarily replaced by a child collision shape during collision detection purposes
		///If it is NULL, the m_collisionShape is not temporarily replaced.
		protected btCollisionShape m_rootCollisionShape;

		protected CollisionFlags m_collisionFlags;

		protected int m_islandTag1;
		//protected int m_companionId;
		internal btSolverBody m_companionBody;

		internal ActivationState m_activationState1;
		protected double m_deactivationTime;

		protected double m_friction;
		protected double m_restitution;
		internal double m_rollingFriction;

		///m_internalType is reserved to distinguish Bullet's btCollisionObject, btRigidBody, btSoftBody, btGhostObject etc.
		///do not assign your own m_internalType unless you write a new dynamics object class.
		protected CollisionObjectTypes m_internalType;

		///users can point to their objects, m_userPointer is not used by Bullet, see setUserPointer/getUserPointer

		protected object m_userObjectPointer;

		protected int m_userIndex;

		///time of impact calculation
		protected double m_hitFraction;

		///Swept sphere radius (0.0 by default), see btConvexConvexAlgorithm::
		protected double m_ccdSweptSphereRadius;

		/// Don't do continuous collision detection if the motion (in one step) is less then m_ccdMotionThreshold
		protected double m_ccdMotionThreshold;

		/// If some object should have elaborate collision filtering by sub-classes
		protected bool m_checkCollideWith;

		protected btList<btCollisionObject> m_objectsWithoutCollisionCheck;

		///internal update revision number. It will be increased when the object changes. This allows some subsystems to perform lazy evaluation.
		protected int m_updateRevision;




		//	

		public enum CollisionFlags
		{
			CF_STATIC_OBJECT = 1,
			CF_KINEMATIC_OBJECT = 2,
			CF_NO_CONTACT_RESPONSE = 4,
			CF_CUSTOM_MATERIAL_CALLBACK = 8,//this allows per-triangle material (friction/restitution)
			CF_CHARACTER_OBJECT = 16,
			CF_DISABLE_VISUALIZE_OBJECT = 32, //disable debug drawing
			CF_DISABLE_SPU_COLLISION_PROCESSING = 64//disable parallel/SPU processing
		};

		public enum CollisionObjectTypes
		{
			CO_COLLISION_OBJECT = 1,
			CO_RIGID_BODY = 2,
			///CO_GHOST_OBJECT keeps track of all objects overlapping its AABB and that pass its collision filter
			///It is useful for collision sensors, explosion objects, character controller etc.
			CO_GHOST_OBJECT = 4,
			CO_SOFT_BODY = 8,
			CO_HF_FLUID = 16,
			CO_USER_TYPE = 32,
			CO_FEATHERSTONE_LINK = 64
		};

		public enum AnisotropicFrictionFlags
		{
			CF_ANISOTROPIC_FRICTION_DISABLED = 0,
			CF_ANISOTROPIC_FRICTION = 1,
			CF_ANISOTROPIC_ROLLING_FRICTION = 2
		};

		public bool mergesSimulationIslands()
		{
			///static objects, kinematic and object without contact response don't merge islands
			return ( ( m_collisionFlags & ( CollisionFlags.CF_STATIC_OBJECT
					| CollisionFlags.CF_KINEMATIC_OBJECT
					| CollisionFlags.CF_NO_CONTACT_RESPONSE ) ) == 0 );
		}

		public void getAnisotropicFriction( out btVector3 result )
		{
			result = m_anisotropicFriction;
		}

		public void setAnisotropicFriction( ref btVector3 anisotropicFriction
							, AnisotropicFrictionFlags frictionMode = AnisotropicFrictionFlags.CF_ANISOTROPIC_FRICTION )
		{
			m_anisotropicFriction = anisotropicFriction;
			bool isUnity = ( anisotropicFriction.x != 1 ) || ( anisotropicFriction.y != 1 ) || ( anisotropicFriction.z != 1 );
			m_hasAnisotropicFriction = isUnity ? frictionMode : AnisotropicFrictionFlags.CF_ANISOTROPIC_FRICTION_DISABLED;
		}
		public bool hasAnisotropicFriction( AnisotropicFrictionFlags frictionMode = AnisotropicFrictionFlags.CF_ANISOTROPIC_FRICTION )
		{
			return ( m_hasAnisotropicFriction & frictionMode ) != 0;
		}

		///the constraint solver can discard solving contacts, if the distance is above this threshold. 0 by default.
		///Note that using contacts with positive distance can improve stability. It increases, however, the chance of colliding with degerate contacts, such as 'interior' triangle edges
		public void setContactProcessingThreshold( double contactProcessingThreshold )
		{
			m_contactProcessingThreshold = contactProcessingThreshold;
		}
		public double getContactProcessingThreshold()
		{
			return m_contactProcessingThreshold;
		}

		public bool isStaticObject()
		{
			return ( m_collisionFlags & CollisionFlags.CF_STATIC_OBJECT ) != 0;
		}

		public bool isKinematicObject()
		{
			return ( m_collisionFlags & CollisionFlags.CF_KINEMATIC_OBJECT ) != 0;
		}

		public bool isStaticOrKinematicObject()
		{
			return ( m_collisionFlags & ( CollisionFlags.CF_KINEMATIC_OBJECT | CollisionFlags.CF_STATIC_OBJECT ) ) != 0;
		}

		public bool hasContactResponse()
		{
			return ( m_collisionFlags & CollisionFlags.CF_NO_CONTACT_RESPONSE ) == 0;
		}


		public virtual void setCollisionShape( btCollisionShape collisionShape )
		{
			m_updateRevision++;
			m_collisionShape = collisionShape;
			m_rootCollisionShape = collisionShape;
		}

		public virtual btCollisionShape getCollisionShape()
		{
			return m_collisionShape;
		}

		public void setIgnoreCollisionCheck( btCollisionObject co, bool ignoreCollisionCheck )
		{
			if( ignoreCollisionCheck )
			{
				//We don't check for duplicates. Is it ok to leave that up to the user of this API?
				//int index = m_objectsWithoutCollisionCheck.findLinearSearch(co);
				//if (index == m_objectsWithoutCollisionCheck.Count)
				//{
				m_objectsWithoutCollisionCheck.Add( co );
				//}
			}
			else
			{
				m_objectsWithoutCollisionCheck.Remove( co );
			}
			m_checkCollideWith = m_objectsWithoutCollisionCheck.Count > 0;
		}

		public virtual bool checkCollideWithOverride( btCollisionObject co )
		{
			int index = m_objectsWithoutCollisionCheck.IndexOf( co );
			if( index < m_objectsWithoutCollisionCheck.Count )
			{
				return false;
			}
			return true;
		}




		///Avoid using this internal API call, the extension pointer is used by some Bullet extensions. 
		///If you need to store your own user pointer, use 'setUserPointer/getUserPointer' instead.
		public object internalGetExtensionPointer()
		{
			return m_extensionPointer;
		}
		///Avoid using this internal API call, the extension pointer is used by some Bullet extensions
		///If you need to store your own user pointer, use 'setUserPointer/getUserPointer' instead.
		public void internalSetExtensionPointer( object pointer )
		{
			m_extensionPointer = pointer;
		}

		public ActivationState getActivationState() { return m_activationState1; }

		//public void setActivationState( ActivationState newState );

		internal void setDeactivationTime( double time )
		{
			m_deactivationTime = time;
		}
		double getDeactivationTime()
		{
			return m_deactivationTime;
		}

		//void forceActivationState( ActivationState newState );

		//void activate( bool forceActivation = false );

		public bool isActive()
		{
			return ( ( getActivationState() != ActivationState.ISLAND_SLEEPING )
				&& ( getActivationState() != ActivationState.DISABLE_SIMULATION ) );
		}

		void setRestitution( double rest )
		{
			m_updateRevision++;
			m_restitution = rest;
		}
		public double getRestitution()
		{
			return m_restitution;
		}
		public void setFriction( double frict )
		{
			m_updateRevision++;
			m_friction = frict;
		}
		public double getFriction()
		{
			return m_friction;
		}

		public void setRollingFriction( double frict )
		{
			m_updateRevision++;
			m_rollingFriction = frict;
		}
		public double getRollingFriction()
		{
			return m_rollingFriction;
		}


		///reserved for Bullet internal usage
		public CollisionObjectTypes getInternalType()
		{
			return m_internalType;
		}

		public btITransform getWorldTransform(  )
		{
			return m_worldTransform;
		}

		public void setWorldTransform( ref btTransform worldTrans )
		{
			m_updateRevision++;
			m_worldTransform = worldTrans;
		}

		public void setWorldTransform( btITransform worldTrans )
		{
			m_updateRevision++;
			m_worldTransform = worldTrans.T;
		}


		public btBroadphaseProxy getBroadphaseHandle()
		{
			return m_broadphaseHandle;
		}

		public void setBroadphaseHandle( btBroadphaseProxy handle )
		{
			m_broadphaseHandle = handle;
		}


		public void getInterpolationWorldTransform( out btTransform result )
		{
			result = m_interpolationWorldTransform;
		}

		public btITransform getInterpolationWorldTransform( )
		{
			return m_interpolationWorldTransform;
		}

		public void setInterpolationWorldTransform( ref btTransform trans )
		{
			m_updateRevision++;
			m_interpolationWorldTransform = trans;
		}

		public void setInterpolationLinearVelocity( ref btVector3 linvel )
		{
			m_updateRevision++;
			m_interpolationLinearVelocity = linvel;
		}

		public void setInterpolationAngularVelocity( ref btVector3 angvel )
		{
			m_updateRevision++;
			m_interpolationAngularVelocity = angvel;
		}

		public void getInterpolationLinearVelocity( out btVector3 result )
		{
			result = m_interpolationLinearVelocity;
		}

		public void getInterpolationAngularVelocity( out btVector3 result )
		{
			result = m_interpolationAngularVelocity;
		}
		public btIVector3 getInterpolationLinearVelocity( )
		{
			return m_interpolationLinearVelocity;
		}

		public btIVector3 getInterpolationAngularVelocity( )
		{
			return m_interpolationAngularVelocity;
		}

		public int getIslandTag()
		{
			return m_islandTag1;
		}

		public void setIslandTag( int tag )
		{
			m_islandTag1 = tag;
		}
		/*
		public int getCompanionId()
		{
			return m_companionId;
		}

		public void setCompanionId( int id )
		{
			m_companionId = id;
		}
		*/
		public btSolverBody getCompanionBody()
		{
			return m_companionBody;
		}

		public void setCompanionBody( btSolverBody id )
		{
			m_companionBody = id;
		}

		public double getHitFraction()
		{
			return m_hitFraction;
		}

		public void setHitFraction( double hitFraction )
		{
			m_hitFraction = hitFraction;
		}


		public CollisionFlags getCollisionFlags()
		{
			return m_collisionFlags;
		}

		public void setCollisionFlags( CollisionFlags flags )
		{
			m_collisionFlags = flags;
		}

		///Swept sphere radius (0.0 by default), see btConvexConvexAlgorithm::
		public double getCcdSweptSphereRadius()
		{
			return m_ccdSweptSphereRadius;
		}

		///Swept sphere radius (0.0 by default), see btConvexConvexAlgorithm::
		public void setCcdSweptSphereRadius( double radius )
		{
			m_ccdSweptSphereRadius = radius;
		}

		public double getCcdMotionThreshold()
		{
			return m_ccdMotionThreshold;
		}

		public double getCcdSquareMotionThreshold()
		{
			return m_ccdMotionThreshold * m_ccdMotionThreshold;
		}



		/// Don't do continuous collision detection if the motion (in one step) is less then m_ccdMotionThreshold
		public void setCcdMotionThreshold( double ccdMotionThreshold )
		{
			m_ccdMotionThreshold = ccdMotionThreshold;
		}

		///users can point to their objects, userPointer is not used by Bullet
		public object getUserPointer()
		{
			return m_userObjectPointer;
		}

		public int getUserIndex()
		{
			return m_userIndex;
		}
		///users can point to their objects, userPointer is not used by Bullet
		public void setUserPointer( object userPointer )
		{
			m_userObjectPointer = userPointer;
		}

		///users can point to their objects, userPointer is not used by Bullet
		public void setUserIndex( int index )
		{
			m_userIndex = index;
		}

		public int getUpdateRevisionInternal()
		{
			return m_updateRevision;
		}


		public bool checkCollideWith( btCollisionObject co )
		{
			if( m_checkCollideWith )
				return checkCollideWithOverride( co );

			return true;
		}

		/*
		virtual int calculateSerializeBufferSize();

		///fills the dataBuffer and returns the struct name (and 0 on failure)
		virtual stringchar serialize( void dataBuffer, class btSerializer serializer);

		virtual void serializeSingleObject(class btSerializer serializer);
		*/
	};

#if SERIALIZE_DONE
	///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
	struct btCollisionObjectDoubleData
	{
		object m_broadphaseHandle;
		object m_collisionShape;
		btCollisionShapeData m_rootCollisionShape;
		char m_name;

		btTransformDoubleData m_worldTransform;
		btTransformDoubleData m_interpolationWorldTransform;
		btVector3DoubleData m_interpolationLinearVelocity;
		btVector3DoubleData m_interpolationAngularVelocity;
		btVector3DoubleData m_anisotropicFriction;
		double m_contactProcessingThreshold;
		double m_deactivationTime;
		double m_friction;
		double m_rollingFriction;
		double m_restitution;
		double m_hitFraction;
		double m_ccdSweptSphereRadius;
		double m_ccdMotionThreshold;

		btCollisionObject.AnisotropicFrictionFlags m_hasAnisotropicFriction;
		int m_collisionFlags;
		int m_islandTag1;
		int m_companionId;
		int m_activationState1;
		int m_internalType;
		int m_checkCollideWith;

		char m_padding[4];
	};

	///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
	struct btCollisionObjectFloatData
	{
		void m_broadphaseHandle;
		void m_collisionShape;
		btCollisionShapeData m_rootCollisionShape;
		char m_name;

		btTransformFloatData m_worldTransform;
		btTransformFloatData m_interpolationWorldTransform;
		btVector3FloatData m_interpolationLinearVelocity;
		btVector3FloatData m_interpolationAngularVelocity;
		btVector3FloatData m_anisotropicFriction;
		float m_contactProcessingThreshold;
		float m_deactivationTime;
		float m_friction;
		float m_rollingFriction;

		float m_restitution;
		float m_hitFraction;
		float m_ccdSweptSphereRadius;
		float m_ccdMotionThreshold;

		int m_hasAnisotropicFriction;
		int m_collisionFlags;
		int m_islandTag1;
		int m_companionId;
		int m_activationState1;
		int m_internalType;
		int m_checkCollideWith;
		char m_padding[4];
	};



	public int btCollisionObject::calculateSerializeBufferSize()
	{
		return sizeof( btCollisionObjectData );
	}
#endif 
}

