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

using Bullet.LinearMath;

namespace Bullet.Collision.Dispatch
{

	public partial class btCollisionObject
	{
		public btCollisionObject()
		{
			m_anisotropicFriction = new btVector3( 1, 1, 1 );
			m_hasAnisotropicFriction = AnisotropicFrictionFlags.CF_ANISOTROPIC_FRICTION_DISABLED;
			m_contactProcessingThreshold = btScalar.BT_LARGE_FLOAT;
			//m_broadphaseHandle= null ;
			m_collisionShape = null;
			m_extensionPointer = null;
			m_rootCollisionShape = null;
			m_collisionFlags = CollisionFlags.CF_STATIC_OBJECT;
			m_islandTag1 = -1;
			//m_companionId = -1;
			m_companionBody = null;
			m_activationState1 = ActivationState.ACTIVE_TAG;
			m_deactivationTime = (double)0;
			m_friction = 0.5;
			m_restitution = 0;
			m_rollingFriction = 0.0f;
			m_internalType = CollisionObjectTypes.CO_COLLISION_OBJECT;
			m_userObjectPointer = 0;
			m_userIndex = -1;
			m_hitFraction = 1;
			m_ccdSweptSphereRadius = 0;
			m_ccdMotionThreshold = 0;
			m_checkCollideWith = false;
			m_updateRevision = 0;
			m_worldTransform = btTransform.Identity;
		}

		public void setActivationState( ActivationState newState )
		{
			if( ( m_activationState1 != ActivationState.DISABLE_DEACTIVATION )
				&& ( m_activationState1 != ActivationState.DISABLE_SIMULATION ) )
				m_activationState1 = newState;
		}

		public void forceActivationState( ActivationState newState )
		{
			m_activationState1 = newState;
		}

		public void activate( bool forceActivation = false )
		{
			if( forceActivation || ( m_collisionFlags & ( CollisionFlags.CF_STATIC_OBJECT | CollisionFlags.CF_KINEMATIC_OBJECT ) ) == 0 )
			{
				setActivationState( ActivationState.ACTIVE_TAG );
				m_deactivationTime = (double)( 0 );
			}
		}

		/*
	stringchar* serialize( object dataBuffer, btSerializer* serializer )
	{

		btCollisionObjectData* dataOut = (btCollisionObjectData*)dataBuffer;

		m_worldTransform.serialize(dataOut.m_worldTransform);
		m_interpolationWorldTransform.serialize(dataOut.m_interpolationWorldTransform);
		m_interpolationLinearVelocity.serialize(dataOut.m_interpolationLinearVelocity);
		m_interpolationAngularVelocity.serialize(dataOut.m_interpolationAngularVelocity);
		m_anisotropicFriction.serialize(dataOut.m_anisotropicFriction);
		dataOut.m_hasAnisotropicFriction = m_hasAnisotropicFriction;
		dataOut.m_contactProcessingThreshold = m_contactProcessingThreshold;
		dataOut.m_broadphaseHandle = 0;
		dataOut.m_collisionShape = serializer.getUniquePointer( m_collisionShape);
		dataOut.m_rootCollisionShape = 0;//@todo
		dataOut.m_collisionFlags = m_collisionFlags;
		dataOut.m_islandTag1 = m_islandTag1;
		dataOut.m_companionId = m_companionId;
		dataOut.m_activationState1 = m_activationState1;
		dataOut.m_deactivationTime = m_deactivationTime;
		dataOut.m_friction = m_friction;
		dataOut.m_rollingFriction = m_rollingFriction;
		dataOut.m_restitution = m_restitution;
		dataOut.m_internalType = m_internalType;

		char* name = (char*)serializer.findNameForPointer( this );
	dataOut.m_name = (char*)serializer.getUniquePointer( name);
		if (dataOut.m_name)
		{
			serializer.serializeName( name);
		}
		dataOut.m_hitFraction = m_hitFraction;
		dataOut.m_ccdSweptSphereRadius = m_ccdSweptSphereRadius;
		dataOut.m_ccdMotionThreshold = m_ccdMotionThreshold;
		dataOut.m_checkCollideWith = m_checkCollideWith;

		return btCollisionObjectDataName;
	}

		void serializeSingleObject(class btSerializer* serializer)
		{
    int len = calculateSerializeBufferSize();
		btChunk* chunk = serializer.allocate( len, 1 );
		stringchar* structType = serialize( chunk.m_oldPtr, serializer );
		serializer.finalizeChunk( chunk, structType, BT_COLLISIONOBJECT_CODE,(object)this);
}

		*/

	}
}
