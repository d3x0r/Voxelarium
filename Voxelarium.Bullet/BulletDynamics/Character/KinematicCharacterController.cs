/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include <stdio.h>
#include "LinearMath/btIDebugDraw.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"
#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "LinearMath/btDefaultMotionState.h"
#include "btKinematicCharacterController.h"


// static helper method
static btVector3
getNormalizedVector(ref btVector3 v)
{
	btVector3 n(0, 0, 0);

	if (v.length() > SIMD_EPSILON) {
		n = v.normalized();
	}
	return n;
}


///@todo Interact with dynamic objects,
///Ride kinematicly animated platforms properly
///More realistic (or maybe just a config option) falling
/// . Should integrate falling velocity manually and use that in stepDown()
///Support jumping
///Support ducking
class btKinematicClosestNotMeRayResultCallback : btCollisionWorld::ClosestRayResultCallback
{
public:
	btKinematicClosestNotMeRayResultCallback (btCollisionObject me) : btCollisionWorld::ClosestRayResultCallback(btVector3(0.0, 0.0, 0.0), btVector3(0.0, 0.0, 0.0))
	{
		m_me = me;
	}

	virtual double addSingleResult(btCollisionWorld::LocalRayResult& rayResult,bool normalInWorldSpace)
	{
		if (rayResult.m_collisionObject == m_me)
			return 1.0;

		return ClosestRayResultCallback::addSingleResult (rayResult, normalInWorldSpace);
	}
protected:
	btCollisionObject m_me;
};

class btKinematicClosestNotMeConvexResultCallback : btCollisionWorld::ClosestConvexResultCallback
{
public:
	btKinematicClosestNotMeConvexResultCallback (btCollisionObject me, ref btVector3 up, double minSlopeDot)
	: btCollisionWorld::ClosestConvexResultCallback(btVector3(0.0, 0.0, 0.0), btVector3(0.0, 0.0, 0.0))
	, m_me(me)
	, m_up(up)
	, m_minSlopeDot(minSlopeDot)
	{
	}

	virtual double addSingleResult(btCollisionWorld::LocalConvexResult& convexResult,bool normalInWorldSpace)
	{
		if (convexResult.m_hitCollisionObject == m_me)
			return (double)(1.0);

		if (!convexResult.m_hitCollisionObject.hasContactResponse())
			return (double)(1.0);

		btVector3 hitNormalWorld;
		if (normalInWorldSpace)
		{
			hitNormalWorld = convexResult.m_hitNormalLocal;
		} else
		{
			///need to transform normal into worldspace
			hitNormalWorld = convexResult.m_hitCollisionObject.getWorldTransform().getBasis()*convexResult.m_hitNormalLocal;
		}

		double dotUp = m_up.dot(hitNormalWorld);
		if (dotUp < m_minSlopeDot) {
			return (double)(1.0);
		}

		return ClosestConvexResultCallback::addSingleResult (convexResult, normalInWorldSpace);
	}
protected:
	btCollisionObject m_me;
	btVector3 m_up;
	double m_minSlopeDot;
};

/*
 * Returns the reflection direction of a ray going 'direction' hitting a surface with normal 'normal'
 *
 * from: http://www-cs-students.stanford.edu/~adityagp/final/node3.html
 */
btVector3 btKinematicCharacterController::computeReflectionDirection (ref btVector3 direction, ref btVector3 normal)
{
	return direction - ((double)(2.0) * direction.dot(normal)) * normal;
}

/*
 * Returns the portion of 'direction' that is parallel to 'normal'
 */
btVector3 btKinematicCharacterController::parallelComponent (ref btVector3 direction, ref btVector3 normal)
{
	double magnitude = direction.dot(normal);
	return normal * magnitude;
}

/*
 * Returns the portion of 'direction' that is perpindicular to 'normal'
 */
btVector3 btKinematicCharacterController::perpindicularComponent (ref btVector3 direction, ref btVector3 normal)
{
	return direction - parallelComponent(direction, normal);
}

btKinematicCharacterController::btKinematicCharacterController (btPairCachingGhostObject* ghostObject,btConvexShape* convexShape,double stepHeight, int upAxis)
{
	m_upAxis = upAxis;
	m_addedMargin = 0.02;
	m_walkDirection.setValue(0,0,0);
	m_useGhostObjectSweepTest = true;
	m_ghostObject = ghostObject;
	m_stepHeight = stepHeight;
	m_turnAngle = (double)(0.0);
	m_convexShape=convexShape;	
	m_useWalkDirection = true;	// use walk direction by default, legacy behavior
	m_velocityTimeInterval = 0.0;
	m_verticalVelocity = 0.0;
	m_verticalOffset = 0.0;
	m_gravity = 9.8 * 3 ; // 3G acceleration.
	m_fallSpeed = 55.0; // Terminal velocity of a sky diver in m/s.
	m_jumpSpeed = 10.0; // ?
	m_wasOnGround = false;
	m_wasJumping = false;
	m_interpolateUp = true;
	setMaxSlope(btRadians(45.0));
	m_currentStepOffset = 0;
	full_drop = false;
	bounce_fix = false;
}

btKinematicCharacterController::~btKinematicCharacterController ()
{
}

btPairCachingGhostObject* btKinematicCharacterController::getGhostObject()
{
	return m_ghostObject;
}

bool btKinematicCharacterController::recoverFromPenetration ( btCollisionWorld* collisionWorld)
{
	// Here we must refresh the overlapping paircache as the penetrating movement itself or the
	// previous recovery iteration might have used setWorldTransform and pushed us into an object
	// that is not in the previous cache contents from the last timestep, as will happen if we
	// are pushed into a new AABB overlap. Unhandled this means the next convex sweep gets stuck.
	//
	// Do this by calling the broadphase's setAabb with the moved AABB, this will update the broadphase
	// paircache and the ghostobject's internal paircache at the same time.    /BW

	btVector3 minAabb, maxAabb;
	m_convexShape.getAabb(m_ghostObject.getWorldTransform(), minAabb,maxAabb);
	collisionWorld.getBroadphase().setAabb(m_ghostObject.getBroadphaseHandle(), 
						 minAabb, 
						 maxAabb, 
						 collisionWorld.getDispatcher());
						 
	bool penetration = false;

	collisionWorld.getDispatcher().dispatchAllCollisionPairs(m_ghostObject.getOverlappingPairCache(), collisionWorld.getDispatchInfo(), collisionWorld.getDispatcher());

	m_currentPosition = m_ghostObject.getWorldTransform().getOrigin();
	
	double maxPen = (double)(0.0);
	for (int i = 0; i < m_ghostObject.getOverlappingPairCache().getNumOverlappingPairs(); i++)
	{
		m_manifoldArray.resize(0);

		btBroadphasePair* collisionPair = &m_ghostObject.getOverlappingPairCache().getOverlappingPairArray()[i];

		btCollisionObject obj0 = static_cast<btCollisionObject>(collisionPair.m_pProxy0.m_clientObject);
                btCollisionObject obj1 = static_cast<btCollisionObject>(collisionPair.m_pProxy1.m_clientObject);

		if ((obj0 && !obj0.hasContactResponse()) || (obj1 && !obj1.hasContactResponse()))
			continue;
		
		if (collisionPair.m_algorithm)
			collisionPair.m_algorithm.getAllContactManifolds(m_manifoldArray);

		
		for (int j=0;j<m_manifoldArray.Count;j++)
		{
			btPersistentManifold* manifold = m_manifoldArray[j];
			double directionSign = manifold.getBody0() == m_ghostObject ? (double)(-1.0) : (double)(1.0);
			for (int p=0;p<manifold.getNumContacts();p++)
			{
				btManifoldPointpt = manifold.getContactPoint(p);

				double dist = pt.getDistance();

				if (dist < 0.0)
				{
					if (dist < maxPen)
					{
						maxPen = dist;
						m_touchingNormal = pt.m_normalWorldOnB * directionSign;//??

					}
					m_currentPosition += pt.m_normalWorldOnB * directionSign * dist * (double)(0.2);
					penetration = true;
				} else {
					//Console.WriteLine("touching %f\n", dist);
				}
			}
			
			//manifold.clearManifold();
		}
	}
	btTransform newTrans = m_ghostObject.getWorldTransform();
	newTrans.setOrigin(m_currentPosition);
	m_ghostObject.setWorldTransform(newTrans);
//	Console.WriteLine("m_touchingNormal = %f,%f,%f\n",m_touchingNormal,m_touchingNormal[1],m_touchingNormal[2]);
	return penetration;
}

void btKinematicCharacterController::stepUp ( btCollisionWorld* world)
{
	// phase 1: up
	btTransform start, end;
	m_targetPosition = m_currentPosition + getUpAxisDirections()[m_upAxis] * (m_stepHeight + (m_verticalOffset > 0?m_verticalOffset:0));

	start.setIdentity ();
	end.setIdentity ();

	/* FIXME: Handle penetration properly */
	start.setOrigin (m_currentPosition + getUpAxisDirections()[m_upAxis] * (m_convexShape.getMargin() + m_addedMargin));
	end.setOrigin (m_targetPosition);

	btKinematicClosestNotMeConvexResultCallback callback (m_ghostObject, -getUpAxisDirections()[m_upAxis], (double)(0.7071));
	callback.m_collisionFilterGroup = getGhostObject().getBroadphaseHandle().m_collisionFilterGroup;
	callback.m_collisionFilterMask = getGhostObject().getBroadphaseHandle().m_collisionFilterMask;
	
	if (m_useGhostObjectSweepTest)
	{
		m_ghostObject.convexSweepTest (m_convexShape, start, end, callback, world.getDispatchInfo().m_allowedCcdPenetration);
	}
	else
	{
		world.convexSweepTest (m_convexShape, start, end, callback);
	}
	
	if (callback.hasHit())
	{
		// Only modify the position if the hit was a slope and not a wall or ceiling.
		if(callback.m_hitNormalWorld.dot(getUpAxisDirections()[m_upAxis]) > 0.0)
		{
			// we moved up only a fraction of the step height
			m_currentStepOffset = m_stepHeight * callback.m_closestHitFraction;
			if (m_interpolateUp == true)
				m_currentPosition.setInterpolate3 (m_currentPosition, m_targetPosition, callback.m_closestHitFraction);
			else
				m_currentPosition = m_targetPosition;
		}
		m_verticalVelocity = 0.0;
		m_verticalOffset = 0.0;
	} else {
		m_currentStepOffset = m_stepHeight;
		m_currentPosition = m_targetPosition;
	}
}

void btKinematicCharacterController::updateTargetPositionBasedOnCollision (ref btVector3 hitNormal, double tangentMag, double normalMag)
{
	btVector3 movementDirection = m_targetPosition - m_currentPosition;
	double movementLength = movementDirection.length();
	if (movementLength>SIMD_EPSILON)
	{
		movementDirection.normalize();

		btVector3 reflectDir = computeReflectionDirection (movementDirection, hitNormal);
		reflectDir.normalize();

		btVector3 parallelDir, perpindicularDir;

		parallelDir = parallelComponent (reflectDir, hitNormal);
		perpindicularDir = perpindicularComponent (reflectDir, hitNormal);

		m_targetPosition = m_currentPosition;
		if (0)//tangentMag != 0.0)
		{
			btVector3 parComponent = parallelDir * double (tangentMag*movementLength);
//			Console.WriteLine("parComponent=%f,%f,%f\n",parComponent[0],parComponent[1],parComponent[2]);
			m_targetPosition +=  parComponent;
		}

		if (normalMag != 0.0)
		{
			btVector3 perpComponent = perpindicularDir * double (normalMag*movementLength);
//			Console.WriteLine("perpComponent=%f,%f,%f\n",perpComponent[0],perpComponent[1],perpComponent[2]);
			m_targetPosition += perpComponent;
		}
	} else
	{
//		Console.WriteLine("movementLength don't normalize a zero vector\n");
	}
}

void btKinematicCharacterController::stepForwardAndStrafe ( btCollisionWorld* collisionWorld, ref btVector3 walkMove)
{
	// Console.WriteLine("m_normalizedDirection=%f,%f,%f\n",
	// 	m_normalizedDirection[0],m_normalizedDirection[1],m_normalizedDirection[2]);
	// phase 2: forward and strafe
	btTransform start, end;
	m_targetPosition = m_currentPosition + walkMove;

	start.setIdentity ();
	end.setIdentity ();
	
	double fraction = 1.0;
	double distance2 = (m_currentPosition-m_targetPosition).length2();
//	Console.WriteLine("distance2=%f\n",distance2);

	if (m_touchingContact)
	{
		if (m_normalizedDirection.dot(m_touchingNormal) > (double)(0.0))
		{
			//interferes with step movement
			//updateTargetPositionBasedOnCollision (m_touchingNormal);
		}
	}

	int maxIter = 10;

	while (fraction > (double)(0.01) && maxIter-- > 0)
	{
		start.setOrigin (m_currentPosition);
		end.setOrigin (m_targetPosition);
		btVector3 sweepDirNegative(m_currentPosition - m_targetPosition);

		btKinematicClosestNotMeConvexResultCallback callback (m_ghostObject, sweepDirNegative, (double)(0.0));
		callback.m_collisionFilterGroup = getGhostObject().getBroadphaseHandle().m_collisionFilterGroup;
		callback.m_collisionFilterMask = getGhostObject().getBroadphaseHandle().m_collisionFilterMask;


		double margin = m_convexShape.getMargin();
		m_convexShape.setMargin(margin + m_addedMargin);


		if (m_useGhostObjectSweepTest)
		{
			m_ghostObject.convexSweepTest (m_convexShape, start, end, callback, collisionWorld.getDispatchInfo().m_allowedCcdPenetration);
		} else
		{
			collisionWorld.convexSweepTest (m_convexShape, start, end, callback, collisionWorld.getDispatchInfo().m_allowedCcdPenetration);
		}
		
		m_convexShape.setMargin(margin);

		
		fraction -= callback.m_closestHitFraction;

		if (callback.hasHit())
		{	
			// we moved only a fraction
			//double hitDistance;
			//hitDistance = (callback.m_hitPointWorld - m_currentPosition).length();

//			m_currentPosition.setInterpolate3 (m_currentPosition, m_targetPosition, callback.m_closestHitFraction);

			updateTargetPositionBasedOnCollision (callback.m_hitNormalWorld);
			btVector3 currentDir = m_targetPosition - m_currentPosition;
			distance2 = currentDir.length2();
			if (distance2 > SIMD_EPSILON)
			{
				currentDir.normalize();
				/* See Quake2: "If velocity is against original velocity, stop ead to avoid tiny oscilations in sloping corners." */
				if (currentDir.dot(m_normalizedDirection) <= (double)(0.0))
				{
					break;
				}
			} else
			{
//				Console.WriteLine("currentDir: don't normalize a zero vector\n");
				break;
			}

		} else {
			// we moved whole way
			m_currentPosition = m_targetPosition;
		}

	//	if (callback.m_closestHitFraction == 0)
	//		break;

	}
}

void btKinematicCharacterController::stepDown ( btCollisionWorld* collisionWorld, double dt)
{
	btTransform start, end, end_double;
	bool runonce = false;

	// phase 3: down
	/*double additionalDownStep = (m_wasOnGround && !onGround()) ? m_stepHeight : 0.0;
	btVector3 step_drop = getUpAxisDirections()[m_upAxis] * (m_currentStepOffset + additionalDownStep);
	double downVelocity = (additionalDownStep == 0.0 && m_verticalVelocity<0.0?-m_verticalVelocity:0.0) * dt;
	btVector3 gravity_drop = getUpAxisDirections()[m_upAxis] * downVelocity; 
	m_targetPosition -= (step_drop + gravity_drop);*/

	btVector3 orig_position = m_targetPosition;
	
	double downVelocity = (m_verticalVelocity<0?-m_verticalVelocity:0) * dt;

	if(downVelocity > 0.0 && downVelocity > m_fallSpeed
		&& (m_wasOnGround || !m_wasJumping))
		downVelocity = m_fallSpeed;

	btVector3 step_drop = getUpAxisDirections()[m_upAxis] * (m_currentStepOffset + downVelocity);
	m_targetPosition -= step_drop;

	btKinematicClosestNotMeConvexResultCallback callback (m_ghostObject, getUpAxisDirections()[m_upAxis], m_maxSlopeCosine);
        callback.m_collisionFilterGroup = getGhostObject().getBroadphaseHandle().m_collisionFilterGroup;
        callback.m_collisionFilterMask = getGhostObject().getBroadphaseHandle().m_collisionFilterMask;

        btKinematicClosestNotMeConvexResultCallback callback2 (m_ghostObject, getUpAxisDirections()[m_upAxis], m_maxSlopeCosine);
        callback2.m_collisionFilterGroup = getGhostObject().getBroadphaseHandle().m_collisionFilterGroup;
        callback2.m_collisionFilterMask = getGhostObject().getBroadphaseHandle().m_collisionFilterMask;

	while (1)
	{
		start.setIdentity ();
		end.setIdentity ();

		end_double.setIdentity ();

		start.setOrigin (m_currentPosition);
		end.setOrigin (m_targetPosition);

		//set double test for 2x the step drop, to check for a large drop vs small drop
		end_double.setOrigin (m_targetPosition - step_drop);

		if (m_useGhostObjectSweepTest)
		{
			m_ghostObject.convexSweepTest (m_convexShape, start, end, callback, collisionWorld.getDispatchInfo().m_allowedCcdPenetration);

			if (!callback.hasHit())
			{
				//test a double fall height, to see if the character should interpolate it's fall (full) or not (partial)
				m_ghostObject.convexSweepTest (m_convexShape, start, end_double, callback2, collisionWorld.getDispatchInfo().m_allowedCcdPenetration);
			}
		} else
		{
			collisionWorld.convexSweepTest (m_convexShape, start, end, callback, collisionWorld.getDispatchInfo().m_allowedCcdPenetration);

			if (!callback.hasHit())
					{
							//test a double fall height, to see if the character should interpolate it's fall (large) or not (small)
							collisionWorld.convexSweepTest (m_convexShape, start, end_double, callback2, collisionWorld.getDispatchInfo().m_allowedCcdPenetration);
					}
		}
	
		double downVelocity2 = (m_verticalVelocity<0?-m_verticalVelocity:0) * dt;
		bool has_hit = false;
		if (bounce_fix == true)
			has_hit = callback.hasHit() || callback2.hasHit();
		else
			has_hit = callback2.hasHit();

		if(downVelocity2 > 0.0 && downVelocity2 < m_stepHeight && has_hit == true && runonce == false
					&& (m_wasOnGround || !m_wasJumping))
		{
			//redo the velocity calculation when falling a small amount, for fast stairs motion
			//for larger falls, use the smoother/slower interpolated movement by not touching the target position

			m_targetPosition = orig_position;
					downVelocity = m_stepHeight;

				btVector3 step_drop = getUpAxisDirections()[m_upAxis] * (m_currentStepOffset + downVelocity);
			m_targetPosition -= step_drop;
			runonce = true;
			continue; //re-run previous tests
		}
		break;
	}

	if (callback.hasHit() || runonce == true)
	{
		// we dropped a fraction of the height . hit floor

		double fraction = (m_currentPosition.y - callback.m_hitPointWorld.y) / 2;

		//Console.WriteLine("hitpoint: %g - pos %g\n", callback.m_hitPointWorld.y, m_currentPosition.y);

		if (bounce_fix == true)
		{
			if (full_drop == true)
                                m_currentPosition.setInterpolate3 (m_currentPosition, m_targetPosition, callback.m_closestHitFraction);
                        else
                                //due to errors in the closestHitFraction variable when used with large polygons, calculate the hit fraction manually
                                m_currentPosition.setInterpolate3 (m_currentPosition, m_targetPosition, fraction);
		}
		else
			m_currentPosition.setInterpolate3 (m_currentPosition, m_targetPosition, callback.m_closestHitFraction);

		full_drop = false;

		m_verticalVelocity = 0.0;
		m_verticalOffset = 0.0;
		m_wasJumping = false;
	} else {
		// we dropped the full height
		
		full_drop = true;

		if (bounce_fix == true)
		{
			downVelocity = (m_verticalVelocity<0?-m_verticalVelocity:0) * dt;
			if (downVelocity > m_fallSpeed && (m_wasOnGround || !m_wasJumping))
			{
				m_targetPosition += step_drop; //undo previous target change
				downVelocity = m_fallSpeed;
				step_drop = getUpAxisDirections()[m_upAxis] * (m_currentStepOffset + downVelocity);
				m_targetPosition -= step_drop;
			}
		}
		//Console.WriteLine("full drop - %g, %g\n", m_currentPosition.y, m_targetPosition.y);

		m_currentPosition = m_targetPosition;
	}
}



void btKinematicCharacterController::setWalkDirection
(
ref btVector3 walkDirection
)
{
	m_useWalkDirection = true;
	m_walkDirection = walkDirection;
	m_normalizedDirection = getNormalizedVector(m_walkDirection);
}



void btKinematicCharacterController::setVelocityForTimeInterval
(
ref btVector3 velocity,
double timeInterval
)
{
//	Console.WriteLine("setVelocity!\n");
//	Console.WriteLine("  interval: %f\n", timeInterval);
//	Console.WriteLine("  velocity: (%f, %f, %f)\n",
//		 velocity.x, velocity.y, velocity.z);

	m_useWalkDirection = false;
	m_walkDirection = velocity;
	m_normalizedDirection = getNormalizedVector(m_walkDirection);
	m_velocityTimeInterval += timeInterval;
}

void btKinematicCharacterController::reset ( btCollisionWorld* collisionWorld )
{
        m_verticalVelocity = 0.0;
        m_verticalOffset = 0.0;
        m_wasOnGround = false;
        m_wasJumping = false;
        m_walkDirection.setValue(0,0,0);
        m_velocityTimeInterval = 0.0;

        //clear pair cache
        btHashedOverlappingPairCache *cache = m_ghostObject.getOverlappingPairCache();
        while (cache.getOverlappingPairArray().Count > 0)
        {
                cache.removeOverlappingPair(cache.getOverlappingPairArray()[0].m_pProxy0, cache.getOverlappingPairArray()[0].m_pProxy1, collisionWorld.getDispatcher());
        }
}

void btKinematicCharacterController::warp (ref btVector3 origin)
{
	btTransform xform;
	xform.setIdentity();
	xform.setOrigin (origin);
	m_ghostObject.setWorldTransform (xform);
}


void btKinematicCharacterController::preStep (  btCollisionWorld* collisionWorld)
{
	
	int numPenetrationLoops = 0;
	m_touchingContact = false;
	while (recoverFromPenetration (collisionWorld))
	{
		numPenetrationLoops++;
		m_touchingContact = true;
		if (numPenetrationLoops > 4)
		{
			//Console.WriteLine("character could not recover from penetration = %d\n", numPenetrationLoops);
			break;
		}
	}

	m_currentPosition = m_ghostObject.getWorldTransform().getOrigin();
	m_targetPosition = m_currentPosition;
//	Console.WriteLine("m_targetPosition=%f,%f,%f\n",m_targetPosition[0],m_targetPosition[1],m_targetPosition[2]);

	
}

#include <stdio.h>

void btKinematicCharacterController::playerStep (  btCollisionWorld* collisionWorld, double dt)
{
//	Console.WriteLine("playerStep(): ");
//	Console.WriteLine("  dt = %f", dt);

	// quick check...
	if (!m_useWalkDirection & (m_velocityTimeInterval <= 0.0 || m_walkDirection.fuzzyZero())) {
//		Console.WriteLine("\n");
		return;		// no motion
	}

	m_wasOnGround = onGround();

	// Update fall velocity.
	m_verticalVelocity -= m_gravity * dt;
	if(m_verticalVelocity > 0.0 && m_verticalVelocity > m_jumpSpeed)
	{
		m_verticalVelocity = m_jumpSpeed;
	}
	if(m_verticalVelocity < 0.0 && btFabs(m_verticalVelocity) > btFabs(m_fallSpeed))
	{
		m_verticalVelocity = -btFabs(m_fallSpeed);
	}
	m_verticalOffset = m_verticalVelocity * dt;


	btTransform xform;
	xform = m_ghostObject.getWorldTransform ();

//	Console.WriteLine("walkDirection(%f,%f,%f)\n",walkDirection,walkDirection[1],walkDirection[2]);
//	Console.WriteLine("walkSpeed=%f\n",walkSpeed);

	stepUp (collisionWorld);
	if (m_useWalkDirection) {
		stepForwardAndStrafe (collisionWorld, m_walkDirection);
	} else {
		//Console.WriteLine("  time: %f", m_velocityTimeInterval);
		// still have some time left for moving!
		double dtMoving =
			(dt < m_velocityTimeInterval) ? dt : m_velocityTimeInterval;
		m_velocityTimeInterval -= dt;

		// how far will we move while we are moving?
		btVector3 move = m_walkDirection * dtMoving;

		//Console.WriteLine("  dtMoving: %f", dtMoving);

		// okay, step
		stepForwardAndStrafe(collisionWorld, move);
	}
	stepDown (collisionWorld, dt);

	// Console.WriteLine("\n");

	xform.setOrigin (m_currentPosition);
	m_ghostObject.setWorldTransform (xform);
}

void btKinematicCharacterController::setFallSpeed (double fallSpeed)
{
	m_fallSpeed = fallSpeed;
}

void btKinematicCharacterController::setJumpSpeed (double jumpSpeed)
{
	m_jumpSpeed = jumpSpeed;
}

void btKinematicCharacterController::setMaxJumpHeight (double maxJumpHeight)
{
	m_maxJumpHeight = maxJumpHeight;
}

bool btKinematicCharacterController::canJump ()
{
	return onGround();
}

void btKinematicCharacterController::jump ()
{
	if (!canJump())
		return;

	m_verticalVelocity = m_jumpSpeed;
	m_wasJumping = true;

#if 0
	currently no jumping.
	btTransform xform;
	m_rigidBody.getMotionState().getWorldTransform (xform);
	btVector3 up = xform.getBasis()[1];
	up.normalize ();
	double magnitude = ((double)(1.0)/m_rigidBody.getInvMass()) * (double)(8.0);
	m_rigidBody.applyCentralImpulse (up * magnitude);
#endif
}

void btKinematicCharacterController::setGravity(double gravity)
{
	m_gravity = gravity;
}

double btKinematicCharacterController::getGravity()
{
	return m_gravity;
}

void btKinematicCharacterController::setMaxSlope(double slopeRadians)
{
	m_maxSlopeRadians = slopeRadians;
	m_maxSlopeCosine = btCos(slopeRadians);
}

double btKinematicCharacterController::getMaxSlope()
{
	return m_maxSlopeRadians;
}

bool btKinematicCharacterController::onGround ()
{
	return m_verticalVelocity == 0.0 && m_verticalOffset == 0.0;
}


btVector3* btKinematicCharacterController::getUpAxisDirections()
{
	static btVector3 sUpAxisDirection[3] = { btVector3(1.0f, 0.0f, 0.0f), btVector3(0.0f, 1.0f, 0.0f), btVector3(0.0f, 0.0f, 1.0f) };
	
	return sUpAxisDirection;
}

void btKinematicCharacterController::debugDraw(btIDebugDraw* debugDrawer)
{
}

void btKinematicCharacterController::setUpInterpolate(bool value)
{
	m_interpolateUp = value;
}
