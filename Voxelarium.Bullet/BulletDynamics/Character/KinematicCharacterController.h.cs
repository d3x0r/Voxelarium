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

namespace Bullet.Dynamics.Character
{

	///btKinematicCharacterController is an object that supports a sliding motion in a world.
	///It uses a ghost object and convex sweep test to test for upcoming collisions. This is combined with discrete collision detection to recover from penetrations.
	///Interaction between btKinematicCharacterController and dynamic rigid bodies needs to be explicity implemented by the user.
	internal class btKinematicCharacterController : btCharacterControllerInterface
	{
		protected:

	double m_halfHeight;

		btPairCachingGhostObject* m_ghostObject;
		btConvexShape* m_convexShape;//is also in m_ghostObject, but it needs to be convex, so we store it here to avoid upcast

		double m_verticalVelocity;
		double m_verticalOffset;
		double m_fallSpeed;
		double m_jumpSpeed;
		double m_maxJumpHeight;
		double m_maxSlopeRadians; // Slope angle that is set (used for returning the exact value)
		double m_maxSlopeCosine;  // Cosine equivalent of m_maxSlopeRadians (calculated once when set, for optimization)
		double m_gravity;

		double m_turnAngle;

		double m_stepHeight;

		double m_addedMargin;//@todo: remove this and fix the code

		///this is the desired walk direction, set by the user
		btVector3 m_walkDirection;
		btVector3 m_normalizedDirection;

		//some internal variables
		btVector3 m_currentPosition;
		double m_currentStepOffset;
		btVector3 m_targetPosition;

		///keep track of the contact manifolds
		btManifoldArray m_manifoldArray;

		bool m_touchingContact;
		btVector3 m_touchingNormal;

		bool m_wasOnGround;
		bool m_wasJumping;
		bool m_useGhostObjectSweepTest;
		bool m_useWalkDirection;
		double m_velocityTimeInterval;
		int m_upAxis;

		static btVector3* getUpAxisDirections();
		bool m_interpolateUp;
		bool full_drop;
		bool bounce_fix;

		btVector3 computeReflectionDirection( ref btVector3 direction, ref btVector3 normal );
		btVector3 parallelComponent( ref btVector3 direction, ref btVector3 normal );
		btVector3 perpindicularComponent( ref btVector3 direction, ref btVector3 normal );

		bool recoverFromPenetration( btCollisionWorld* collisionWorld );
		void stepUp( btCollisionWorld* collisionWorld );
		void updateTargetPositionBasedOnCollision( ref btVector3 hit_normal, double tangentMag = btScalar.BT_ZERO, double normalMag = (double)( 1.0 ) );
		void stepForwardAndStrafe( btCollisionWorld* collisionWorld, ref btVector3 walkMove );
		void stepDown( btCollisionWorld* collisionWorld, double dt );
		public:

	

	btKinematicCharacterController( btPairCachingGhostObject* ghostObject, btConvexShape* convexShape, double stepHeight, int upAxis = 1 );
		~btKinematicCharacterController();


		///btActionInterface interface
		virtual void updateAction( btCollisionWorld* collisionWorld, double deltaTime )
		{
			preStep( collisionWorld );
			playerStep( collisionWorld, deltaTime );
		}

		///btActionInterface interface
		void debugDraw( btIDebugDraw* debugDrawer );

		void setUpAxis( int axis )
		{
			if( axis < 0 )
				axis = 0;
			if( axis > 2 )
				axis = 2;
			m_upAxis = axis;
		}

		/// This should probably be called setPositionIncrementPerSimulatorStep.
		/// This is neither a direction nor a velocity, but the amount to
		///	increment the position each simulation iteration, regardless
		///	of dt.
		/// This call will reset any velocity set by setVelocityForTimeInterval().
		virtual void setWalkDirection( ref btVector3 walkDirection );

		/// Caller provides a velocity with which the character should move for
		///	the given time period.  After the time period, velocity is reset
		///	to zero.
		/// This call will reset any walk direction set by setWalkDirection().
		/// Negative time intervals will result in no motion.
		virtual void setVelocityForTimeInterval( ref btVector3 velocity,
					double timeInterval );

		void reset( btCollisionWorld* collisionWorld );
		void warp( ref btVector3 origin );

		void preStep( btCollisionWorld* collisionWorld );
		void playerStep( btCollisionWorld* collisionWorld, double dt );

		void setFallSpeed( double fallSpeed );
		void setJumpSpeed( double jumpSpeed );
		void setMaxJumpHeight( double maxJumpHeight );
		bool canJump();

		void jump();

		void setGravity( double gravity );
		double getGravity();

		/// The max slope determines the maximum angle that the controller can walk up.
		/// The slope angle is measured in radians.
		void setMaxSlope( double slopeRadians );
		double getMaxSlope();

		btPairCachingGhostObject* getGhostObject();
		void setUseGhostSweepTest( bool useGhostObjectSweepTest )
		{
			m_useGhostObjectSweepTest = useGhostObjectSweepTest;
		}

		bool onGround();
		void setUpInterpolate( bool value );
	};

}
