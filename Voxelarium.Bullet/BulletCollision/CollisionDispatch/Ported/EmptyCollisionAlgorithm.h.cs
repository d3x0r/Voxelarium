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


using System;
using Bullet.Collision.BroadPhase;
using Bullet.LinearMath;

namespace Bullet.Collision.Dispatch
{

	///EmptyAlgorithm is a stub for unsupported collision pairs.
	///The dispatcher can dispatch a persistent btEmptyAlgorithm to avoid a search every frame.
	internal class btEmptyAlgorithm : btCollisionAlgorithm
	{


		internal override void getAllContactManifolds( btManifoldArray manifoldArray )
		{
		}

		internal class CreateFunc : btCollisionAlgorithmCreateFunc
		{
			internal override btCollisionAlgorithm CreateCollisionAlgorithm( btCollisionAlgorithmConstructionInfo ci, btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap )
			{
				btEmptyAlgorithm ca = BulletGlobals.EmptyAlgorithmPool.Get();
				ca.Initialize( ci );
				return ca;
			}
		};

		internal override void Cleanup()
		{
			BulletGlobals.EmptyAlgorithmPool.Free( this );
		}

		public btEmptyAlgorithm()
		{ }

		/*
		internal void Initialize( btCollisionAlgorithmConstructionInfo ci)
		{
			base.Initialize( ci );
        }
		*/

		internal override void processCollision( btCollisionObjectWrapper a
			, ref btTransform body0Transform
			, btCollisionObjectWrapper body1Wrap
			, ref btTransform body1Transform
			, btDispatcherInfo c, btManifoldResult d)
		{
		}

		internal override double calculateTimeOfImpact( btCollisionObject a, btCollisionObject b, btDispatcherInfo c , btManifoldResult d )
		{
			return btScalar.BT_ONE;
		}

	}

}
