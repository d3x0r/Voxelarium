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

using Bullet.LinearMath;

namespace Bullet.Collision.Shapes
{

	/// PHY_ScalarType enumerates possible scalar types.
	/// See the btStridingMeshInterface or btHeightfieldTerrainShape for its use
	public enum PHY_ScalarType
	{
		PHY_FLOAT,
		PHY_DOUBLE,
		PHY_INTEGER,
		PHY_SHORT,
		PHY_FIXEDPOINT88,
		PHY_UCHAR
	};

	///The btConcaveShape class provides an interface for non-moving (static) concave shapes.
	///It has been implemented by the btStaticPlaneShape, btBvhTriangleMeshShape and btHeightfieldTerrainShape.
	public abstract class btConcaveShape : btCollisionShape
	{
		protected double m_collisionMargin;

		public btConcaveShape()
		{
			m_collisionMargin = btScalar.BT_ZERO;
		}

		internal virtual void performConvexcast( btTriangleCallback callback, ref btVector3 raySource, ref btVector3 rayTarget, ref btVector3 aabbMin, ref btVector3 aabbMax )
		{
		}

		internal abstract void processAllTriangles( btTriangleCallback callback, ref btVector3 aabbMin, ref btVector3 aabbMax );

		public override double getMargin()
		{
			return m_collisionMargin;
		}
		public override void setMargin( double collisionMargin )
		{
			m_collisionMargin = collisionMargin;
		}
	}
}
