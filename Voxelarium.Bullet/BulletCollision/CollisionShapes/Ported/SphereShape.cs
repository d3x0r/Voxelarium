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

namespace Bullet.Collision.Shapes
{

	public class btSphereShape : btConvexInternalShape
	{

		public btSphereShape()
		{
			m_shapeType = BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE;
			m_implicitShapeDimensions.x = 1.0;
			m_collisionMargin = 1.0;
		}
		public void Initialize( double radius )
		{
			m_implicitShapeDimensions.x = ( radius );
			m_collisionMargin = radius;
		}
		public btSphereShape( double radius )
		{
			m_shapeType = BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE;
			m_implicitShapeDimensions.x = ( radius );
			m_collisionMargin = radius;
		}


		public double getRadius()
		{
			return m_implicitShapeDimensions.x * m_localScaling.x;
		}

		void setUnscaledRadius( double radius )
		{
			m_implicitShapeDimensions.x = radius;
			base.setMargin( radius );
		}

		//debugging
		public override string ToString()
		{ return "Sphere"; }

		public override void setMargin( double margin )
		{
			base.setMargin( margin );
		}

		public override double getMargin()
		{
			//to improve gjk behaviour, use radius+margin as the full margin, so never get into the penetration case
			//this means, non-uniform scaling is not supported anymore
			return getRadius();
		}

		public override void localGetSupportingVertexWithoutMargin( ref btVector3 vec, out btVector3 result )
		{
			//(void)vec;
			result = btVector3.Zero;
		}

		//notice that the vectors should be unit length
		public override void batchedUnitVectorGetSupportingVertexWithoutMargin( btVector3[] vectors, btVector3[] supportVerticesOut, int numVectors )
		{
			//(void)vectors;

			for( int i = 0; i < numVectors; i++ )
			{
				supportVerticesOut[i] = btVector3.Zero;
			}
		}


		public override void localGetSupportingVertex( ref btVector3 vec, out btVector3 supVertex )
		{
			localGetSupportingVertexWithoutMargin( ref vec, out supVertex );

			btVector3 vecnorm = vec;
			if( vecnorm.length2() < ( btScalar.SIMD_EPSILON * btScalar.SIMD_EPSILON ) )
			{
				vecnorm.setValue( (double)( -1.0), (double)( -1.0), (double)( -1.0) );
			}
			vecnorm.normalize();
			supVertex.AddScale( ref vecnorm, getMargin(), out supVertex );
			//supVertex += getMargin() * vecnorm;
		}


		//broken due to scaling
		public override void getAabb( ref btTransform t, out btVector3 aabbMin, out btVector3 aabbMax )
		{
			btVector3 extent = new btVector3( getMargin(), getMargin(), getMargin());
			t.m_origin.Sub( ref extent, out aabbMin );
			t.m_origin.Add( ref extent, out aabbMax );
		}


		public override void calculateLocalInertia( double mass, out btVector3 inertia )
		{
			double elem = (double)( 0.4 ) * mass * getMargin() * getMargin();
			btVector3.setValue( out inertia, elem, elem, elem );

		}
	}
}
