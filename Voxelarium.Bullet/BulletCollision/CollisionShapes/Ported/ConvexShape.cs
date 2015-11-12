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

using System.Diagnostics;
using Bullet.Collision.BroadPhase;
using Bullet.LinearMath;

namespace Bullet.Collision.Shapes
{
	/// The btConvexShape is an abstract shape interface, implemented by all convex shapes such as btBoxShape, btConvexHullShape etc.
	/// It describes general convex shapes using the localGetSupportingVertex interface, used by collision detectors such as btGjkPairDetector.
	public abstract class btConvexShape : btCollisionShape
	{
		public const int MAX_PREFERRED_PENETRATION_DIRECTIONS = 10;
		public delegate btVector3 getLocalSupport( ref btVector3 vert, out btVector3 result );

		public abstract void localGetSupportingVertex( ref btVector3 vec, out btVector3 result );

		////////
		public abstract void localGetSupportingVertexWithoutMargin( ref btVector3 vec, out btVector3 result );

		//notice that the vectors should be unit length
		public abstract void batchedUnitVectorGetSupportingVertexWithoutMargin( btVector3[] vectors, btVector3[] supportVerticesOut, int numVectors );

		///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
		public override abstract void getAabb( ref btTransform t, out btVector3 aabbMin, out btVector3 aabbMax );

		public abstract void getAabbSlow( ref btTransform t, out btVector3 aabbMin, out btVector3 aabbMax );

		public abstract override void setLocalScaling( ref btVector3 scaling );
		public abstract override void getLocalScaling( out btVector3 result );

		public abstract override void setMargin( double margin );

		public abstract override double getMargin();

		public abstract int getNumPreferredPenetrationDirections();

		public abstract void getPreferredPenetrationDirection( int index, out btVector3 penetrationVector );



		public virtual void project( ref btTransform trans, ref btVector3 dir
			, ref double min, ref double max
			, out btVector3 witnesPtMin, out btVector3 witnesPtMax )
		{
			btVector3 localAxis; trans.m_basis.ApplyInverse( ref dir, out localAxis );
			btVector3 tmpv;
			localGetSupportingVertex( ref localAxis, out tmpv );
			btVector3 vtx1; trans.Apply( ref tmpv, out vtx1 );
			localAxis.Invert( out localAxis );
			localGetSupportingVertex( ref localAxis, out tmpv );

			btVector3 vtx2; trans.Apply( ref tmpv, out vtx2 );

			min = vtx1.dot( ref dir );
			max = vtx2.dot( ref dir );
			witnesPtMax = vtx2;
			witnesPtMin = vtx1;

			if( min > max )
			{
				double tmp = min;
				min = max;
				max = tmp;
				witnesPtMax = vtx1;
				witnesPtMin = vtx2;
			}
		}


		static void convexHullSupport( ref btVector3 localDirOrg, btVector3[] points, int numPoints, ref btVector3 localScaling, out btVector3 result )
		{
			btVector3 vec; localDirOrg.Mult( ref localScaling, out vec );
			double maxDot;
			long ptIndex = vec.maxDot( points, numPoints, out maxDot );
			Debug.Assert( ptIndex >= 0 );
			points[ptIndex].Mult( ref localScaling, out result );
		}

		internal void localGetSupportVertexWithoutMarginNonVirtual( ref btVector3 localDir, out btVector3 result )
		{
			switch( m_shapeType )
			{
				case BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE:
					{
						result = btVector3.Zero;
						return;
					}
				case BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE:
					{
						btBoxShape convexShape = (btBoxShape)this;
						btVector3 halfExtents = convexShape.getImplicitShapeDimensions();

						result = new btVector3( btScalar.btFsels( localDir.x, halfExtents.x, -halfExtents.x ),
							btScalar.btFsels( localDir.y, halfExtents.y, -halfExtents.y ),
							btScalar.btFsels( localDir.z, halfExtents.z, -halfExtents.z ) );
						return;
					}
				case BroadphaseNativeTypes.TRIANGLE_SHAPE_PROXYTYPE:
					{
						btTriangleShape triangleShape = (btTriangleShape)this;
						btVector3 dir = localDir;
						//btVector3[] vertices = triangleShape.m_vertices1;
						btVector3 dots; dir.dot3( ref triangleShape.m_vertices1, ref triangleShape.m_vertices2
							, ref triangleShape.m_vertices2, out dots );
						triangleShape.getVertex( dots.maxAxis(), out result );
						return;
					}
				case BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE:
					{
						btCylinderShape cylShape = (btCylinderShape)this;
						cylShape.localGetSupportingVertexWithoutMargin( ref localDir, out result );
						return;
					}
				case BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE:
					{
						btCapsuleShape capsuleShape = (btCapsuleShape)this;
						capsuleShape.localGetSupportingVertexWithoutMargin( ref localDir, out result );
						return;
					}
				case BroadphaseNativeTypes.CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE:
					{
						btConvexPointCloudShape convexPointCloudShape = (btConvexPointCloudShape)this;
						//convexPointCloudShape.localGetSupportingVertexWithoutMargin( ref localDir, out result );
						btVector3[] points = convexPointCloudShape.getUnscaledPoints();
						int numPoints = convexPointCloudShape.getNumPoints();
						btVector3 tmp;
						convexPointCloudShape.getLocalScalingNV( out tmp );
						btConvexShape.convexHullSupport( ref localDir, points, numPoints, ref tmp, out result );
						return;
					}
				case BroadphaseNativeTypes.CONVEX_HULL_SHAPE_PROXYTYPE:
					{
						btConvexHullShape convexHullShape = (btConvexHullShape)this;
						btVector3[] points = convexHullShape.getUnscaledPoints();
						int numPoints = convexHullShape.getNumPoints();
						btVector3 tmp;
						convexHullShape.getLocalScalingNV( out tmp );
						convexHullSupport( ref localDir, points, numPoints, ref tmp, out result );
						return;
					}
				default:
					this.localGetSupportingVertexWithoutMargin( ref localDir, out result );
					return;
			}

			// should never reach here
			//Debug.Assert( false );
			//result = btVector3.Zero;
		}

		public void localGetSupportVertexNonVirtual( ref btVector3 localDir, out btVector3 result )
		{
			btVector3 localDirNorm = localDir;
			if( localDirNorm.length2() < ( btScalar.SIMD_EPSILON * btScalar.SIMD_EPSILON ) )
			{
				localDirNorm = btVector3.NegOne;
			}
			localDirNorm.normalize();
			localDirNorm.Mult( getMarginNonVirtual(), out localDirNorm );
			localGetSupportVertexWithoutMarginNonVirtual( ref localDirNorm, out result );
			result.Add( ref localDirNorm, out result );
		}

		/* TODO: This should be bumped up to btCollisionShape () */
		public double getMarginNonVirtual()
		{
			switch( m_shapeType )
			{
				case BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE:
					{
						btSphereShape sphereShape = (btSphereShape)this;
						return sphereShape.getRadius();
					}
				case BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE:
					{
						btBoxShape convexShape = (btBoxShape)this;
						return convexShape.getMarginNV();
					}
				case BroadphaseNativeTypes.TRIANGLE_SHAPE_PROXYTYPE:
					{
						btTriangleShape triangleShape = (btTriangleShape)this;
						return triangleShape.getMarginNV();
					}
				case BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE:
					{
						btCylinderShape cylShape = (btCylinderShape)this;
						return cylShape.getMarginNV();
					}
				case BroadphaseNativeTypes.CONE_SHAPE_PROXYTYPE:
					{
						btConeShape conShape = (btConeShape)this;
						return conShape.getMarginNV();
					}
				case BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE:
					{
						btCapsuleShape capsuleShape = (btCapsuleShape)this;
						return capsuleShape.getMarginNV();
					}
				case BroadphaseNativeTypes.CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE:
				/* fall through */
				case BroadphaseNativeTypes.CONVEX_HULL_SHAPE_PROXYTYPE:
					{
						btPolyhedralConvexShape convexHullShape = (btPolyhedralConvexShape)this;
						return convexHullShape.getMarginNV();
					}
				default:
					return this.getMargin();
			}

			// should never reach here
			//Debug.Assert( false );
			//return (double)( 0.0f );
		}

		public void getAabbNonVirtual( ref btTransform t, out btVector3 aabbMin, out btVector3 aabbMax )
		{
			switch( m_shapeType )
			{
				case BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE:
					{
						btSphereShape sphereShape = (btSphereShape)this;
						double radius = sphereShape.getImplicitShapeDimensions().x;// * convexShape.getLocalScaling().x;
						double margin = radius + sphereShape.getMarginNonVirtual();
						btVector3 extent = new btVector3( margin, margin, margin );
						t.m_origin.Sub( ref extent, out aabbMin );
						t.m_origin.Add( ref extent, out aabbMax );
					}
					break;
				case BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE:
				/* fall through */
				case BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE:
					{
						btBoxShape convexShape = (btBoxShape)this;
						double margin = convexShape.getMarginNonVirtual();
						btVector3 halfExtents = convexShape.getImplicitShapeDimensions();
						btVector3 tmp = new btVector3( margin, margin, margin );
						halfExtents.Add( ref tmp, out halfExtents );
						btMatrix3x3 abs_b; t.m_basis.absolute( out abs_b );
						btVector3 extent; halfExtents.dot3( ref abs_b.m_el0, ref abs_b.m_el1, ref abs_b.m_el2, out extent );

						t.m_origin.Sub( ref extent, out aabbMin );
						t.m_origin.Add( ref extent, out aabbMax );
						break;
					}
				case BroadphaseNativeTypes.TRIANGLE_SHAPE_PROXYTYPE:
					{
						btTriangleShape triangleShape = (btTriangleShape)this;
						double margin = triangleShape.getMarginNonVirtual();
						aabbMax = aabbMin = btVector3.Zero;
						for( int i = 0; i < 3; i++ )
						{
							btVector3 vec = btVector3.Zero;
							vec[i] = (double)( 1.0 );
							btVector3 tmp;
							t.m_basis.ApplyInverse( ref vec, out tmp );
							btVector3 sv; localGetSupportVertexWithoutMarginNonVirtual( ref tmp, out sv );

							t.Apply( ref sv, out tmp );

							aabbMax[i] = tmp[i] + margin;
							vec[i] = (double)( -1.0);

							t.m_basis.ApplyInverse( ref vec, out tmp );
                            localGetSupportVertexWithoutMarginNonVirtual( ref tmp, out sv );
							t.Apply( ref sv, out tmp );
							aabbMin[i] = tmp[i] - margin;
						}
					}
					break;
				case BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE:
					{
						btCapsuleShape capsuleShape = (btCapsuleShape)this;
						btVector3 halfExtents = new btVector3( capsuleShape.getRadius(), capsuleShape.getRadius(), capsuleShape.getRadius());
						int m_upAxis = capsuleShape.getUpAxis();
						halfExtents[m_upAxis] = capsuleShape.getRadius() + capsuleShape.getHalfHeight();
						btVector3 tmp = new btVector3( capsuleShape.getMarginNonVirtual(), capsuleShape.getMarginNonVirtual(), capsuleShape.getMarginNonVirtual() );
                        halfExtents.Add( ref tmp, out halfExtents );
						btMatrix3x3 abs_b; t.m_basis.absolute( out abs_b );
						btVector3 extent; halfExtents.dot3( ref abs_b.m_el0, ref abs_b.m_el1, ref abs_b.m_el2, out extent );
						t.m_origin.Sub( ref extent, out aabbMin );
						t.m_origin.Add( ref extent, out aabbMax );
					}
					break;
				case BroadphaseNativeTypes.CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE:
				case BroadphaseNativeTypes.CONVEX_HULL_SHAPE_PROXYTYPE:
					{
						btPolyhedralConvexAabbCachingShape convexHullShape = (btPolyhedralConvexAabbCachingShape)this;
						double margin = convexHullShape.getMarginNonVirtual();
						convexHullShape.getNonvirtualAabb( ref t, out aabbMin, out aabbMax, margin );
					}
					break;
				default:
					getAabb( ref t, out aabbMin, out aabbMax );
					break;
			}

			// should never reach here
			Debug.Assert( false );
			aabbMin = aabbMax = btVector3.Zero;
		}
	}
}
