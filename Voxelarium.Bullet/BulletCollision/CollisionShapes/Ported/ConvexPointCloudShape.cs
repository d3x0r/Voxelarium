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


	///The btConvexPointCloudShape implements an implicit convex hull of an array of vertices.
	internal class btConvexPointCloudShape : btPolyhedralConvexAabbCachingShape
	{
		btVector3[] m_unscaledPoints;
		int m_numPoints;



		public btConvexPointCloudShape()
		{
			m_localScaling = btVector3.One;
			m_shapeType = BroadphaseNativeTypes.CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE;
			m_unscaledPoints = null;
			m_numPoints = 0;
		}

		public btConvexPointCloudShape( btVector3[] points, int numPoints, ref btVector3 localScaling, bool computeAabb = true )
		{
			m_localScaling = localScaling;
			m_shapeType = BroadphaseNativeTypes.CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE;
			m_unscaledPoints = points;
			m_numPoints = numPoints;

			if( computeAabb )
				recalcLocalAabb();
		}

		public void setPoints( btVector3[] points, int numPoints, bool computeAabb, ref btVector3 localScaling )
		{
			m_unscaledPoints = points;
			m_numPoints = numPoints;
			m_localScaling = localScaling;

			if( computeAabb )
				recalcLocalAabb();
		}
		public void setPoints( btVector3[] points, int numPoints, bool computeAabb = true )
		{
			setPoints( points, numPoints, computeAabb, ref btVector3.One );
		}

		public btVector3[] getUnscaledPoints()
		{
			return m_unscaledPoints;
		}


		public virtual int getNumPoints()
		{
			return m_numPoints;
		}

		public void getScaledPoint( int index, out btVector3 result )
		{
			m_unscaledPoints[index].Mult( ref m_localScaling, out result );
			//return m_unscaledPoints[index] * m_localScaling;
		}

		//debugging
		public override string ToString()
		{
			return "ConvexPointCloud";
		}


		///in case we receive negative scaling
		public override void setLocalScaling( ref btVector3 scaling )
		{
			m_localScaling = scaling;
			recalcLocalAabb();
		}

		public override void localGetSupportingVertexWithoutMargin( ref btVector3 vec0, out btVector3 result )
		{
			result = btVector3.Zero;
			double maxDot = (double)( -btScalar.BT_LARGE_FLOAT );

			btVector3 vec = vec0;
			double lenSqr = vec.length2();
			if( lenSqr < (double)( 0.0001 ) )
			{
				vec.setValue( 1, 0, 0 );
			}
			else
			{
				double rlen = (double)( 1.0 ) / btScalar.btSqrt( lenSqr );
				//vec *= rlen;
				vec.Mult( rlen, out vec );
			}

			if( m_numPoints > 0 )
			{
				// Here we take advantage of dot(a*b, c) = dot( a, b*c) to do less work. Note this transformation is true mathematically, not numerically.
				//    btVector3 scaled = vec * m_localScaling;
				int index = (int)vec.maxDot( m_unscaledPoints, m_numPoints, out maxDot );   //FIXME: may violate encapsulation of m_unscaledPoints
				getScaledPoint( index, out result );
			}
		}

		public override void batchedUnitVectorGetSupportingVertexWithoutMargin( btVector3[] vectors, btVector3[] supportVerticesOut, int numVectors )
		{
			for( int j = 0; j < numVectors; j++ )
			{
				btVector3 vec; vectors[j].Mult( ref m_localScaling, out vec );  // dot( a*c, b) = dot(a, b*c)
				double maxDot;
				int index = (int)vec.maxDot( m_unscaledPoints, m_numPoints, out maxDot );
				supportVerticesOut[j][3] = (double)( -btScalar.BT_LARGE_FLOAT );
				if( 0 <= index )
				{
					//WARNING: don't swap next lines, the w component would get overwritten!
					getScaledPoint( index, out supportVerticesOut[j] );
					supportVerticesOut[j][3] = maxDot;
				}
			}
		}



		public override void localGetSupportingVertex( ref btVector3 vec, out btVector3 result )
		{
			localGetSupportingVertexWithoutMargin( ref vec, out result );

			if( getMargin() != btScalar.BT_ZERO )
			{
				btVector3 vecnorm = vec;
				if( vecnorm.length2() < ( btScalar.SIMD_EPSILON * btScalar.SIMD_EPSILON ) )
				{
					vecnorm = btVector3.NegOne;
				}
				vecnorm.normalize();
				vecnorm.Mult( getMargin(), out vecnorm );
				result.Add( ref vecnorm, out result );
			}
		}



		//currently just for debugging (drawing), perhaps future support for algebraic continuous collision detection
		//Please note that you can debug-draw btConvexHullShape with the Raytracer Demo
		public override int getNumVertices()
		{
			return m_numPoints;
		}

		public override int getNumEdges()
		{
			return 0;
		}

		public override void getEdge( int i, out btVector3 pa, out btVector3 pb )
		{
			// clouds don't have edges.
			Debug.Assert( false );
			pb = pa = btVector3.Zero;
		}

		public override void getVertex( int i, out btVector3 vtx )
		{
			m_unscaledPoints[i].Mult( ref m_localScaling, out vtx );
		}

		public override int getNumPlanes()
		{
			return 0;
		}

		public override void getPlane( ref btVector3 o, ref btVector3 n, int side )
		{

			Debug.Assert( false );
		}

		//not yet
		public override bool isInside( ref btVector3 v, double a)
		{
			Debug.Assert( false );
			return false;
		}

	}

}


