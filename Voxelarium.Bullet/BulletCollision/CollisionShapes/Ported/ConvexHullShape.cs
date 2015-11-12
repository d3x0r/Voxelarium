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
using Bullet.LinearMath;
using Bullet.Types;

namespace Bullet.Collision.Shapes
{

	///The btConvexHullShape implements an implicit convex hull of an array of vertices.
	///Bullet provides a general and fast collision detector for convex shapes based on GJK and EPA using localGetSupportingVertex.
	public class btConvexHullShape : btPolyhedralConvexAabbCachingShape
	{
		btList<btVector3> m_unscaledPoints = new btList<btVector3>();


		public btVector3[] getUnscaledPoints()
		{
			return m_unscaledPoints.InternalArray;
		}

		///getPoints is obsolete, please use getUnscaledPoints
		public btVector3[] getPoints()
		{
			return getUnscaledPoints();
		}


		public void getScaledPoint( int i, out btVector3 result )
		{
			m_unscaledPoints[i].Mult( ref m_localScaling, out result );
		}

		public int getNumPoints()
		{
			return m_unscaledPoints.Count;
		}


		public override string ToString() { return "Convex"; }



		///this constructor optionally takes in a pointer to points. Each point is assumed to be 3 consecutive double (x,y,z), the striding defines the number of bytes between each point, in memory.
		///It is easier to not pass any points in the constructor, and just add one point at a time, using addPoint.
		///btConvexHullShape make an internal copy of the points.
		public btConvexHullShape( btVector3[] points, int numPoints = 0 )
		{
			m_shapeType = BroadPhase.BroadphaseNativeTypes.CONVEX_HULL_SHAPE_PROXYTYPE;
			m_unscaledPoints.Count = m_unscaledPoints.Capacity = ( numPoints );

			for( int i = 0; i < numPoints; i++ )
			{
				m_unscaledPoints[i] = points[i];
			}

			recalcLocalAabb();

		}



		///in case we receive negative scaling
		public override void setLocalScaling( ref btVector3 scaling )
		{
			m_localScaling = scaling;
			recalcLocalAabb();
		}

		public void addPoint( ref btVector3 point, bool recalculateLocalAabb = true )
		{
			m_unscaledPoints.Add( point );
			if( recalculateLocalAabb )
				recalcLocalAabb();

		}

		public override void localGetSupportingVertexWithoutMargin( ref btVector3 vec, out btVector3 result )
		{
			result = btVector3.Zero;
			double maxDot = (double)( -btScalar.BT_LARGE_FLOAT );

			// Here we take advantage of dot(a, b*c) = dot(a*b, c).  Note: This is true mathematically, but not numerically. 
			if( 0 < m_unscaledPoints.Count )
			{
				btVector3 scaled; vec.Mult(ref m_localScaling, out scaled );
				int index = (int)scaled.maxDot( m_unscaledPoints.InternalArray, m_unscaledPoints.Count, out maxDot ); // FIXME: may violate encapsulation of m_unscaledPoints
				m_unscaledPoints[index].Mult( ref m_localScaling, out result );
			}

			//return supVec;
		}

		public override void batchedUnitVectorGetSupportingVertexWithoutMargin( btVector3[] vectors, btVector3[] supportVerticesOut, int numVectors )
		{
			double newDot;
			//use 'w' component of supportVerticesOut?
			{
				for( int i = 0; i < numVectors; i++ )
				{
					supportVerticesOut[i][3] = (double)( -btScalar.BT_LARGE_FLOAT );
				}
			}

			for( int j = 0; j < numVectors; j++ )
			{
				btVector3 vec; vectors[j].Mult( ref m_localScaling, out vec );        // dot(a*b,c) = dot(a,b*c)
				if( 0 < m_unscaledPoints.Count )
				{
					int i = (int)vec.maxDot( m_unscaledPoints.InternalArray, m_unscaledPoints.Count, out newDot );
					getScaledPoint( i, out supportVerticesOut[j] );
					supportVerticesOut[j][3] = newDot;
				}
				else
					supportVerticesOut[j][3] = -btScalar.BT_LARGE_FLOAT;
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
			return m_unscaledPoints.Count;
		}

		public override int getNumEdges()
		{
			return m_unscaledPoints.Count;
		}

		public override void getEdge( int i, out btVector3 pa, out btVector3 pb )
		{

			int index0 = i % m_unscaledPoints.Count;
			int index1 = ( i + 1 ) % m_unscaledPoints.Count;
			 getScaledPoint( index0, out pa );
			getScaledPoint( index1, out pb );
		}

		public override void getVertex( int i, out btVector3 vtx )
		{
			getScaledPoint( i, out vtx );
		}

		public override int getNumPlanes()
		{
			return 0;
		}

		public override void getPlane( out btVector3 normal, out btVector3 support, int i )
		{
			Debug.Assert( false );
			normal = btVector3.xAxis;
			support = btVector3.Zero;
		}

		//not yet
		public override bool isInside( ref btVector3 pt, double tolerance )
		{
			Debug.Assert( false );
			return false;
		}

#if SERIALIZE_DONE
///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btConvexHullShapeData
{
	btConvexInternalShapeData	m_convexInternalShapeData;

	btVector3FloatData	*m_unscaledPointsFloatPtr;
	btVector3DoubleData	*m_unscaledPointsDoublePtr;

	int		m_numUnscaledPoints;
	char m_padding3[4];

};


public	int	btConvexHullShape::calculateSerializeBufferSize()
{
	return sizeof(btConvexHullShapeData);
}
///fills the dataBuffer and returns the struct name (and 0 on failure)
string btConvexHullShape::serialize( object dataBuffer, btSerializer* serializer )
{
	//int szc = sizeof(btConvexHullShapeData);
	btConvexHullShapeData* shapeData = (btConvexHullShapeData*)dataBuffer;
	btConvexInternalShape::serialize( shapeData.m_convexInternalShapeData, serializer );

	int numElem = m_unscaledPoints.Count;
	shapeData.m_numUnscaledPoints = numElem;
#if BT_USE_DOUBLE_PRECISION
	shapeData.m_unscaledPointsFloatPtr = 0;
	shapeData.m_unscaledPointsDoublePtr = numElem ? (btVector3Data*)serializer.getUniquePointer((object)&m_unscaledPoints):  0;
#else
	shapeData.m_unscaledPointsFloatPtr = numElem ? (btVector3Data*)serializer.getUniquePointer( (object)m_unscaledPoints ) : 0;
	shapeData.m_unscaledPointsDoublePtr = 0;
#endif

	if( numElem )
	{
		int sz = sizeof( btVector3Data );
		//	int sz2 = sizeof(btVector3DoubleData);
		//	int sz3 = sizeof(btVector3FloatData);
		btChunk* chunk = serializer.allocate( sz, numElem );
		btVector3Data* memPtr = (btVector3Data*)chunk.m_oldPtr;
		for( int i = 0; i < numElem; i++, memPtr++ )
		{
			m_unscaledPoints[i].serialize( *memPtr );
		}
		serializer.finalizeChunk( chunk, btVector3DataName, BT_ARRAY_CODE, (object)m_unscaledPoints );
	}

	return "btConvexHullShapeData";
}
#endif
		public override void project( ref btTransform trans, ref btVector3 dir, ref double minProj, ref double maxProj, out btVector3 witnesPtMin, out btVector3 witnesPtMax )
		{
			minProj = btScalar.BT_MAX_FLOAT;
			maxProj = btScalar.BT_MIN_FLOAT;
			witnesPtMax = witnesPtMin = btVector3.Zero;
            int numVerts = m_unscaledPoints.Count;
			for( int i = 0; i < numVerts; i++ )
			{
				btVector3 vtx; m_unscaledPoints[i].Mult(ref m_localScaling, out vtx );
				btVector3 pt; trans.Apply( ref vtx, out pt );
				double dp = pt.dot( ref dir );
				if( dp < minProj )
				{
					minProj = dp;
					witnesPtMin = pt;
				}
				if( dp > maxProj )
				{
					maxProj = dp;
					witnesPtMax = pt;
				}
			}

			if( minProj > maxProj )
			{
				btScalar.btSwap( ref minProj, ref maxProj );
				btScalar.btSwap( ref witnesPtMin, ref witnesPtMax );
			}


		}




	};



}
