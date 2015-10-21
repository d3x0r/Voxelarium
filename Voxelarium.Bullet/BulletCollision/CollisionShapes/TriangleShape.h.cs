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

	public class btTriangleShape : btPolyhedralConvexShape
	{

		public btVector3[] m_vertices1 = new btVector3[3];

		public override int getNumVertices()
		{
			return 3;
		}
		/*
		public ref btVector3 getVertexPtr( int index )
		{
			return m_vertices1[index];
		}

		public ref btVector3 getVertexPtr( int index )
		{
			return m_vertices1[index];
		}
		*/

		public override void getVertex( int index, out btVector3 vert )
		{
			vert = m_vertices1[index];
		}

		public override int getNumEdges()
		{
			return 3;
		}

		public override void getEdge( int i, out btVector3 pa, out btVector3 pb )
		{
			pa = m_vertices1[i];
			pb = m_vertices1[( i + 1 ) % 3];
		}


		public override void getAabb( ref btTransform t, out btVector3 aabbMin, out btVector3 aabbMax )
		{
			//		Debug.Assert(false);
			getAabbSlow( ref t, out aabbMin, out aabbMax );
		}

		public override void localGetSupportingVertexWithoutMargin( ref btVector3 dir, out btVector3 result )
		{
			btVector3 dots; dir.dot3( ref m_vertices1[0], ref m_vertices1[1], ref m_vertices1[2], out dots );
			result = m_vertices1[dots.maxAxis()];

		}

		public override void batchedUnitVectorGetSupportingVertexWithoutMargin( btVector3[] vectors, btVector3[] supportVerticesOut, int numVectors )
		{
			for( int i = 0; i < numVectors; i++ )
			{
				btVector3 dots; vectors[i].dot3( ref m_vertices1[0], ref m_vertices1[1], ref m_vertices1[2], out dots );
				supportVerticesOut[i] = m_vertices1[dots.maxAxis()];
			}
		}

		public btTriangleShape() 
		{
			m_shapeType = BroadphaseNativeTypes.TRIANGLE_SHAPE_PROXYTYPE;
		}

		public btTriangleShape( ref btVector3 p0, ref btVector3 p1, ref btVector3 p2 ) 
		{
			m_shapeType = BroadphaseNativeTypes.TRIANGLE_SHAPE_PROXYTYPE;
			m_vertices1[0] = p0;
			m_vertices1[1] = p1;
			m_vertices1[2] = p2;
		}


		public override void getPlane( out btVector3 planeNormal, out btVector3 planeSupport, int i )
		{
			getPlaneEquation( i, out planeNormal, out planeSupport );
		}

		public override int getNumPlanes()
		{
			return 1;
		}

		public void calcNormal( out btVector3 normal )
		{
			btVector3 tmp;
			btVector3 tmp2;
			m_vertices1[1].Sub( ref m_vertices1[0], out tmp );
            m_vertices1[2].Sub( ref m_vertices1[0], out tmp2 );
            tmp.cross( ref tmp2, out normal );
			normal.normalize();
		}

		public virtual void getPlaneEquation( int i, out btVector3 planeNormal, out btVector3 planeSupport )
		{
			//(void)i;
			calcNormal( out planeNormal );
			planeSupport = m_vertices1[0];
		}

		public override void calculateLocalInertia( double mass, out btVector3 inertia )
		{
//			(void)mass;
			Debug.Assert( false );
			btVector3.setValue( out inertia, ( double)( 0.0 ), btScalar.BT_ZERO, btScalar.BT_ZERO );
		}

		public override bool isInside( ref btVector3 pt, double tolerance )
		{
			btVector3 normal;
			calcNormal( out normal );
			//distance to plane
			double dist = pt.dot( ref normal );
			double planeconst = m_vertices1[0].dot( ref normal );
			dist -= planeconst;
			if( dist >= -tolerance && dist <= tolerance )
			{
				//inside check on edge-planes
				int i;
				for( i = 0; i < 3; i++ )
				{
					btVector3 pa, pb;
					getEdge( i, out pa, out pb );
					btVector3 edge; pb.Sub( ref pa, out edge );
					btVector3 edgeNormal; edge.cross( ref normal, out edgeNormal );
					edgeNormal.normalize();
					double dist2 = pt.dot( ref edgeNormal );
					double edgeConst = pa.dot( ref edgeNormal );
					dist2 -= edgeConst;
					if( dist2 < -tolerance )
						return false;
				}

				return true;
			}

			return false;
		}
		//debugging
		public override string ToString()
		{
			return "Triangle";
		}

		public override int getNumPreferredPenetrationDirections()
		{
			return 2;
		}

		public virtual void getPreferredPenetrationDirection( int index, out btVector3 penetrationVector )
		{
			calcNormal( out penetrationVector );
			if( index > 0 )
				penetrationVector.Invert( out penetrationVector );
		}


	};

}
