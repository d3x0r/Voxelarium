//#define USE_CONNECTED_FACES
#define TEST_INTERNAL_OBJECTS
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2011 Advanced Micro Devices, Inc.  http://bulletphysics.org

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
	///This file was written by Erwin Coumans

	public class btConvexPolyhedron
	{

		public struct btFace
		{
			public btList<short> m_indices;
			//	List<int>	m_connectedFaces;
			public double[] m_plane;// = new double[4];
		};

		public btList<btVector3> m_vertices;
		public btList<btFace> m_faces;
		public btList<btVector3> m_uniqueEdges;

		public btVector3 m_localCenter;
		public btVector3 m_extents;
		public double m_radius;
		public btVector3 mC;
		public btVector3 mE;


		public btConvexPolyhedron()
		{

		}


		public struct btInternalVertexPair : btHashInterface
		{
			public btInternalVertexPair( short v0, short v1 )
			{
				m_v0 = ( v0 );
				m_v1 = ( v1 );
				if( m_v1 > m_v0 )
					btScalar.btSwap( ref m_v0, ref m_v1 );
			}
			public short m_v0;
			public short m_v1;
			public uint getHash()
			{
				return (uint)( m_v0 + ( m_v1 << 16 ) );
			}
			public bool Equals( btInternalVertexPair other )
			{
				return m_v0 == other.m_v0 & m_v1 == other.m_v1;
			}
		};

		public struct btInternalEdge
		{
			public btInternalEdge( bool init_default )
			{
				m_face0 = ( -1 );
				m_face1 = ( -1 );
			}
			public short m_face0;
			public short m_face1;
		};

		//

#if TEST_INTERNAL_OBJECTS
		public bool testContainment()
		{
			for( int p = 0; p < 8; p++ )
			{
				btVector3 LocalPt;
				if( p == 0 ) LocalPt = new btVector3( m_extents[0], m_extents[1], m_extents[2] );
				else if( p == 1 ) LocalPt = new btVector3( m_extents[0], m_extents[1], -m_extents[2] );
				else if( p == 2 ) LocalPt = new btVector3( m_extents[0], -m_extents[1], m_extents[2] );
				else if( p == 3 ) LocalPt = new btVector3( m_extents[0], -m_extents[1], -m_extents[2] );
				else if( p == 4 ) LocalPt = new btVector3( -m_extents[0], m_extents[1], m_extents[2] );
				else if( p == 5 ) LocalPt = new btVector3( -m_extents[0], m_extents[1], -m_extents[2] );
				else if( p == 6 ) LocalPt = new btVector3( -m_extents[0], -m_extents[1], m_extents[2] );
				else /*if( p == 7 )*/ LocalPt = new btVector3( -m_extents[0], -m_extents[1], -m_extents[2] );
				LocalPt.Add( ref m_localCenter, out LocalPt );
				for( int i = 0; i < m_faces.Count; i++ )
				{
					btVector3 Normal = new btVector3( m_faces[i].m_plane[0], m_faces[i].m_plane[1], m_faces[i].m_plane[2] );
					double d = LocalPt.dot( ref Normal ) + m_faces[i].m_plane[3];
					if( d > 0.0f )
						return false;
				}
			}
			return true;
		}
#endif

		public void initialize()
		{

			btHashMap<btInternalVertexPair, btInternalEdge> edges = new btHashMap<btInternalVertexPair, btInternalEdge>();

			double TotalArea = 0.0f;
			btVector3[] arr_vertices = m_vertices.InternalArray;
			m_localCenter.setValue( 0, 0, 0 );
			for( int i = 0; i < m_faces.Count; i++ )
			{
				int numVertices = m_faces[i].m_indices.Count;
				int NbTris = numVertices;
				for( int j = 0; j < NbTris; j++ )
				{
					int k = ( j + 1 ) % numVertices;
					btInternalVertexPair vp = new btInternalVertexPair( m_faces[i].m_indices[j], m_faces[i].m_indices[k] );
					btInternalEdge edptr = edges.find( ref vp );
					btVector3 edge; arr_vertices[vp.m_v1].Sub( ref arr_vertices[vp.m_v0], out edge );
					edge.normalize();

					bool found = false;

					for( int p = 0; p < m_uniqueEdges.Count; p++ )
					{
						btVector3 tmp;
						m_uniqueEdges[p].Sub( ref edge, out tmp );
						if( btVector3.IsAlmostZero( ref tmp ) ) { found = true; break; }
						m_uniqueEdges[p].Add( ref edge, out tmp );
						if( btVector3.IsAlmostZero( ref tmp ) ) { found = true; break; }
					}

					if( !found )
					{
						m_uniqueEdges.Add( edge );
					}
					/* this is broken; it did not port correctly... */
					Debugger.Break();
					if( edptr.m_face0 == edptr.m_face1 )
					{
						btInternalEdge ed;
						ed.m_face0 = (short)i;
						ed.m_face1 = (short)j;
						edges.insert( ref vp, ref ed );
					}
					else
					{
						Debug.Assert( edptr.m_face0 >= 0 );
						Debug.Assert( edptr.m_face1 < 0 );
						edptr.m_face1 = (short)i;
					}
				}
			}

#if USE_CONNECTED_FACES
			for( int i = 0; i < m_faces.Count; i++ )
			{
				int numVertices = m_faces[i].m_indices.Count;
				m_faces[i].m_connectedFaces.resize( numVertices );

				for( int j = 0; j < numVertices; j++ )
				{
					int k = ( j + 1 ) % numVertices;
					btInternalVertexPair vp( m_faces[i].m_indices[j], m_faces[i].m_indices[k]);
			btInternalEdge* edptr = edges.find( vp );
			Debug.Assert( edptr );
			Debug.Assert( edptr.m_face0 >= 0 );
			Debug.Assert( edptr.m_face1 >= 0 );

			int connectedFace = ( edptr.m_face0 == i ) ? edptr.m_face1 : edptr.m_face0;
			m_faces[i].m_connectedFaces[j] = connectedFace;
		}
	}
#endif//USE_CONNECTED_FACES

			for( int i = 0; i < m_faces.Count; i++ )
			{
				int numVertices = m_faces[i].m_indices.Count;
				int NbTris = numVertices - 2;

				btVector3 p0 = m_vertices[m_faces[i].m_indices[0]];
				for( int j = 1; j <= NbTris; j++ )
				{
					int k = ( j + 1 ) % numVertices;
					btVector3 p1 = m_vertices[m_faces[i].m_indices[j]];
					btVector3 p2 = m_vertices[m_faces[i].m_indices[k]];
					btVector3 tmp;
					btVector3 tmp2;
					btVector3 tmp3;
					p0.Sub( ref p1, out tmp );
					p0.Sub( ref p2, out tmp2 );
					tmp.cross( ref tmp2, out tmp3 );

					double Area = ( tmp3 ).length() * 0.5f;
					p0.Add( ref p1, out tmp );
					tmp.Add( ref p2, out tmp );
					btVector3 Center; tmp.Div( 3.0, out Center );
					m_localCenter.AddScale( ref Center, Area, out m_localCenter );
					TotalArea += Area;
				}
			}
			m_localCenter.Div( TotalArea, out m_localCenter );
			//m_localCenter /= TotalArea;




#if TEST_INTERNAL_OBJECTS
			if( true )
			{
				m_radius = double.MaxValue;
				for( int i = 0; i < m_faces.Count; i++ )
				{
					btVector3 Normal = new btVector3( m_faces[i].m_plane[0], m_faces[i].m_plane[1], m_faces[i].m_plane[2] );
					double dist = btScalar.btFabs( m_localCenter.dot( ref Normal ) + m_faces[i].m_plane[3] );
					if( dist < m_radius )
						m_radius = dist;
				}

				double MinX = double.MaxValue;
				double MinY = double.MaxValue;
				double MinZ = double.MaxValue;
				double MaxX = double.MinValue;
				double MaxY = double.MinValue;
				double MaxZ = double.MinValue;
				unsafe
				{
					fixed ( btVector3* _pt = arr_vertices )
					{
						for( int i = 0; i < m_vertices.Count; i++ )
						{
							btVector3* pt = _pt + i;
							if( pt->x < MinX ) MinX = pt->x;
							if( pt->x > MaxX ) MaxX = pt->x;
							if( pt->y < MinY ) MinY = pt->y;
							if( pt->y > MaxY ) MaxY = pt->y;
							if( pt->z < MinZ ) MinZ = pt->z;
							if( pt->z > MaxZ ) MaxZ = pt->z;
						}
					}
				}
				mC.setValue( MaxX + MinX, MaxY + MinY, MaxZ + MinZ );
				mE.setValue( MaxX - MinX, MaxY - MinY, MaxZ - MinZ );



				//		double r = m_radius / sqrtf(2.0f);
				double r = m_radius / btScalar.btSqrt( 3.0f );
				int LargestExtent = mE.maxAxis();
				double Step = ( mE[LargestExtent] * 0.5f - r ) / 1024.0f;
				m_extents[0] = m_extents[1] = m_extents[2] = r;
				m_extents[LargestExtent] = mE[LargestExtent] * 0.5f;
				bool FoundBox = false;
				for( int j = 0; j < 1024; j++ )
				{
					if( testContainment() )
					{
						FoundBox = true;
						break;
					}

					m_extents[LargestExtent] -= Step;
				}
				if( !FoundBox )
				{
					m_extents[0] = m_extents[1] = m_extents[2] = r;
				}
				else
				{
					// Refine the box
					double Step2 = ( m_radius - r ) / 1024.0f;
					int e0 = ( 1 << LargestExtent ) & 3;
					int e1 = ( 1 << e0 ) & 3;

					for( int j = 0; j < 1024; j++ )
					{
						double Saved0 = m_extents[e0];
						double Saved1 = m_extents[e1];
						m_extents[e0] += Step2;
						m_extents[e1] += Step2;

						if( !testContainment() )
						{
							m_extents[e0] = Saved0;
							m_extents[e1] = Saved1;
							break;
						}
					}
				}
			}
#endif
		}

		public void project( btITransform trans, ref btVector3 dir, out double minProj, out double maxProj, out btVector3 witnesPtMin, out btVector3 witnesPtMax )
		{
			btVector3[] arr_vertices = m_vertices.InternalArray;
			int numVerts;
			if( ( numVerts = m_vertices.Count ) > 0 )
			{
				btVector3 pt; trans.Apply( ref arr_vertices[0], out pt );
				double dp = pt.dot( ref dir );
				minProj = dp;
				witnesPtMin = pt;
				maxProj = dp;
				witnesPtMax = pt;
				for( int i = 1; i < numVerts; i++ )
				{
					trans.Apply( ref arr_vertices[i], out pt );
					dp = pt.dot( ref dir );
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
			}
			else
			{
				Debug.Assert( false );
				minProj = double.MaxValue;
				maxProj = double.MinValue;
				witnesPtMax = witnesPtMin = btVector3.Zero;
			}
#if asdfasdf
			if( minProj > maxProj )
			{
				btScalar.btSwap( ref minProj, ref maxProj );
				btScalar.btSwap( ref witnesPtMin, ref witnesPtMax );
			}
#endif
		}

	};


}

