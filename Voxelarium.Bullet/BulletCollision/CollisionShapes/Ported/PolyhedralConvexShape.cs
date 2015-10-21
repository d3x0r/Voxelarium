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


	///The btPolyhedralConvexShape is an internal interface class for polyhedral convex shapes.
	public abstract class btPolyhedralConvexShape : btConvexInternalShape
	{

		protected btConvexPolyhedron m_polyhedron;



		public btConvexPolyhedron getConvexPolyhedron()
		{
			return m_polyhedron;
		}

		//brute force implementations


		public abstract int getNumVertices();
		public abstract int getNumEdges();
		public abstract void getEdge( int i, out btVector3 pa, out btVector3 pb );
		public abstract void getVertex( int i, out btVector3 vtx );
		public abstract int getNumPlanes();
		public abstract void getPlane( out btVector3 planeNormal, out btVector3 planeSupport, int i );
		//	virtual int getIndex(int i) string  0 ; 

		public abstract bool isInside( ref btVector3 pt, double tolerance );

		public btPolyhedralConvexShape()
		{
			m_polyhedron = null;
		}

		~btPolyhedralConvexShape()
		{
			if( m_polyhedron != null )
			{
				m_polyhedron = null;
			}
		}


		///optional method mainly used to generate multiple contact points by clipping polyhedral features (faces/edges)
		///experimental/work-in-progress
		public virtual bool initializePolyhedralFeatures( bool shiftVerticesByMargin = false )
		{

			if( m_polyhedron != null )
			{
				m_polyhedron = null;
			}

			m_polyhedron = new btConvexPolyhedron();

			int count = getNumVertices();
			btList<btVector3> orgVertices = new btList<btVector3>( count );
			btVector3[] arrVertices = orgVertices.InternalArray;
			for( int i = 0; i < count; i++ )
			{
				getVertex( i, out arrVertices[i] );
			}

			btConvexHullComputer conv = new btConvexHullComputer();

			if( shiftVerticesByMargin )
			{
				btList<btVector3> planeEquations = new btList<btVector3>();
				btGeometryUtil.getPlaneEquationsFromVertices( orgVertices, planeEquations );

				btList<btVector3> shiftedPlaneEquations = new btList<btVector3>();
				for( int p = 0; p < planeEquations.Count; p++ )
				{
					btVector3 plane = planeEquations[p];
					//	   double margin = getMargin();
					plane[3] -= getMargin();
					shiftedPlaneEquations.Add( plane );
				}

				btList<btVector3> tmpVertices = new btList<btVector3>();

				btGeometryUtil.getVerticesFromPlaneEquations( shiftedPlaneEquations, tmpVertices );

				conv.compute( tmpVertices, tmpVertices.Count, 0, 0 );
			}
			else
			{
				conv.compute( orgVertices, orgVertices.Count, 0, 0 );
			}


			btList<btVector3> faceNormals = new btList<btVector3>( conv.faces.Count );
			int numFaces = conv.faces.Count;
			btConvexHullComputer convexUtil = conv;
			btVector3[] arr_faceNormals = faceNormals.InternalArray;


			btList<btConvexPolyhedron.btFace> tmpFaces = new btList<btConvexPolyhedron.btFace>( numFaces );

			int numVertices = convexUtil.vertices.Count;
			m_polyhedron.m_vertices.Count = m_polyhedron.m_vertices.Capacity = ( numVertices );

			for( int p = 0; p < numVertices; p++ )
			{
				m_polyhedron.m_vertices[p] = convexUtil.vertices[p];
			}


			for( int i = 0; i < numFaces; i++ )
			{
				btConvexHullComputer.Edge face = convexUtil.faces[i];
				//Console.WriteLine("face=%d\n",face);
				//btConvexHullComputer::Edge* firstEdge = &convexUtil.edges[face];
				btConvexHullComputer.Edge edge = face;

				btVector3[] edges = new btVector3[3];
				int numEdges = 0;
				//compute face normals

				do
				{
					int src = edge.getSourceVertex();
					tmpFaces[i].m_indices.Add( (short)src );
					int targ = edge.getTargetVertex();
					btVector3 wa = convexUtil.vertices[src];

					btVector3 wb = convexUtil.vertices[targ];
					btVector3 newEdge; wb.Sub( ref wa, out newEdge );
					newEdge.normalize();
					if( numEdges < 2 )
						edges[numEdges++] = newEdge;

					edge = edge.getNextEdgeOfFace();
				} while( edge != face );

				double planeEq = btScalar.BT_LARGE_FLOAT;


				if( numEdges == 2 )
				{
					//faceNormals[i]
					edges[0].cross( ref edges[1], out faceNormals.InternalArray[i] );
					faceNormals[i].normalize();
					tmpFaces[i].m_plane[0] = faceNormals[i].x;
					tmpFaces[i].m_plane[1] = faceNormals[i].y;
					tmpFaces[i].m_plane[2] = faceNormals[i].z;
					tmpFaces[i].m_plane[3] = planeEq;

				}
				else
				{
					Debug.Assert( false );//degenerate?
					faceNormals[i].setZero();
				}

				for( int v = 0; v < tmpFaces[i].m_indices.Count; v++ )
				{
					double eq = m_polyhedron.m_vertices[tmpFaces[i].m_indices[v]].dot( ref arr_faceNormals[i] );
					if( planeEq > eq )
					{
						planeEq = eq;
					}
				}
				tmpFaces[i].m_plane[3] = -planeEq;
			}

			//merge coplanar faces and copy them to m_polyhedron

			double faceWeldThreshold = 0.999f;
			btList<int> todoFaces = new btList<int>();
			for( int i = 0; i < tmpFaces.Count; i++ )
				todoFaces.Add( i );

			btList<int> coplanarFaceGroup = new btList<int>();
			while( todoFaces.Count > 0 )
			{
				int refFace = todoFaces[todoFaces.Count - 1];

				coplanarFaceGroup.Add( refFace );
				btConvexPolyhedron.btFace faceA = tmpFaces[refFace];
				todoFaces.Count--;

				btVector3 faceNormalA = new btVector3( faceA.m_plane[0], faceA.m_plane[1], faceA.m_plane[2] );
				for( int j = todoFaces.Count - 1; j >= 0; j-- )
				{
					int i = todoFaces[j];
					btConvexPolyhedron.btFace faceB = tmpFaces[i];
					btVector3 faceNormalB = new btVector3( faceB.m_plane[0], faceB.m_plane[1], faceB.m_plane[2] );
					if( faceNormalA.dot( ref faceNormalB ) > faceWeldThreshold )
					{
						coplanarFaceGroup.Add( i );
						todoFaces.RemoveAt( i );
					}
				}


				bool did_merge = false;
				if( coplanarFaceGroup.Count > 1 )
				{
					//do the merge: use Graham Scan 2d convex hull

					btList<GrahamVector3> orgpoints = new btList<GrahamVector3>();
					btVector3 averageFaceNormal = btVector3.Zero;

					for( int i = 0; i < coplanarFaceGroup.Count; i++ )
					{
						//				m_polyhedron.m_faces.Add(tmpFaces[coplanarFaceGroup[i]]);

						btConvexPolyhedron.btFace face = tmpFaces[coplanarFaceGroup[i]];
						btVector3 faceNormal = new btVector3( face.m_plane[0], face.m_plane[1], face.m_plane[2] );
						averageFaceNormal.Add( ref faceNormal, out averageFaceNormal );
						for( int f = 0; f < face.m_indices.Count; f++ )
						{
							int orgIndex = face.m_indices[f];
							btVector3 pt = m_polyhedron.m_vertices[orgIndex];

							bool found = false;

							for( int j = 0; j < orgpoints.Count; j++ )
							{
								//if ((orgpoints[i].m_orgIndex == orgIndex) || ((rotatedPt-orgpoints[i]).length2()<0.0001))
								if( orgpoints[j].m_orgIndex == orgIndex )
								{
									found = true;
									break;
								}
							}
							if( !found )
								orgpoints.Add( new GrahamVector3( ref pt, orgIndex ) );
						}
					}



					btConvexPolyhedron.btFace combinedFace = new btConvexPolyhedron.btFace();
					for( int i = 0; i < 4; i++ )
						combinedFace.m_plane[i] = tmpFaces[coplanarFaceGroup[0]].m_plane[i];

					btList<GrahamVector3> hull = new btList<GrahamVector3>();

					averageFaceNormal.normalize();
					GrahamVector3.GrahamScanConvexHull2D( orgpoints, hull, ref averageFaceNormal );

					for( int i = 0; i < hull.Count; i++ )
					{
						combinedFace.m_indices.Add( hull[i].m_orgIndex );
						for( int k = 0; k < orgpoints.Count; k++ )
						{
							if( orgpoints[k].m_orgIndex == hull[i].m_orgIndex )
							{
								orgpoints[k].m_orgIndex = -1; // invalidate...
								break;
							}
						}
					}

					// are there rejected vertices?
					bool reject_merge = false;



					for( int i = 0; i < orgpoints.Count; i++ )
					{
						if( orgpoints[i].m_orgIndex == -1 )
							continue; // this is in the hull...
									  // this vertex is rejected -- is anybody else using this vertex?
						for( int j = 0; j < tmpFaces.Count; j++ )
						{

							btConvexPolyhedron.btFace face = tmpFaces[j];
							// is this a face of the current coplanar group?
							bool is_in_current_group = false;
							for( int k = 0; k < coplanarFaceGroup.Count; k++ )
							{
								if( coplanarFaceGroup[k] == j )
								{
									is_in_current_group = true;
									break;
								}
							}
							if( is_in_current_group ) // ignore this face...
								continue;
							// does this face use this rejected vertex?
							for( int v = 0; v < face.m_indices.Count; v++ )
							{
								if( face.m_indices[v] == orgpoints[i].m_orgIndex )
								{
									// this rejected vertex is used in another face -- reject merge
									reject_merge = true;
									break;
								}
							}
							if( reject_merge )
								break;
						}
						if( reject_merge )
							break;
					}

					if( !reject_merge )
					{
						// do this merge!
						did_merge = true;
						m_polyhedron.m_faces.Add( combinedFace );
					}
				}
				if( !did_merge )
				{
					for( int i = 0; i < coplanarFaceGroup.Count; i++ )
					{
						btConvexPolyhedron.btFace face = tmpFaces[coplanarFaceGroup[i]];
						m_polyhedron.m_faces.Add( face );
					}

				}



			}

			m_polyhedron.initialize();

			return true;
		}

		public override void localGetSupportingVertexWithoutMargin( ref btVector3 vec0, out btVector3 result )
		{
			result = btVector3.Zero;
			int i;
			double maxDot = double.MinValue;

			btVector3 vec = vec0;
			double lenSqr = vec.length2();
			if( lenSqr < (double)( 0.0001 ) )
			{
				vec.setValue( 1, 0, 0 );
			}
			else
			{
				double rlen = (double)( 1.0 ) / btScalar.btSqrt( lenSqr );
				vec.Mult( rlen, out vec );
			}

			double newDot;

			btVector3[] temp = new btVector3[128];
			for( int k = 0; k < getNumVertices(); k += 128 )
			{
				int inner_count = btScalar.btMin( getNumVertices() - k, 128 );
				for( i = 0; i < inner_count; i++ )
					getVertex( i, out temp[i] );
				i = (int)vec.maxDot( temp, inner_count, out newDot );
				if( newDot > maxDot )
				{
					maxDot = newDot;
					result = temp[i];
				}
			}

		}



		public override void batchedUnitVectorGetSupportingVertexWithoutMargin( btVector3[] vectors, btVector3[] supportVerticesOut, int numVectors )
		{
			int i;

			double newDot;

			for( i = 0; i < numVectors; i++ )
			{
				supportVerticesOut[i][3] = btScalar.BT_MIN_FLOAT;
			}

			for( int j = 0; j < numVectors; j++ )
			{
				btVector3 vec = vectors[j];

				btVector3[] temp = new btVector3[128];
				for( int k = 0; k < getNumVertices(); k += 128 )
				{
					int inner_count = btScalar.btMin( getNumVertices() - k, 128 );
					for( i = 0; i < inner_count; i++ )
						getVertex( i, out temp[i] );
					i = (int)vec.maxDot( temp, inner_count, out newDot );
					if( newDot > supportVerticesOut[j][3] )
					{
						supportVerticesOut[j] = temp[i];
						supportVerticesOut[j][3] = newDot;
					}
				}
			}
		}


		public override void calculateLocalInertia( double mass, out btVector3 inertia )
		{
			//not yet, return box inertia

			double margin = getMargin();

			btVector3 aabbMin, aabbMax;
			getAabb( ref btTransform.Identity, out aabbMin, out aabbMax );
			btVector3 tmp;
			aabbMax.Sub( ref aabbMin, out tmp );
			btVector3 halfExtents;// = ( aabbMax - aabbMin ) * (double)( 0.5 );
			tmp.Mult( btScalar.BT_HALF, out halfExtents );

			double lx = btScalar.BT_TWO * ( halfExtents.x + margin );
			double ly = btScalar.BT_TWO * ( halfExtents.y + margin );
			double lz = btScalar.BT_TWO * ( halfExtents.z + margin );
			double x2 = lx * lx;
			double y2 = ly * ly;
			double z2 = lz * lz;
			double scaledmass = mass * (double)( 0.08333333 );
			tmp = new btVector3( y2 + z2, x2 + z2, x2 + y2 );
			tmp.Mult( scaledmass, out inertia );
			//inertia = scaledmass * ( new btVector3( y2 + z2, x2 + z2, x2 + y2 ) );
		}

	}


	///The btPolyhedralConvexAabbCachingShape adds aabb caching to the btPolyhedralConvexShape
	public abstract class btPolyhedralConvexAabbCachingShape : btPolyhedralConvexShape
	{

		btVector3 m_localAabbMin;
		btVector3 m_localAabbMax;
		bool m_isLocalAabbValid;


		protected void setCachedLocalAabb( ref btVector3 aabbMin, ref btVector3 aabbMax )
		{
			m_isLocalAabbValid = true;
			m_localAabbMin = aabbMin;
			m_localAabbMax = aabbMax;
		}

		protected void getCachedLocalAabb( ref btVector3 aabbMin, ref btVector3 aabbMax )
		{
			Debug.Assert( m_isLocalAabbValid );
			aabbMin = m_localAabbMin;
			aabbMax = m_localAabbMax;
		}


		public void getNonvirtualAabb( ref btTransform trans, out btVector3 aabbMin, out btVector3 aabbMax, double margin )
		{

			//lazy evaluation of local aabb
			Debug.Assert( m_isLocalAabbValid );
			btAabbUtil.btTransformAabb( ref m_localAabbMin, ref m_localAabbMax, margin, ref trans, out aabbMin, out aabbMax );
		}


		public override void setLocalScaling( ref btVector3 scaling )
		{
			base.setLocalScaling( ref scaling );
			recalcLocalAabb();
		}

		public btPolyhedralConvexAabbCachingShape()
		{
			//btPolyhedralConvexShape(),
			m_localAabbMin = new btVector3( 1, 1, 1 );
			m_localAabbMax = new btVector3( -1, -1, -1 );
			m_isLocalAabbValid = ( false );
		}

		public override void getAabb( ref btTransform trans, out btVector3 aabbMin, out btVector3 aabbMax )
		{
			getNonvirtualAabb( ref trans, out aabbMin, out aabbMax, getMargin() );
		}


		static btVector3[] _directions = new btVector3[]{
			new btVector3( 1.0,  0.0,   0.0),
			new btVector3( 0.0,  1.0,   0.0),
			new btVector3( 0.0,  0.0,  1.0),
			new btVector3( -1.0, 0.0,   0.0),
			new btVector3( 0.0, -1.0,   0.0),
			new btVector3( 0.0,  0.0, -1.0)
		};
		public void recalcLocalAabb()
		{
			m_isLocalAabbValid = true;

			btVector3[] _supporting = new btVector3[6];

			batchedUnitVectorGetSupportingVertexWithoutMargin( _directions, _supporting, 6 );

			for( int i = 0; i < 3; ++i )
			{
				m_localAabbMax[i] = _supporting[i][i] + m_collisionMargin;
				m_localAabbMin[i] = _supporting[i + 3][i] - m_collisionMargin;
			}

		}
	}
}


