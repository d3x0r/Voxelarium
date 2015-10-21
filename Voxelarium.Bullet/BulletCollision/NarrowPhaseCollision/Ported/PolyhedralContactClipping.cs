//#define ONLY_REPORT_DEEPEST_POINT
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
using Bullet.Collision.Shapes;
using Bullet.LinearMath;
using Bullet.Types;
///This file was written by Erwin Coumans
namespace Bullet.Collision.NarrowPhase
{


	public class btVertexArray : btList<btVector3>
	{
	}

	// Clips a face to the back of a plane
	public class btPolyhedralContactClipping
	{

		static int gExpectedNbTests = 0;
		static int gActualNbTests = 0;
		static bool gUseInternalObject = true;

		// Clips a face to the back of a plane
		public static void clipFace( btVertexArray pVtxIn, btVertexArray ppVtxOut, ref btVector3 planeNormalWS, double planeEqWS )
		{

			int ve;
			double ds, de;
			int numVerts = pVtxIn.Count;
			if( numVerts < 2 )
				return;

			btVector3 firstVertex = pVtxIn[pVtxIn.Count - 1];
			btVector3 endVertex = pVtxIn[0];

			ds = planeNormalWS.dot( ref firstVertex ) + planeEqWS;

			for( ve = 0; ve < numVerts; ve++ )
			{
				endVertex = pVtxIn[ve];

				de = planeNormalWS.dot( ref endVertex ) + planeEqWS;

				if( ds < 0 )
				{
					if( de < 0 )
					{
						// Start < 0, end < 0, so output endVertex
						ppVtxOut.Add( endVertex );
					}
					else
					{
						// Start < 0, end >= 0, so output intersection
						btVector3 tmp;
						firstVertex.lerp( ref endVertex, (double)( ds * 1 / ( ds - de ) ), out tmp );
						ppVtxOut.Add( ref tmp );
					}
				}
				else
				{
					if( de < 0 )
					{
						btVector3 tmp;
						firstVertex.lerp( ref endVertex, (double)( ds * 1 / ( ds - de ) ), out tmp );
						// Start >= 0, end < 0 so output intersection and end
						ppVtxOut.Add( ref tmp );
						ppVtxOut.Add( ref endVertex );
					}
				}
				firstVertex = endVertex;
				ds = de;
			}
		}


		static bool TestSepAxis( btConvexPolyhedron hullA, btConvexPolyhedron hullB, btITransform transA, btITransform transB, ref btVector3 sep_axis, out double depth, out btVector3 witnessPointA, out btVector3 witnessPointB )
		{
			double Min0, Max0;
			double Min1, Max1;
			btVector3 witnesPtMinA, witnesPtMaxA;
			btVector3 witnesPtMinB, witnesPtMaxB;

			hullA.project( transA, ref sep_axis, out Min0, out Max0, out witnesPtMinA, out witnesPtMaxA );
			hullB.project( transB, ref sep_axis, out Min1, out Max1, out witnesPtMinB, out witnesPtMaxB );

			if( Max0 < Min1 || Max1 < Min0 )
			{
				depth = 0;
				witnessPointB = witnessPointA = btVector3.Zero;
				return false;
			}
			double d0 = Max0 - Min1;
			Debug.Assert( d0 >= 0.0f );
			double d1 = Max1 - Min0;
			Debug.Assert( d1 >= 0.0f );
			if( d0 < d1 )
			{
				depth = d0;
				witnessPointA = witnesPtMaxA;
				witnessPointB = witnesPtMinB;

			}
			else
			{
				depth = d1;
				witnessPointA = witnesPtMinA;
				witnessPointB = witnesPtMaxB;
			}

			return true;
		}

		static int gActualSATPairTests = 0;


#if TEST_INTERNAL_OBJECTS

inline void BoxSupport(double extents[3], double sv[3], double p[3])
{
	// This version is ~11.000 cycles (4%) faster overall in one of the tests.
//	IR(p[0]) = IR(extents[0])|(IR(sv[0])&SIGN_BITMASK);
//	IR(p[1]) = IR(extents[1])|(IR(sv[1])&SIGN_BITMASK);
//	IR(p[2]) = IR(extents[2])|(IR(sv[2])SIGN_BITMASK);
	p = sv[0] < 0.0f ? -extents[0] : extents[0];
	p[1] = sv[1] < 0.0f ? -extents[1] : extents[1];
	p[2] = sv[2] < 0.0f ? -extents[2] : extents[2];
}

void InverseTransformPoint3x3(ref btVector3 out, ref btVector3 in, ref btTransform tr)
{
	btMatrix3x3 rot = tr.getBasis();
	ref btVector3 r0 = rot;
	ref btVector3 r1 = rot[1];
	ref btVector3 r2 = rot[2];

	double x = r0.x*in.x + r1.x*in.y + r2.x*in.z;
	double y = r0.y*in.x + r1.y*in.y + r2.y*in.z;
	double z = r0.z*in.x + r1.z*in.y + r2.z*in.z;

	out.setValue(x, y, z);
}

 bool TestInternalObjects( ref btTransform trans0, ref btTransform trans1, ref btVector3 delta_c, ref btVector3 axis, btConvexPolyhedron& convex0, btConvexPolyhedron& convex1, double dmin)
{
	double dp = delta_c.dot(axis);

	btVector3 localAxis0;
	InverseTransformPoint3x3(localAxis0, axis,trans0);
	btVector3 localAxis1;
	InverseTransformPoint3x3(localAxis1, axis,trans1);

	double p0[3];
	BoxSupport(convex0.m_extents, localAxis0, p0);
	double p1[3];
	BoxSupport(convex1.m_extents, localAxis1, p1);

	double Radius0 = p0[0]*localAxis0.x + p0[1]*localAxis0.y + p0[2]*localAxis0.z;
	double Radius1 = p1[0]*localAxis1.x + p1[1]*localAxis1.y + p1[2]*localAxis1.z;

	double MinRadius = Radius0>convex0.m_radius ? Radius0 : convex0.m_radius;
	double MaxRadius = Radius1>convex1.m_radius ? Radius1 : convex1.m_radius;

	double MinMaxRadius = MaxRadius + MinRadius;
	double d0 = MinMaxRadius + dp;
	double d1 = MinMaxRadius - dp;

	double depth = d0<d1 ? d0:d1;
	if(depth>dmin)
		return false;
	return true;
}
#endif //TEST_INTERNAL_OBJECTS



		public static void btSegmentsClosestPoints(
		   out btVector3 ptsVector,
		   out btVector3 offsetA,
		   out btVector3 offsetB,
		   out double tA, out double tB,
		   ref btVector3 translation,
		   ref btVector3 dirA, double hlenA,
		   ref btVector3 dirB, double hlenB )
		{
			// compute the parameters of the closest points on each line segment

			double dirA_dot_dirB = btVector3.btDot( ref dirA, ref dirB );
			double dirA_dot_trans = btVector3.btDot( ref dirA, ref translation );
			double dirB_dot_trans = btVector3.btDot( ref dirB, ref translation );

			double denom = 1.0f - dirA_dot_dirB * dirA_dot_dirB;

			if( denom == 0.0f )
			{
				tA = 0.0f;
			}
			else
			{
				tA = ( dirA_dot_trans - dirB_dot_trans * dirA_dot_dirB ) / denom;
				if( tA < -hlenA )
					tA = -hlenA;
				else if( tA > hlenA )
					tA = hlenA;
			}

			tB = tA * dirA_dot_dirB - dirB_dot_trans;

			if( tB < -hlenB )
			{
				tB = -hlenB;
				tA = tB * dirA_dot_dirB + dirA_dot_trans;

				if( tA < -hlenA )
					tA = -hlenA;
				else if( tA > hlenA )
					tA = hlenA;
			}
			else if( tB > hlenB )
			{
				tB = hlenB;
				tA = tB * dirA_dot_dirB + dirA_dot_trans;

				if( tA < -hlenA )
					tA = -hlenA;
				else if( tA > hlenA )
					tA = hlenA;
			}

			// compute the closest points relative to segment centers.

			dirA.Mult( tA, out offsetA );
			dirB.Mult( tB, out offsetB );
			//offsetA = dirA * tA;
			//offsetB = dirB * tB;
			btVector3 tmp;
			translation.Sub( ref offsetA, out tmp );
			tmp.Add( ref offsetB, out ptsVector );
			//ptsVector = translation - offsetA + offsetB;
		}



		public static bool findSeparatingAxis( btConvexPolyhedron hullA, btConvexPolyhedron hullB
			, btITransform transA, btITransform transB, out btVector3 sep
			, btDiscreteCollisionDetectorInterface.Result resultOut )
		{
			sep = btVector3.Zero;
			gActualSATPairTests++;

			//#if TEST_INTERNAL_OBJECTS
			btVector3 c0; transA.Apply( ref hullA.m_localCenter, out c0 );
			btVector3 c1; transB.Apply( ref hullB.m_localCenter, out c1 );
			btVector3 DeltaC2; c0.Sub( ref c1, out DeltaC2 );
			//#endif

			double dmin = btScalar.BT_MAX_FLOAT;
			int curPlaneTests = 0;

			int numFacesA = hullA.m_faces.Count;
			// Test normals from hullA
			for( int i = 0; i < numFacesA; i++ )
			{
				btVector3 Normal = new btVector3( hullA.m_faces[i].m_plane[0], hullA.m_faces[i].m_plane[1], hullA.m_faces[i].m_plane[2] );
				btVector3 faceANormalWS; transA.getBasis().Apply( ref Normal, out faceANormalWS );
				if( DeltaC2.dot( ref faceANormalWS ) < 0 )
					faceANormalWS.Mult( -1, out faceANormalWS );

				curPlaneTests++;
#if TEST_INTERNAL_OBJECTS
		gExpectedNbTests++;
		if(gUseInternalObject && !TestInternalObjects(transA,transB, DeltaC2, faceANormalWS, hullA, hullB, dmin))
			continue;
		gActualNbTests++;
#endif

				double d;
				btVector3 wA, wB;
				if( !TestSepAxis( hullA, hullB, transA, transB, ref faceANormalWS, out d, out wA, out wB ) )
				{
					return false;
				}

				if( d < dmin )
				{
					dmin = d;
					sep = faceANormalWS;
				}
			}

			int numFacesB = hullB.m_faces.Count;
			// Test normals from hullB
			for( int i = 0; i < numFacesB; i++ )
			{
				btVector3 Normal = new btVector3( hullB.m_faces[i].m_plane[0], hullB.m_faces[i].m_plane[1], hullB.m_faces[i].m_plane[2] );
				btVector3 WorldNormal; transB.getBasis().Apply( ref Normal, out WorldNormal );
				if( DeltaC2.dot( ref WorldNormal ) < 0 )
					WorldNormal.Mult( -1, out WorldNormal );

				curPlaneTests++;
#if TEST_INTERNAL_OBJECTS
		gExpectedNbTests++;
		if(gUseInternalObject && !TestInternalObjects(transA,transB,DeltaC2, WorldNormal, hullA, hullB, dmin))
			continue;
		gActualNbTests++;
#endif

				double d;
				btVector3 wA, wB;
				if( !TestSepAxis( hullA, hullB, transA, transB, ref WorldNormal, out d, out wA, out wB ) )
				{
					return false;
				}

				if( d < dmin )
				{
					dmin = d;
					sep = WorldNormal;
				}
			}

			//btVector3 edgeAstart, edgeAend, edgeBstart, edgeBend;
			int edgeA = -1;
			int edgeB = -1;
			btVector3 worldEdgeA = btVector3.Zero;
			btVector3 worldEdgeB = btVector3.Zero;
			btVector3 witnessPointA, witnessPointB;
			witnessPointA = witnessPointB = btVector3.Zero;


			int curEdgeEdge = 0;
			// Test edges
			for( int e0 = 0; e0 < hullA.m_uniqueEdges.Count; e0++ )
			{
				btVector3 edge0 = hullA.m_uniqueEdges[e0];
				btVector3 WorldEdge0; transA.getBasis().Apply( ref edge0, out WorldEdge0 );
				for( int e1 = 0; e1 < hullB.m_uniqueEdges.Count; e1++ )
				{
					btVector3 edge1 = hullB.m_uniqueEdges[e1];
					btVector3 WorldEdge1; transB.getBasis().Apply( ref edge1, out WorldEdge1 );

					btVector3 Cross; WorldEdge0.cross( ref WorldEdge1, out Cross );
					curEdgeEdge++;
					if( !Cross.IsAlmostZero() )
					{
						Cross = Cross.normalize();
						if( DeltaC2.dot( ref Cross ) < 0 )
							Cross.Invert( out Cross );


#if TEST_INTERNAL_OBJECTS
				gExpectedNbTests++;
				if(gUseInternalObject && !TestInternalObjects(transA,transB,DeltaC2, Cross, hullA, hullB, dmin))
					continue;
				gActualNbTests++;
#endif

						double dist;
						btVector3 wA, wB;
						if( !TestSepAxis( hullA, hullB, transA, transB, ref Cross, out dist, out wA, out wB ) )
						{
							return false;
						}

						if( dist < dmin )
						{
							dmin = dist;
							sep = Cross;
							edgeA = e0;
							edgeB = e1;
							worldEdgeA = WorldEdge0;
							worldEdgeB = WorldEdge1;
							witnessPointA = wA;
							witnessPointB = wB;
						}
					}
				}
			}

			if( edgeA >= 0 && edgeB >= 0 )
			{
				//		Console.WriteLine("edge-edge\n");
				//add an edge-edge contact

				btVector3 ptsVector;
				btVector3 offsetA;
				btVector3 offsetB;
				double tA;
				double tB;

				btVector3 translation; witnessPointB.Sub( ref witnessPointA, out translation );

				btVector3 dirA = worldEdgeA;
				btVector3 dirB = worldEdgeB;

				double hlenB = btScalar.BT_LARGE_FLOAT;
				double hlenA = btScalar.BT_LARGE_FLOAT;

				btSegmentsClosestPoints( out ptsVector, out offsetA, out offsetB, out tA, out tB,
					ref translation,
					ref dirA, hlenA,
					ref dirB, hlenB );

				double nlSqrt = ptsVector.length2();
				if( nlSqrt > btScalar.SIMD_EPSILON )
				{
					double nl = btScalar.btSqrt( nlSqrt );
					ptsVector.Mult( 1 / nl, out ptsVector );
					if( ptsVector.dot( ref DeltaC2 ) < 0 )
					{
						ptsVector.Invert( out ptsVector );
					}
					btVector3 ptOnB; witnessPointB.Add( ref offsetB, out ptOnB );
					double distance = nl;
					resultOut.addContactPoint( ref ptsVector, ref ptOnB, -distance );
				}
			}


			if( ( DeltaC2.dot( ref sep ) ) < 0.0f )
				sep.Invert( out sep );

			return true;
		}

		public static void clipFaceAgainstHull( ref btVector3 separatingNormal
				, btConvexPolyhedron hullA, btITransform transA
				, btVertexArray worldVertsB1, double minDist, double maxDist
				, btDiscreteCollisionDetectorInterface.Result resultOut )
		{
			btVertexArray worldVertsB2 = new btVertexArray();
			btVertexArray pVtxIn = worldVertsB1;
			btVertexArray pVtxOut = worldVertsB2;
			pVtxOut.Capacity = ( pVtxIn.Count );

			int closestFaceA = -1;
			{
				double dmin = btScalar.BT_MAX_FLOAT;
				for( int face = 0; face < hullA.m_faces.Count; face++ )
				{
					btVector3 Normal = new btVector3( hullA.m_faces[face].m_plane[0], hullA.m_faces[face].m_plane[1], hullA.m_faces[face].m_plane[2] );
					btVector3 faceANormalWS; transA.getBasis().Apply( ref Normal, out faceANormalWS );

					double d = faceANormalWS.dot( ref separatingNormal );
					if( d < dmin )
					{
						dmin = d;
						closestFaceA = face;
					}
				}
			}
			if( closestFaceA < 0 )
				return;

			btConvexPolyhedron.btFace polyA = hullA.m_faces[closestFaceA];

			// clip polygon to back of planes of all faces of hull A that are adjacent to witness face
			int numVerticesA = polyA.m_indices.Count;
			for( int e0 = 0; e0 < numVerticesA; e0++ )
			{
				btVector3 a = hullA.m_vertices[polyA.m_indices[e0]];
				btVector3 b = hullA.m_vertices[polyA.m_indices[( e0 + 1 ) % numVerticesA]];
				btVector3 edge0; a.Sub( ref b, out edge0 );
				btVector3 WorldEdge0; transA.getBasis().Apply( ref edge0, out WorldEdge0 );
				btVector3 tmp = new btVector3( polyA.m_plane[0], polyA.m_plane[1], polyA.m_plane[2] );
				btVector3 worldPlaneAnormal1; transA.getBasis().Apply( ref tmp, out worldPlaneAnormal1 );

				btVector3 planeNormalWS1; WorldEdge0.cross( ref worldPlaneAnormal1, out planeNormalWS1 );//.cross(WorldEdge0);
				planeNormalWS1.Invert( out planeNormalWS1 );
				btVector3 worldA1; transA.Apply( ref a, out worldA1 );
				double planeEqWS1 = -worldA1.dot( ref planeNormalWS1 );

				//int otherFace=0;
#if BLA1
		int otherFace = polyA.m_connectedFaces[e0];
		btVector3 localPlaneNormal (hullA.m_faces[otherFace].m_plane[0],hullA.m_faces[otherFace].m_plane[1],hullA.m_faces[otherFace].m_plane[2]);
		double localPlaneEq = hullA.m_faces[otherFace].m_plane[3];

		btVector3 planeNormalWS = transA.getBasis()*localPlaneNormal;
		double planeEqWS=localPlaneEq-planeNormalWS.dot(transA.getOrigin());
#else
				btVector3 planeNormalWS = planeNormalWS1;
				double planeEqWS = planeEqWS1;

#endif
				//clip face

				clipFace( pVtxIn, pVtxOut, ref planeNormalWS, planeEqWS );
				btScalar.btSwap( ref pVtxIn, ref pVtxOut );
				pVtxOut.Capacity = ( 0 );
			}



			//btVector3 point;


			// only keep points that are behind the witness face
			{
				btVector3 localPlaneNormal = new btVector3( polyA.m_plane[0], polyA.m_plane[1], polyA.m_plane[2]);
				double localPlaneEq = polyA.m_plane[3];
				btVector3 planeNormalWS; transA.getBasis().Apply( ref localPlaneNormal, out planeNormalWS );
				btVector3 origin;
				transA.getOrigin( out origin );
                double planeEqWS = localPlaneEq - planeNormalWS.dot( ref origin );
				for( int i = 0; i < pVtxIn.Count; i++ )
				{
					//btVector3 vtx = pVtxIn.at( i );
					double depth = planeNormalWS.dot( ref pVtxIn.InternalArray[i] ) + planeEqWS;
					if( depth <= minDist )
					{
						//				Console.WriteLine("clamped: depth=%f to minDist=%f\n",depth,minDist);
						depth = minDist;
					}

					if( depth <= maxDist )
					{
						//btVector3 point = pVtxIn.at( i );
#if ONLY_REPORT_DEEPEST_POINT
				curMaxDist = depth;
#else
#if false
				if (depth<-3)
				{
					Console.WriteLine("error in btPolyhedralContactClipping depth = %f\n", depth);
					Console.WriteLine("likely wrong separatingNormal passed in\n");
				} 
#endif
						resultOut.addContactPoint( ref separatingNormal, ref pVtxIn.InternalArray[i], depth );
#endif
					}
				}
			}
#if ONLY_REPORT_DEEPEST_POINT
	if (curMaxDist<maxDist)
	{
		resultOut.addContactPoint(separatingNormal,point,curMaxDist);
	}
#endif //ONLY_REPORT_DEEPEST_POINT

		}





		public static void clipHullAgainstHull( ref btVector3 separatingNormal1, btConvexPolyhedron hullA
			, btConvexPolyhedron hullB
			, btITransform transA, btITransform transB, double minDist, double maxDist, btDiscreteCollisionDetectorInterface.Result resultOut)
		{

			btVector3 separatingNormal; separatingNormal1.normalized( out separatingNormal);
			//	btVector3 c0 = transA * hullA.m_localCenter;
			//	btVector3 c1 = transB * hullB.m_localCenter;
			//btVector3 DeltaC2 = c0 - c1;



			int closestFaceB = -1;
			double dmax = btScalar.BT_MIN_FLOAT;
			{
				for( int face = 0; face < hullB.m_faces.Count; face++ )
				{
					btVector3 Normal = new btVector3( hullB.m_faces[face].m_plane[0], hullB.m_faces[face].m_plane[1], hullB.m_faces[face].m_plane[2] );
					btVector3 WorldNormal;  transB.getBasis().Apply( ref Normal, out WorldNormal );
					double d = WorldNormal.dot( ref separatingNormal );
					if( d > dmax )
					{
						dmax = d;
						closestFaceB = face;
					}
				}
			}
			btVertexArray worldVertsB1 = new btVertexArray();
			{
				btConvexPolyhedron.btFace polyB = hullB.m_faces[closestFaceB];
				int numVertices = polyB.m_indices.Count;
				for( int e0 = 0; e0 < numVertices; e0++ )
				{
					btVector3 b = hullB.m_vertices[polyB.m_indices[e0]];
					btVector3 tmp;
					transB.Apply( ref b, out tmp );
					worldVertsB1.Add( ref tmp );
				}
			}


			if( closestFaceB >= 0 )
				clipFaceAgainstHull( ref separatingNormal, hullA, transA, worldVertsB1, minDist, maxDist, resultOut );

		}

	};

}
