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

using System.Diagnostics;
using Bullet.Collision.BroadPhase;
using Bullet.LinearMath;

namespace Bullet.Collision.Shapes
{

	///The btBox2dShape is a box primitive around the origin, its sides axis aligned with length specified by half extents, in local shape coordinates. When used as part of a btCollisionObject or btRigidBody it will be an oriented box in world space.
	internal class btBox2dShape : btPolyhedralConvexShape
	{

		//btVector3	m_boxHalfExtents1; //use m_implicitShapeDimensions instead

		btVector3 m_centroid;
		btVector3[] m_vertices = new btVector3[4];
		btVector3[] m_normals = new btVector3[4];



		public  void getHalfExtentsWithMargin( out btVector3 result )
		{
			btVector3 halfExtents; getHalfExtentsWithoutMargin( out halfExtents );
			btVector3 margin = new btVector3( getMargin(), getMargin(), getMargin() );
			halfExtents.Add( ref halfExtents, out result );
		}

		public  void getHalfExtentsWithoutMargin( out btVector3 result )
		{
			result = m_implicitShapeDimensions;//changed in Bullet 2.63: assume the scaling and margin are included
		}


		public override void localGetSupportingVertex( ref btVector3 vec, out btVector3 result )
		{
			btVector3 halfExtents; getHalfExtentsWithoutMargin( out halfExtents );
			btVector3 margin = new btVector3( getMargin(), getMargin(), getMargin() );
			halfExtents.Add( ref margin, out halfExtents );

			result = new btVector3( btScalar.btFsels( vec.x, halfExtents.x, -halfExtents.x ),
				btScalar.btFsels( vec.y, halfExtents.y, -halfExtents.y ),
				btScalar.btFsels( vec.z, halfExtents.z, -halfExtents.z ) );
		}

		public override void localGetSupportingVertexWithoutMargin( ref btVector3 vec, out btVector3 result )
		{
			btVector3 halfExtents; getHalfExtentsWithoutMargin( out halfExtents );

			result = new btVector3( btScalar.btFsels( vec.x, halfExtents.x, -halfExtents.x ),
				btScalar.btFsels( vec.y, halfExtents.y, -halfExtents.y ),
				btScalar.btFsels( vec.z, halfExtents.z, -halfExtents.z ) );
		}

		public override void batchedUnitVectorGetSupportingVertexWithoutMargin( btVector3[] vectors, btVector3[] supportVerticesOut, int numVectors )
		{
			for( int i = 0; i < numVectors; i++ )
			{
				localGetSupportingVertexWithoutMargin( ref vectors[i], out supportVerticesOut[i] );
			}
		}


		///a btBox2dShape is a flat 2D box in the X-Y plane (Z extents are zero)
		btBox2dShape( ref btVector3 boxHalfExtents )
		{
			m_centroid = btVector3.Zero;
			m_vertices[0].setValue( -boxHalfExtents.x, -boxHalfExtents.y, 0 );
			m_vertices[1].setValue( boxHalfExtents.x, -boxHalfExtents.y, 0 );
			m_vertices[2].setValue( boxHalfExtents.x, boxHalfExtents.y, 0 );
			m_vertices[3].setValue( -boxHalfExtents.x, boxHalfExtents.y, 0 );

			m_normals[0].setValue( 0, -1, 0 );
			m_normals[1].setValue( 1, 0, 0 );
			m_normals[2].setValue( 0, 1, 0 );
			m_normals[3].setValue( -1, 0, 0 );

			double minDimension = boxHalfExtents.x;
			if( minDimension > boxHalfExtents.y )
				minDimension = boxHalfExtents.y;
			setSafeMargin( minDimension );

			m_shapeType = BroadphaseNativeTypes.BOX_2D_SHAPE_PROXYTYPE;
			btVector3 margin = new btVector3( getMargin(), getMargin(), getMargin() );
			btVector3 tmp;
			boxHalfExtents.Mult( ref m_localScaling, out tmp );
			tmp.Sub( ref margin, out m_implicitShapeDimensions );
		}

		public override void setMargin( double collisionMargin )
		{
			//correct the m_implicitShapeDimensions for the margin
			btVector3 oldMargin = new btVector3( getMargin(), getMargin(), getMargin() );
			btVector3 implicitShapeDimensionsWithMargin; m_implicitShapeDimensions.Add( ref oldMargin, out implicitShapeDimensionsWithMargin );

			base.setMargin( collisionMargin );
			btVector3 newMargin = new btVector3( getMargin(), getMargin(), getMargin() );
			implicitShapeDimensionsWithMargin.Sub( ref newMargin, out m_implicitShapeDimensions );

		}
		public override void setLocalScaling( ref btVector3 scaling )
		{
			btVector3 oldMargin = new btVector3( getMargin(), getMargin(), getMargin() );
			btVector3 implicitShapeDimensionsWithMargin; m_implicitShapeDimensions.Add( ref oldMargin, out implicitShapeDimensionsWithMargin );
			btVector3 unScaledImplicitShapeDimensionsWithMargin; implicitShapeDimensionsWithMargin.Div( ref m_localScaling, out unScaledImplicitShapeDimensionsWithMargin );

			base.setLocalScaling( ref scaling );

			btVector3 tmp;
			unScaledImplicitShapeDimensionsWithMargin.Mult( ref m_localScaling, out tmp );
			tmp.Sub( ref oldMargin, out m_implicitShapeDimensions );

		}


		int getVertexCount()
		{
			return 4;
		}

		public override int getNumVertices()
		{
			return 4;
		}

		public void getVertices( out btVector3[] result )
		{
			result = m_vertices;
		}

		public void getNormals( out btVector3[] normals )
		{
			normals = m_normals;
		}







		public override void getPlane( out btVector3 planeNormal, out btVector3 planeSupport, int i )
		{
			//this plane might not be aligned...
			btVector3 plane;
			getPlaneEquation( out plane, i );
			planeNormal = plane;// btVector3( plane.x, plane.y, plane.z );
			plane.Invert( out plane );
			localGetSupportingVertex( ref plane, out planeSupport );
		}


		public void getCentroid( out btVector3 result )
		{
			result = m_centroid;
		}

		public override int getNumPlanes()
		{
			return 1;
		}



		public override int getNumEdges()
		{
			return 4;
		}


		public override void getVertex( int i, out btVector3 vtx )
		{

			btVector3 tmp = new btVector3( ( 1 - ( i & 1 ) ) - ( i & 1 ),
						 ( 1 - ( ( i & 2 ) >> 1 ) ) - ( ( i & 2 ) >> 1 ),
						 ( 1 - ( ( i & 4 ) >> 2 ) ) - ( i & 4 ) >> 2 );
			m_implicitShapeDimensions.Mult( ref tmp, out vtx );

		}


		void getPlaneEquation( out btVector3 plane, int i )
		{
			btVector3 halfExtents; getHalfExtentsWithoutMargin( out halfExtents );

			switch( i )
			{
				case 0:
					btVector3.setValue( out plane, (double)( 1.0 ), btScalar.BT_ZERO, btScalar.BT_ZERO, -halfExtents.x );
					break;
				case 1:
					btVector3.setValue( out plane, (double)( -1.0 ), btScalar.BT_ZERO, btScalar.BT_ZERO, -halfExtents.x );
					break;
				case 2:
					btVector3.setValue( out plane, btScalar.BT_ZERO, (double)( 1.0 ), btScalar.BT_ZERO, -halfExtents.y );
					break;
				case 3:
					btVector3.setValue( out plane, btScalar.BT_ZERO, (double)( -1.0 ), btScalar.BT_ZERO, -halfExtents.y );
					break;
				case 4:
					btVector3.setValue( out plane, btScalar.BT_ZERO, btScalar.BT_ZERO, (double)( 1.0 ), -halfExtents.z );
					break;
				case 5:
					btVector3.setValue( out plane, btScalar.BT_ZERO, btScalar.BT_ZERO, (double)( -1.0 ), -halfExtents.z );
					break;
				default:
					Debug.Assert( false );
					break;
			}
			plane = btVector3.Zero;
		}


		public override void getEdge( int i, out btVector3 pa, out btVector3 pb )
		//virtual void getEdge(int i,Edge& edge)
		{
			int edgeVert0 = 0;
			int edgeVert1 = 0;

			switch( i )
			{
				case 0:
					edgeVert0 = 0;
					edgeVert1 = 1;
					break;
				case 1:
					edgeVert0 = 0;
					edgeVert1 = 2;
					break;
				case 2:
					edgeVert0 = 1;
					edgeVert1 = 3;

					break;
				case 3:
					edgeVert0 = 2;
					edgeVert1 = 3;
					break;
				case 4:
					edgeVert0 = 0;
					edgeVert1 = 4;
					break;
				case 5:
					edgeVert0 = 1;
					edgeVert1 = 5;

					break;
				case 6:
					edgeVert0 = 2;
					edgeVert1 = 6;
					break;
				case 7:
					edgeVert0 = 3;
					edgeVert1 = 7;
					break;
				case 8:
					edgeVert0 = 4;
					edgeVert1 = 5;
					break;
				case 9:
					edgeVert0 = 4;
					edgeVert1 = 6;
					break;
				case 10:
					edgeVert0 = 5;
					edgeVert1 = 7;
					break;
				case 11:
					edgeVert0 = 6;
					edgeVert1 = 7;
					break;
				default:
					Debug.Assert( false );
					break;
			}

			getVertex( edgeVert0, out pa );
			getVertex( edgeVert1, out pb );
		}


		public override bool isInside( ref btVector3 pt, double tolerance )
		{
			btVector3 halfExtents; getHalfExtentsWithoutMargin( out halfExtents );

			//double minDist = 2*tolerance;

			bool result = ( pt.x <= ( halfExtents.x + tolerance ) ) &&
							( pt.x >= ( -halfExtents.x - tolerance ) ) &&
							( pt.y <= ( halfExtents.y + tolerance ) ) &&
							( pt.y >= ( -halfExtents.y - tolerance ) ) &&
							( pt.z <= ( halfExtents.z + tolerance ) ) &&
							( pt.z >= ( -halfExtents.z - tolerance ) );

			return result;
		}


		public override string ToString()
		{
			return "Box2d";
		}

		public override  int getNumPreferredPenetrationDirections()
		{
			return 6;
		}

		public override void getPreferredPenetrationDirection( int index, out btVector3 penetrationVector )
		{
			switch( index )
			{
				case 0:
					btVector3.setValue( out penetrationVector, btScalar.BT_ONE, btScalar.BT_ZERO, btScalar.BT_ZERO );
					break;
				case 1:
					btVector3.setValue( out penetrationVector, btScalar.BT_NEG_ONE, btScalar.BT_ZERO, btScalar.BT_ZERO );
					break;
				case 2:
					btVector3.setValue( out penetrationVector, btScalar.BT_ZERO, btScalar.BT_ONE, btScalar.BT_ZERO );
					break;
				case 3:
					btVector3.setValue( out penetrationVector, btScalar.BT_ZERO, btScalar.BT_NEG_ONE, btScalar.BT_ZERO );
					break;
				case 4:
					btVector3.setValue( out penetrationVector, btScalar.BT_ZERO, btScalar.BT_ZERO, btScalar.BT_ONE );
					break;
				case 5:
					btVector3.setValue( out penetrationVector, btScalar.BT_ZERO, btScalar.BT_ZERO, btScalar.BT_NEG_ONE );
					break;
				default:
					Debug.Assert( false );
					break;
			}
			penetrationVector = btVector3.Zero;
		}

		public override void getAabb( ref btTransform t, out btVector3 aabbMin, out btVector3 aabbMax )
		{
			btAabbUtil.btTransformAabb( ref m_implicitShapeDimensions, getMargin(), ref t, out aabbMin, out aabbMax );
		}


		public override void calculateLocalInertia( double mass, out btVector3 inertia )
		{
			//double margin = btScalar.BT_ZERO;
			btVector3 halfExtents; getHalfExtentsWithMargin( out halfExtents );

			double lx = btScalar.BT_TWO * ( halfExtents.x );
			double ly = btScalar.BT_TWO * ( halfExtents.y );
			double lz = btScalar.BT_TWO * ( halfExtents.z );

			btVector3.setValue( out inertia, mass / ( (double)( 12.0 ) ) * ( ly * ly + lz * lz ),
							mass / ( (double)( 12.0 ) ) * ( lx * lx + lz * lz ),
							mass / ( (double)( 12.0 ) ) * ( lx * lx + ly * ly ) );

		}


	};

}

