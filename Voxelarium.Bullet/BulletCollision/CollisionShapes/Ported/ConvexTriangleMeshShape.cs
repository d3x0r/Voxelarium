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
using System.Diagnostics;

namespace Bullet.Collision.Shapes
{


	/// The btConvexTriangleMeshShape is a convex hull of a triangle mesh, but the performance is not as good as btConvexHullShape.
	/// A small benefit of this class is that it uses the btStridingMeshInterface, so you can avoid the duplication of the triangle mesh data. Nevertheless, most users should use the much better performing btConvexHullShape instead.
	internal class btConvexTriangleMeshShape<Index, Data> : btPolyhedralConvexAabbCachingShape
	{

		btStridingMeshInterface<Index, Data> m_stridingMesh;


		btStridingMeshInterface<Index, Data> getMeshInterface()
		{
			return m_stridingMesh;
		}


		//debugging
		public override string ToString() { return "ConvexTrimesh"; }

		/*
		virtual btVector3 localGetSupportingVertex( ref btVector3 vec )const;
		virtual btVector3 localGetSupportingVertexWithoutMargin( ref btVector3 vec )const;
		virtual void batchedUnitVectorGetSupportingVertexWithoutMargin( btVector3* vectors, btVector3* supportVerticesOut, int numVectors );
		virtual int	getNumVertices();
		virtual int getNumEdges();
		virtual void getEdge(int i,ref btVector3 pa,ref btVector3 pb);
		virtual void getVertex(int i,ref btVector3 vtx);
		virtual int	getNumPlanes();
		virtual void getPlane(ref btVector3 planeNormal,ref btVector3 planeSupport,int i );
		virtual	bool isInside(ref btVector3 pt,double tolerance);


		virtual void	setLocalScaling(ref btVector3 scaling);
		virtual ref btVector3 getLocalScaling();
		*/

		///computes the exact moment of inertia and the transform from the coordinate system defined by the principal axes of the moment of inertia
		///and the center of mass to the current coordinate system. A mass of 1 is assumed, for other masses just multiply the computed "inertia"
		///by the mass. The resulting transform "principal" has to be applied inversely to the mesh in order for the local coordinate system of the
		///shape to be centered at the center of mass and to coincide with the principal axes. This also necessitates a correction of the world transform
		///of the collision object by the principal transform. This method also computes the volume of the convex mesh.
		//void calculatePrincipalAxisTransform(ref btTransform principal, ref btVector3 inertia, double volume);

		public btConvexTriangleMeshShape( btStridingMeshInterface<Index, Data> meshInterface, bool calcAabb = true )
					: base()
		{
			m_stridingMesh = ( meshInterface );
			m_shapeType = BroadphaseNativeTypes.CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE;
			if( calcAabb )
				recalcLocalAabb();
		}




		///It's not nice to have all this virtual function overhead, so perhaps we can also gather the points once
		///but then we are duplicating
		internal class LocalSupportVertexCallback : btInternalTriangleIndexCallback
		{

			btVector3 m_supportVertexLocal;

			internal double m_maxDot;
			internal btVector3 m_supportVecLocal;

			internal LocalSupportVertexCallback( ref btVector3 supportVecLocal )
			{
				m_supportVertexLocal = btVector3.Zero;
				m_maxDot = btScalar.BT_MIN_FLOAT;
				m_supportVecLocal = ( supportVecLocal );
			}

			internal override void internalProcessTriangleIndex( btVector3[] triangle, int partId, int triangleIndex )
			{
				//(void)triangleIndex;
				//(void)partId;

				for( int i = 0; i < 3; i++ )
				{
					double dot = m_supportVecLocal.dot( triangle[i] );
					if( dot > m_maxDot )
					{
						m_maxDot = dot;
						m_supportVertexLocal = triangle[i];
					}
				}
			}

			internal void GetSupportVertexLocal( out btVector3 result )
			{
				result = m_supportVertexLocal;
			}

		};




		public override void localGetSupportingVertexWithoutMargin( ref btVector3 vec0, out btVector3 result )
		{
			btVector3 supVec = btVector3.Zero;// ( btScalar.BT_ZERO, btScalar.BT_ZERO, btScalar.BT_ZERO);

			btVector3 vec = vec0;
			double lenSqr = vec.length2();
			if( lenSqr < (double)( 0.0001 ) )
			{
				vec.setValue( 1, 0, 0 );
			}
			else
			{
				double rlen = btScalar.BT_ONE / btScalar.btSqrt( lenSqr );
				vec *= rlen;
			}

			LocalSupportVertexCallback supportCallback = new LocalSupportVertexCallback( ref vec );
			btVector3 aabbMax = btVector3.Max;
			btVector3 aabbMin = btVector3.Min;
			m_stridingMesh.InternalProcessAllTriangles( supportCallback, ref aabbMin, ref aabbMax );
			 supportCallback.GetSupportVertexLocal( out result );
		}

		public override void batchedUnitVectorGetSupportingVertexWithoutMargin( btVector3[] vectors, btVector3[] supportVerticesOut, int numVectors )
		{
			//use 'w' component of supportVerticesOut?
			{
				for( int i = 0; i < numVectors; i++ )
				{
					supportVerticesOut[i][3] = btScalar.BT_MIN_FLOAT;
				}
			}

			///@todo: could do the batch inside the callback!


			for( int j = 0; j < numVectors; j++ )
			{
				btVector3 vec = vectors[j];
				LocalSupportVertexCallback supportCallback = new LocalSupportVertexCallback( ref vec );
				btVector3 aabbMax = btVector3.Max;
				btVector3 aabbMin = btVector3.Min;
				m_stridingMesh.InternalProcessAllTriangles( supportCallback, ref aabbMin, ref aabbMax );
				supportCallback.GetSupportVertexLocal( out supportVerticesOut[j] );
			}

		}



		btVector3 localGetSupportingVertex( ref btVector3 vec )
		{
			btVector3 supVertex; localGetSupportingVertexWithoutMargin( ref vec, out supVertex );

			if( getMargin() != btScalar.BT_ZERO )
			{
				btVector3 vecnorm = vec;
				if( vecnorm.length2() < ( btScalar.SIMD_EPSILON * btScalar.SIMD_EPSILON ) )
				{
					vecnorm = new btVector3( btScalar.BT_NEG_ONE );
				}
				vecnorm.normalize();
				supVertex.AddScale( ref vecnorm, getMargin(), out supVertex );
				//supVertex += getMargin() * vecnorm;
			}
			return supVertex;
		}









		//currently just for debugging (drawing), perhaps future support for algebraic continuous collision detection
		//Please note that you can debug-draw btConvexTriangleMeshShape with the Raytracer Demo
		public override int getNumVertices()
		{
			//cache this?
			return 0;

		}

		public override int getNumEdges()
		{
			return 0;
		}

		public override void getEdge( int edge, out btVector3 start, out btVector3 end )
		{
			Debug.Assert( false );
			start = btVector3.Zero;
			end = btVector3.Zero;
		}

		public override void getVertex( int vert, out btVector3 result )
		{
			Debug.Assert( false );
			result = btVector3.Zero;
		}

		public override int getNumPlanes()
		{
			return 0;
		}

		public override void getPlane( out btVector3 a, out btVector3 b, int side )
		{
			Debug.Assert( false );
			a = btVector3.Zero;
			b = btVector3.yAxis;
		}

		//not yet
		public override bool isInside( ref btVector3 a, double b )
		{
			Debug.Assert( false );
			return false;
		}



		public override void setLocalScaling( ref btVector3 scaling )
		{
			m_stridingMesh.setScaling( ref scaling );

			recalcLocalAabb();

		}


		public override void getLocalScaling( out btVector3 result )
		{
			m_stridingMesh.getScaling( out result );
		}

		class CenterCallback : btInternalTriangleIndexCallback
		{
			bool first;
			btVector3 ref_point;
			btVector3 sum;
			double volume;

			
			internal CenterCallback()
			{
				first = ( true );
				ref_point = btVector3.Zero;
				sum = btVector3.Zero;
				volume = ( 0 );
            }

			internal override void internalProcessTriangleIndex( btVector3[] triangle, int partId, int triangleIndex )
			{
				//(void)triangleIndex;
				//(void)partId;
				if( first )
				{
					ref_point = triangle[0];
					first = false;
				}
				else
				{
					btVector3 tmp1;
					btVector3 tmp2;
					triangle[1].Sub( ref ref_point, out tmp1 );
					triangle[2].Sub( ref ref_point, out tmp2 );
					double vol = btScalar.btFabs( ( triangle[0] - ref_point ).triple( ref tmp1, ref tmp2 ) );
					sum +=  ( ( triangle[0] + triangle[1] + triangle[2] + ref_point ) ) * ( (double)( 0.25 ) * vol );
					volume += vol;
				}
			}

			internal btVector3 getCenter()
			{
				return ( volume > 0 ) ? sum / volume : ref_point;
			}

			internal double getVolume()
			{
				return volume * (double)( 1.0 / 6 );
			}

		};

		internal class InertiaCallback : btInternalTriangleIndexCallback
		{
			btMatrix3x3 sum;
			btVector3 center;


			internal InertiaCallback( ref btVector3 center )
			{
				sum = new btMatrix3x3( 0, 0, 0, 0, 0, 0, 0, 0, 0 );
				this.center = ( center );
			}

			internal override void internalProcessTriangleIndex( btVector3[] triangle, int partId, int triangleIndex )
			{
				//(void)triangleIndex;
				//(void)partId;
				btMatrix3x3 i = btMatrix3x3.Identity;
				btVector3 a = triangle[0] - center;
				btVector3 b = triangle[1] - center;
				btVector3 c = triangle[2] - center;
				double volNeg = -btScalar.btFabs( a.triple( ref b, ref c ) ) * (double)( 1.0 / 6 );
				for( int j = 0; j < 3; j++ )
				{
					for( int k = 0; k <= j; k++ )
					{
						i[j][k] = i[k][j] = volNeg * ( (double)( 0.1 ) * ( a[j] * a[k] + b[j] * b[k] + c[j] * c[k] )
						   + (double)( 0.05 ) * ( a[j] * b[k] + a[k] * b[j] + a[j] * c[k] + a[k] * c[j] + b[j] * c[k] + b[k] * c[j] ) );
					}
				}
				double i00 = -i[0][0];
				double i11 = -i[1][1];
				double i22 = -i[2][2];
				i[0][0] = i11 + i22;
				i[1][1] = i22 + i00;
				i[2][2] = i00 + i11;
				sum.m_el0 += i.m_el0;
				sum.m_el1 += i.m_el1;
				sum.m_el2 += i.m_el2;
			}

			internal void getInertia( out btMatrix3x3 result )
			{
				result = sum;
			}

		};

#if this_is_used
		///computes the exact moment of inertia and the transform from the coordinate system defined by the principal axes of the moment of inertia
		///and the center of mass to the current coordinate system. A mass of 1 is assumed, for other masses just multiply the computed "inertia"
		///by the mass. The resulting transform "principal" has to be applied inversely to the mesh in order for the local coordinate system of the
		///shape to be centered at the center of mass and to coincide with the principal axes. This also necessitates a correction of the world transform
		///of the collision object by the principal transform. This method also computes the volume of the convex mesh.
		public virtual void calculatePrincipalAxisTransform( out btTransform principal, out btVector3 inertia, double volume )
		{

			CenterCallback centerCallback = new CenterCallback();
			btVector3 aabbMax = btVector3.Max;
			btVector3 aabbMin = btVector3.Min;
			m_stridingMesh.InternalProcessAllTriangles( centerCallback, ref aabbMin, ref aabbMax );
			btVector3 center = centerCallback.getCenter();
			principal.m_origin =  center;
			volume = centerCallback.getVolume( );

			InertiaCallback inertiaCallback = new InertiaCallback( ref center );
			m_stridingMesh.InternalProcessAllTriangles( inertiaCallback, ref aabbMin, ref aabbMax );

			btMatrix3x3 i; inertiaCallback.getInertia( out i );
			i.diagonalize( out principal.m_basis, (double)( 0.00001 ), 20 );
			inertia = new btVector3( i[0][0], i[1][1], i[2][2] );
			inertia /= volume;
		}
#endif

	};



}


