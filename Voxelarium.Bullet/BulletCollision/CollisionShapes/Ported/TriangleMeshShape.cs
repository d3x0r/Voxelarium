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


	///The btTriangleMeshShape is an internal concave triangle mesh interface. Don't use this class directly, use btBvhTriangleMeshShape instead.
	internal class btTriangleMeshShape<Index, Data> : btConcaveShape
	{
		protected btVector3 m_localAabbMin;
		protected btVector3 m_localAabbMax;
		protected btStridingMeshInterface<Index, Data> m_meshInterface;

		public virtual void localGetSupportingVertexWithoutMargin( ref btVector3 vec, out btVector3 result )
		{
			Debug.Assert( false );
			localGetSupportingVertex( ref vec, out result );
		}

		btStridingMeshInterface<Index, Data> getMeshInterface()
		{
			return m_meshInterface;
		}

		void getLocalAabbMin( out btVector3 result )
		{
			result = m_localAabbMin;
		}
		void getLocalAabbMax( out btVector3 result )
		{
			result = m_localAabbMax;
		}



		//debugging
		public override string ToString() { return "TRIANGLEMESH"; }

		///btTriangleMeshShape constructor has been disabled/protected, so that users will not mistakenly use this class.
		///Don't use btTriangleMeshShape but use btBvhTriangleMeshShape instead!
		protected btTriangleMeshShape( btStridingMeshInterface<Index, Data> meshInterface )
		{
			m_meshInterface = ( meshInterface );
			m_shapeType = BroadphaseNativeTypes.TRIANGLE_MESH_SHAPE_PROXYTYPE;
			if( meshInterface.hasPremadeAabb() )
			{
				meshInterface.getPremadeAabb( out m_localAabbMin, out m_localAabbMax );
			}
			else
			{
				recalcLocalAabb();
			}
		}



		public override void getAabb( btITransform trans, out btVector3 aabbMin, out btVector3 aabbMax )
		{
			btVector3 localHalfExtents; btVector3.getHalfExtent( ref m_localAabbMin, ref m_localAabbMax, out localHalfExtents );
			localHalfExtents += new btVector3( getMargin(), getMargin(), getMargin() );
			btVector3 localCenter; btVector3.getCenter( ref m_localAabbMin, ref m_localAabbMax, out localCenter );

			btMatrix3x3 abs_b; trans.getBasis().absolute( out abs_b );

			btVector3 center; trans.Apply( ref localCenter, out center );

			btVector3 extent; localHalfExtents.dot3( ref abs_b, out extent );
			aabbMin = center - extent;
			aabbMax = center + extent;
		}

		public virtual void recalcLocalAabb()
		{
			for( int i = 0; i < 3; i++ )
			{
				btVector3 vec = btVector3.Zero;
				vec[i] = btScalar.BT_ONE;
				btVector3 tmp; localGetSupportingVertex( ref vec, out tmp );
				m_localAabbMax[i] = tmp[i] + m_collisionMargin;
				vec[i] = btScalar.BT_NEG_ONE;
				localGetSupportingVertex( ref vec, out tmp );
				m_localAabbMin[i] = tmp[i] - m_collisionMargin;
			}
		}



		internal class SupportVertexCallback : btTriangleCallback
		{
			btVector3 m_supportVertexLocal;

			public btITransform m_worldTrans;
			public double m_maxDot;
			public btVector3 m_supportVecLocal;

			public SupportVertexCallback( ref btVector3 supportVecWorld, btITransform trans )
			{
				m_supportVertexLocal = btVector3.Zero;
				m_worldTrans = ( trans );
				m_maxDot = ( (double)( -btScalar.BT_LARGE_FLOAT ) );
				m_supportVecLocal = supportVecWorld * m_worldTrans.getBasis();
			}

			public void processTriangle( btVector3[] triangle, int partId, int triangleIndex )
			{
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

			public void GetSupportVertexWorldSpace( out btVector3 result )
			{
				m_worldTrans.Apply( ref m_supportVertexLocal, out result );
			}

			public void GetSupportVertexLocal( out btVector3 result )
			{
				result = m_supportVertexLocal;
			}

		};


		public override void setLocalScaling( ref btVector3 scaling )
		{
			m_meshInterface.setScaling( ref scaling );
			recalcLocalAabb();
		}

		public override void getLocalScaling( out btVector3 result )
		{
			m_meshInterface.getScaling( out result );
		}






		//#define DEBUG_TRIANGLE_MESH

		internal class FilteredCallback : btInternalTriangleIndexCallback
		{
			btTriangleCallback m_callback;
			btIVector3 m_aabbMin;
			btIVector3 m_aabbMax;

			internal FilteredCallback( btTriangleCallback callback, btIVector3 aabbMin, btIVector3 aabbMax )
			{
				m_callback = ( callback );
				m_aabbMin = ( aabbMin );
				m_aabbMax = ( aabbMax );
			}

			internal override void internalProcessTriangleIndex( btVector3[] triangle, int partId, int triangleIndex )
			{
				if( btAabbUtil.TestTriangleAgainstAabb2( triangle, m_aabbMin, m_aabbMax ) )
				{
					//check aabb in triangle-space, before doing this
					m_callback.processTriangle( triangle, partId, triangleIndex );
				}

			}

		};


		internal override void processAllTriangles( btTriangleCallback callback, ref btVector3 aabbMin, ref btVector3 aabbMax )
		{
			FilteredCallback filterCallback = new FilteredCallback( callback, aabbMin, aabbMax );

			m_meshInterface.InternalProcessAllTriangles( filterCallback, ref aabbMin, ref aabbMax );
		}





		public override void calculateLocalInertia( double mass, out btVector3 inertia )
		{
			//(void)mass;
			//moving concave objects not supported
			Debug.Assert( false );
			inertia = btVector3.Zero;
		}


		public virtual void localGetSupportingVertex( ref btVector3 vec, out btVector3 result )
		{
			SupportVertexCallback supportCallback = new SupportVertexCallback( ref vec, btTransform.Identity );

			btVector3 aabbMax = btVector3.Max;
			btVector3 aabbMin = btVector3.Min;

			processAllTriangles( supportCallback, ref aabbMin, ref aabbMax );

			supportCallback.GetSupportVertexLocal( out result );
		}

	};



}
