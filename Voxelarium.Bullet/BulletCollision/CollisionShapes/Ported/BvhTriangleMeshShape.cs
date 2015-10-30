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

using System;
using System.Diagnostics;
using Bullet.Collision.BroadPhase;
using Bullet.LinearMath;

namespace Bullet.Collision.Shapes
{

	///The btBvhTriangleMeshShape is a static-triangle mesh shape, it can only be used for fixed/non-moving objects.
	///If you required moving concave triangle meshes, it is recommended to perform convex decomposition
	///using HACD, see Bullet/Demos/ConvexDecompositionDemo. 
	///Alternatively, you can use btGimpactMeshShape for moving concave triangle meshes.
	///btBvhTriangleMeshShape has several optimizations, such as bounding volume hierarchy and 
	///cache friendly traversal for PlayStation 3 Cell SPU. 
	///It is recommended to enable useQuantizedAabbCompression for better memory usage.
	///It takes a triangle mesh as input, for example a btTriangleMesh or btTriangleIndexVertexArray. The btBvhTriangleMeshShape class allows for triangle mesh deformations by a refit or partialRefit method.
	///Instead of building the bounding volume hierarchy acceleration structure, it is also possible to serialize (save) and deserialize (load) the structure from disk.
	///See Demos\ConcaveDemo\ConcavePhysicsDemo.cpp for an example.
	internal class btBvhTriangleMeshShape<Index, Data> : btTriangleMeshShape<Index, Data>
	{

		btOptimizedBvh<Index, Data> m_bvh;
		btTriangleInfoMap m_triangleInfoMap;

		bool m_useQuantizedAabbCompression;
		bool m_ownsBvh;


		public bool getOwnsBvh()
		{
			return m_ownsBvh;
		}

		public class MyNodeOverlapCallback<Index, Data> : btNodeOverlapCallback
		{
			public btStridingMeshInterface<Index, Data> m_meshInterface;
			public btTriangleCallback m_callback;

			public MyNodeOverlapCallback( btTriangleCallback callback, btStridingMeshInterface<Index, Data> meshInterface )
			{
				m_meshInterface = ( meshInterface );
				m_callback = ( callback );
			}

			public virtual void processNode( int nodeSubPart, int nodeTriangleIndex )
			{
				btVector3[] m_triangle = new btVector3[3];
				Data[] vertexbase;
				int numverts;
				PHY_ScalarType type;
				int stride;
				Index[] indexbase;
				int indexstride;
				int numfaces;
				PHY_ScalarType indicestype;

				m_meshInterface.getLockedReadOnlyVertexIndexBase(
					out vertexbase,
					out numverts,
					out type,
					//out stride,
					out indexbase,
					//out indexstride,
					out numfaces,
					out indicestype,
					nodeSubPart
					);

				//uint* gfxbase = (uint*)( indexbase + nodeTriangleIndex * indexstride );

				Debug.Assert( indicestype == PHY_ScalarType.PHY_INTEGER || indicestype == PHY_ScalarType.PHY_SHORT );

				btVector3 meshScaling; m_meshInterface.getScaling( out meshScaling );
				for( int j = 2; j >= nodeTriangleIndex + 0; j-- )
				{
					int graphicsindex = Convert.ToInt32( indexbase[nodeTriangleIndex + j] );

					m_triangle[j] = new btVector3( Convert.ToDouble( vertexbase[graphicsindex] ) * meshScaling.x
						, Convert.ToDouble( vertexbase[graphicsindex + 1] ) * meshScaling.y
						, Convert.ToDouble( vertexbase[graphicsindex + 2] ) * meshScaling.z );
				}

				/* Perform ray vs. triangle collision here */
				m_callback.processTriangle( m_triangle, nodeSubPart, nodeTriangleIndex );
				m_meshInterface.unLockReadOnlyVertexBase( nodeSubPart );
			}
		};


		//debugging
		public override string ToString() { return "BVHTRIANGLEMESH"; }

		btOptimizedBvh<Index, Data> getOptimizedBvh()
		{
			return m_bvh;
		}

		bool usesQuantizedAabbCompression()
		{
			return m_useQuantizedAabbCompression;
		}

		void setTriangleInfoMap( btTriangleInfoMap triangleInfoMap )
		{
			m_triangleInfoMap = triangleInfoMap;
		}

		btTriangleInfoMap getTriangleInfoMap()
		{
			return m_triangleInfoMap;
		}


#if SERIALIZE_DONE
		virtual int calculateSerializeBufferSize();
	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	string	serialize(object dataBuffer, btSerializer* serializer);

	virtual void	serializeSingleBvh(btSerializer* serializer);

	virtual void	serializeSingleTriangleInfoMap(btSerializer* serializer);
#endif


		///Bvh Concave triangle mesh is a static-triangle mesh shape with Bounding Volume Hierarchy optimization.
		///Uses an interface to access the triangles to allow for sharing graphics/physics triangles.
		public btBvhTriangleMeshShape( btStridingMeshInterface<Index, Data> meshInterface, bool useQuantizedAabbCompression, bool buildBvh = true )
					: base( meshInterface )
		{
			m_bvh = ( null );
			m_triangleInfoMap = ( null );
			m_useQuantizedAabbCompression = ( useQuantizedAabbCompression );
			m_ownsBvh = ( false );
			m_shapeType = BroadphaseNativeTypes.TRIANGLE_MESH_SHAPE_PROXYTYPE;
			//construct bvh from meshInterface
#if !DISABLE_BVH

			if( buildBvh )
			{
				buildOptimizedBvh();
			}

#endif //DISABLE_BVH

		}

		///optionally pass in a larger bvh aabb, used for quantization. This allows for deformations within this aabb
		public btBvhTriangleMeshShape( btStridingMeshInterface<Index, Data> meshInterface, bool useQuantizedAabbCompression, ref btVector3 bvhAabbMin, ref btVector3 bvhAabbMax, bool buildBvh = true )
			: base( meshInterface )
		{
			m_bvh = ( null );
			m_triangleInfoMap = ( null );
			m_useQuantizedAabbCompression = ( useQuantizedAabbCompression );
			m_ownsBvh = ( false );
			m_shapeType = BroadphaseNativeTypes.TRIANGLE_MESH_SHAPE_PROXYTYPE;
			//construct bvh from meshInterface
#if !DISABLE_BVH

			if( buildBvh )
			{
				m_bvh = new btOptimizedBvh();

				m_bvh.build( meshInterface, m_useQuantizedAabbCompression, bvhAabbMin, bvhAabbMax );
				m_ownsBvh = true;
			}

#endif //DISABLE_BVH

		}

		public void partialRefitTree( ref btVector3 aabbMin, ref btVector3 aabbMax )
		{
			m_bvh.refitPartial( m_meshInterface, aabbMin, aabbMax );

			m_localAabbMin.setMin( ref aabbMin );
			m_localAabbMax.setMax( ref aabbMax );
		}


		public void refitTree( ref btVector3 aabbMin, ref btVector3 aabbMax )
		{
			m_bvh.refit( m_meshInterface, aabbMin, aabbMax );

			recalcLocalAabb();
		}

		~btBvhTriangleMeshShape()
		{
			if( m_ownsBvh )
			{
				m_bvh = null;
			}
		}

		public override void performRaycast( btTriangleCallback callback, ref btVector3 raySource, ref btVector3 rayTarget )
		{
			MyNodeOverlapCallback myNodeCallback = new MyNodeOverlapCallback( callback, m_meshInterface );

			m_bvh.reportRayOverlappingNodex( &myNodeCallback, raySource, rayTarget );
		}

		public override void performConvexcast( btTriangleCallback callback, ref btVector3 raySource, ref btVector3 rayTarget, ref btVector3 aabbMin, ref btVector3 aabbMax )
		{

			MyNodeOverlapCallback myNodeCallback = new MyNodeOverlapCallback( callback, m_meshInterface );

			m_bvh.reportBoxCastOverlappingNodex( &myNodeCallback, raySource, rayTarget, aabbMin, aabbMax );
		}


		public class MyNodeOverlapCallback<Index, Data> : btNodeOverlapCallback
		{
			btStridingMeshInterface<Index, Data> m_meshInterface;
			btTriangleCallback m_callback;
			btVector3[] m_triangle = new btVector3[3];


			MyNodeOverlapCallback( btTriangleCallback callback, btStridingMeshInterface<Index, Data> meshInterface )
			{
				m_meshInterface = ( meshInterface );
				m_callback = ( callback );
			}

			public virtual void processNode( int nodeSubPart, int nodeTriangleIndex )
			{
				Data[] vertexbase;
				int numverts;
				PHY_ScalarType type;
				//int stride;
				Index[] indexbase;
				//int indexstride;
				int numfaces;
				PHY_ScalarType indicestype;


				m_meshInterface.getLockedReadOnlyVertexIndexBase(
					out vertexbase,
					out numverts,
					out type,
					//out stride,
					out indexbase,
					//out indexstride,
					out numfaces,
					out indicestype,
					nodeSubPart );

				//uint* gfxbase = (uint*)( indexbase + nodeTriangleIndex * indexstride );
				Debug.Assert( indicestype == PHY_ScalarType.PHY_INTEGER || indicestype == PHY_ScalarType.PHY_SHORT || indicestype == PHY_ScalarType.PHY_UCHAR );

				btVector3 meshScaling; m_meshInterface.getScaling( out meshScaling );
				for( int j = 2; j >= 0; j-- )
				{

					int graphicsindex = indicestype == PHY_ScalarType.PHY_SHORT ? ( (ushort*)gfxbase )[j] : indicestype == PHY_ScalarType.PHY_INTEGER
						? gfxbase[j]
						: ( gfxbase )[j];


#if DEBUG_TRIANGLE_MESH
				Console.WriteLine("%d ,",graphicsindex);
#endif //DEBUG_TRIANGLE_MESH
					if( type == PHY_ScalarType.PHY_FLOAT )
					{
						Data graphicsbase = (Data)( vertexbase + graphicsindex * stride );

						m_triangle[j] = btVector3(
																			graphicsbase[0] * meshScaling.x,
																			graphicsbase[1] * meshScaling.y,
																			graphicsbase[2] * meshScaling.z );
					}
					else
					{
						double* graphicsbase = (double*)( vertexbase + graphicsindex * stride );

						m_triangle[j] = btVector3(
										(double)( graphicsbase[0] ) * meshScaling.x,
										(double)( graphicsbase[1] ) * meshScaling.y,
										(double)( graphicsbase[2] ) * meshScaling.z );
					}
#if DEBUG_TRIANGLE_MESH
				Console.WriteLine("triangle vertices:%f,%f,%f\n",triangle[j].x,triangle[j].y,triangle[j].z);
#endif //DEBUG_TRIANGLE_MESH
				}

				m_callback.processTriangle( m_triangle, nodeSubPart, nodeTriangleIndex );
				m_meshInterface.unLockReadOnlyVertexBase( nodeSubPart );
			}

		};

		//perform bvh tree traversal and report overlapping triangles to 'callback'
		public override void processAllTriangles( btTriangleCallback callback, ref btVector3 aabbMin, ref btVector3 aabbMax )
		{

#if DISABLE_BVH
	//brute force traverse all triangles
	btTriangleMeshShape::processAllTriangles(callback,aabbMin,aabbMax);
#else

			//first get all the nodes
			MyNodeOverlapCallback<Index, Data> myNodeCallback = new MyNodeOverlapCallback<Index,Data>( callback, m_meshInterface );

			m_bvh.reportAabbOverlappingNodex( myNodeCallback, out aabbMin, out aabbMax );
#endif//DISABLE_BVH
		}

		public virtual void setLocalScaling( ref btVector3 scaling )
		{
			if( ( getLocalScaling() - scaling ).length2() > SIMD_EPSILON )
			{
				btTriangleMeshShape::setLocalScaling( scaling );
				buildOptimizedBvh();
			}
		}

		public void buildOptimizedBvh()
		{
			if( m_ownsBvh )
			{
				m_bvh = null;
			}
			///m_localAabbMin/m_localAabbMax is already re-calculated in btTriangleMeshShape. We could just scale aabb, but this needs some more work
			//object mem = btAlignedAlloc( sizeof( btOptimizedBvh ), 16 );
			m_bvh = new btOptimizedBvh();
			//rebuild the bvh...
			m_bvh.build( m_meshInterface, m_useQuantizedAabbCompression, m_localAabbMin, m_localAabbMax );
			m_ownsBvh = true;
		}

		public void setOptimizedBvh( btOptimizedBvh bvh, ref btVector3 scaling /* btVector.One */ )
		{
			Debug.Assert( !m_bvh );
			Debug.Assert( !m_ownsBvh );

			m_bvh = bvh;
			m_ownsBvh = false;
			// update the scaling without rebuilding the bvh
			if( ( getLocalScaling() - scaling ).length2() > SIMD_EPSILON )
			{
				btTriangleMeshShape::setLocalScaling( scaling );
			}
		}


#if SERIALIZE_DONE
///fills the dataBuffer and returns the struct name (and 0 on failure)
string btBvhTriangleMeshShape::serialize( object dataBuffer, btSerializer* serializer )
{
	btTriangleMeshShapeData* trimeshData = (btTriangleMeshShapeData*)dataBuffer;

	btCollisionShape::serialize( &trimeshData.m_collisionShapeData, serializer );

	m_meshInterface.serialize( &trimeshData.m_meshInterface, serializer );

	trimeshData.m_collisionMargin = float( m_collisionMargin );



	if( m_bvh && !( serializer.getSerializationFlags() & BT_SERIALIZE_NO_BVH ) )
	{
		object chunk = serializer.findPointer( m_bvh );
		if( chunk )
		{
#if BT_USE_DOUBLE_PRECISION
			trimeshData.m_quantizedDoubleBvh = (btQuantizedBvhData*)chunk;
			trimeshData.m_quantizedFloatBvh = 0;
#else
			trimeshData.m_quantizedFloatBvh = (btQuantizedBvhData*)chunk;
			trimeshData.m_quantizedDoubleBvh = 0;
#endif //BT_USE_DOUBLE_PRECISION
		}
		else
		{

#if BT_USE_DOUBLE_PRECISION
			trimeshData.m_quantizedDoubleBvh = (btQuantizedBvhData*)serializer.getUniquePointer(m_bvh);
			trimeshData.m_quantizedFloatBvh = 0;
#else
			trimeshData.m_quantizedFloatBvh = (btQuantizedBvhData*)serializer.getUniquePointer( m_bvh );
			trimeshData.m_quantizedDoubleBvh = 0;
#endif //BT_USE_DOUBLE_PRECISION

			int sz = m_bvh.calculateSerializeBufferSizeNew();
			btChunk* chunk = serializer.allocate( sz, 1 );
			string structType = m_bvh.serialize( chunk.m_oldPtr, serializer );
			serializer.finalizeChunk( chunk, structType, BT_QUANTIZED_BVH_CODE, m_bvh );
		}
	}
	else
	{
		trimeshData.m_quantizedFloatBvh = 0;
		trimeshData.m_quantizedDoubleBvh = 0;
	}



	if( m_triangleInfoMap && !( serializer.getSerializationFlags() & BT_SERIALIZE_NO_TRIANGLEINFOMAP ) )
	{
		object chunk = serializer.findPointer( m_triangleInfoMap );
		if( chunk )
		{
			trimeshData.m_triangleInfoMap = (btTriangleInfoMapData*)chunk;
		}
		else
		{
			trimeshData.m_triangleInfoMap = (btTriangleInfoMapData*)serializer.getUniquePointer( m_triangleInfoMap );
			int sz = m_triangleInfoMap.calculateSerializeBufferSize();
			btChunk* chunk = serializer.allocate( sz, 1 );
			string structType = m_triangleInfoMap.serialize( chunk.m_oldPtr, serializer );
			serializer.finalizeChunk( chunk, structType, BT_TRIANLGE_INFO_MAP, m_triangleInfoMap );
		}
	}
	else
	{
		trimeshData.m_triangleInfoMap = 0;
	}

	return "btTriangleMeshShapeData";
}

void btBvhTriangleMeshShape::serializeSingleBvh( btSerializer* serializer )
{
	if( m_bvh )
	{
		int len = m_bvh.calculateSerializeBufferSizeNew(); //make sure not to use calculateSerializeBufferSize because it is used for in-place
		btChunk* chunk = serializer.allocate( len, 1 );
		string structType = m_bvh.serialize( chunk.m_oldPtr, serializer );
		serializer.finalizeChunk( chunk, structType, BT_QUANTIZED_BVH_CODE, (object)m_bvh );
	}
}

void btBvhTriangleMeshShape::serializeSingleTriangleInfoMap( btSerializer* serializer )
{
	if( m_triangleInfoMap )
	{
		int len = m_triangleInfoMap.calculateSerializeBufferSize();
		btChunk* chunk = serializer.allocate( len, 1 );
		string structType = m_triangleInfoMap.serialize( chunk.m_oldPtr, serializer );
		serializer.finalizeChunk( chunk, structType, BT_TRIANLGE_INFO_MAP, (object)m_triangleInfoMap );
	}
}
#endif

	};

#if SERIALIZE_DONE
///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btTriangleMeshShapeData
{
	btCollisionShapeData	m_collisionShapeData;

	btStridingMeshInterfaceData m_meshInterface;

	btQuantizedBvhFloatData		*m_quantizedFloatBvh;
	btQuantizedBvhDoubleData	*m_quantizedDoubleBvh;

	btTriangleInfoMapData	*m_triangleInfoMap;
	
	float	m_collisionMargin;

	char m_pad3[4];
	
};


public	int	btBvhTriangleMeshShape::calculateSerializeBufferSize()
{
	return sizeof(btTriangleMeshShapeData);
}
#endif

}
