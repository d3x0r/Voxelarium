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

namespace Bullet.Collision.Shapes
{

	///	The btStridingMeshInterface is the interface class for high performance generic access to triangle meshes, used in combination with btBvhTriangleMeshShape and some other collision shapes.
	/// Using index striding of 3*sizeof(integer) it can use triangle arrays, using index striding of 1*sizeof(integer) it can handle triangle strips.
	/// It allows for sharing graphics and collision meshes. Also it provides locking/unlocking of graphics meshes that are in gpu memory.
	public abstract class btStridingMeshInterface<Index, Data>
	{
		protected btVector3 m_scaling;


		public btStridingMeshInterface()
		{
			m_scaling = btVector3.One;
		}

		/// get read and write access to a subpart of a triangle mesh
		/// this subpart has a continuous array of vertices and indices
		/// in this way the mesh can be handled as chunks of memory with striding
		/// very similar to OpenGL vertexarray support
		/// make a call to unLockVertexBase when the read and write access is finished	
		public abstract void getLockedVertexIndexBase( out Data[] vertexbase, out int numverts, out PHY_ScalarType type
			, out Index[] indexbase, out int numfaces, out PHY_ScalarType indicestype, int subpart = 0 );

		public abstract void getLockedReadOnlyVertexIndexBase( out Data[] vertexbase, out int numverts, out PHY_ScalarType type
			, out Index[] indexbase, out int numfaces, out PHY_ScalarType indicestype, int subpart = 0 );

		/// unLockVertexBase finishes the access to a subpart of the triangle mesh
		/// make a call to unLockVertexBase when the read and write access (using getLockedVertexIndexBase) is finished
		public abstract void unLockVertexBase( int subpart );

		public abstract void unLockReadOnlyVertexBase( int subpart );


		/// getNumSubParts returns the number of seperate subparts
		/// each subpart has a continuous array of vertices and indices
		public abstract int getNumSubParts();

		public abstract void preallocateVertices( int numverts );
		public abstract void preallocateIndices( int numindices );

		public virtual bool hasPremadeAabb() { return false; }

		public virtual void setPremadeAabb( ref btVector3 aabbMin, ref btVector3 aabbMax )
		{
		}
		public virtual void getPremadeAabb( out btVector3 aabbMin, out btVector3 aabbMax )
		{
			aabbMax = aabbMin = btVector3.Zero;
		}

		public void getScaling( out btVector3 result )
		{
			result = m_scaling;
		}
		public btIVector3 getScaling()
		{
			return m_scaling;
		}
		public void setScaling( ref btVector3 scaling )
		{
			m_scaling = scaling;
		}


		public unsafe virtual void InternalProcessAllTriangles( btInternalTriangleIndexCallback callback, ref btVector3 aabbMin, ref btVector3 aabbMax )
		{
			int numtotalphysicsverts = 0;
			int part, graphicssubparts = getNumSubParts();
			Data[] vertexarray;
			Index[] indexarray;
			//int indexstride;
			PHY_ScalarType type;
			PHY_ScalarType gfxindextype;
			//int stride;
			int numverts, numtriangles;
			int gfxindex;
			btVector3[] triangle = new btVector3[3];

			btVector3 meshScaling; getScaling( out meshScaling );

			///if the number of parts is big, the performance might drop due to the innerloop switch on indextype
			for( part = 0; part < graphicssubparts; part++ )
			{
				getLockedReadOnlyVertexIndexBase( out vertexarray, out numverts, out type, out indexarray, out numtriangles, out gfxindextype, part );
				numtotalphysicsverts += numtriangles * 3; //upper bound

				///unlike that developers want to pass in double-precision meshes in single-precision Bullet build
				///so disable this feature by default
				///see patch http://code.google.com/p/bullet/issues/detail?id=213

				switch( type )
				{
					case PHY_ScalarType.PHY_FLOAT:
						{
							float[] tmp_data = vertexarray as float[];
							fixed ( float* _vertexbase = tmp_data )
							{
								float* graphicsbase;
								float* vertexbase = _vertexbase;
								switch( gfxindextype )
								{
									case PHY_ScalarType.PHY_INTEGER:
										{
											uint[] tmp_index = indexarray as uint[];
											fixed ( uint* index_base = tmp_index )
											{
												uint* tri_indices = ( index_base );
												for( gfxindex = 0; gfxindex < numtriangles; gfxindex++ )
												{
													graphicsbase = ( vertexbase + tri_indices[0] );
													triangle[0].setValue( graphicsbase[0] * meshScaling.x, graphicsbase[1] * meshScaling.y, graphicsbase[2] * meshScaling.z );
													graphicsbase = (float*)( vertexbase + tri_indices[1] );
													triangle[1].setValue( graphicsbase[0] * meshScaling.x, graphicsbase[1] * meshScaling.y, graphicsbase[2] * meshScaling.z );
													graphicsbase = (float*)( vertexbase + tri_indices[2] );
													triangle[2].setValue( graphicsbase[0] * meshScaling.x, graphicsbase[1] * meshScaling.y, graphicsbase[2] * meshScaling.z );
													callback.internalProcessTriangleIndex( triangle, part, gfxindex );
													tri_indices += 3;
												}
											}
											break;
										}
									case PHY_ScalarType.PHY_SHORT:
										{
											ushort[] tmp_index = indexarray as ushort[];
											fixed ( ushort* index_base = tmp_index )
											{
												ushort* tri_indices = ( index_base );
												for( gfxindex = 0; gfxindex < numtriangles; gfxindex++ )
												{
													//ushort* tri_indices = ( ushort int*)( indexbase + gfxindex * indexstride );
													graphicsbase = (float*)( vertexbase + tri_indices[0] );
													triangle[0].setValue( graphicsbase[0] * meshScaling.x, graphicsbase[1] * meshScaling.y, graphicsbase[2] * meshScaling.z );
													graphicsbase = (float*)( vertexbase + tri_indices[1] );
													triangle[1].setValue( graphicsbase[0] * meshScaling.x, graphicsbase[1] * meshScaling.y, graphicsbase[2] * meshScaling.z );
													graphicsbase = (float*)( vertexbase + tri_indices[2] );
													triangle[2].setValue( graphicsbase[0] * meshScaling.x, graphicsbase[1] * meshScaling.y, graphicsbase[2] * meshScaling.z );
													callback.internalProcessTriangleIndex( triangle, part, gfxindex );
													tri_indices += 3;
												}
											}
											break;
										}
									case PHY_ScalarType.PHY_UCHAR:
										{
											byte[] tmp_index = indexarray as byte[];
											fixed ( byte* index_base = tmp_index )
											{
												byte* tri_indices = ( index_base );
												for( gfxindex = 0; gfxindex < numtriangles; gfxindex++ )
												{
													//tri_indices = ( string( indexbase + gfxindex * indexstride );
													graphicsbase = (float*)( vertexbase + tri_indices[0] );
													triangle[0].setValue( graphicsbase[0] * meshScaling.x, graphicsbase[1] * meshScaling.y, graphicsbase[2] * meshScaling.z );
													graphicsbase = (float*)( vertexbase + tri_indices[1] );
													triangle[1].setValue( graphicsbase[0] * meshScaling.x, graphicsbase[1] * meshScaling.y, graphicsbase[2] * meshScaling.z );
													graphicsbase = (float*)( vertexbase + tri_indices[2] );
													triangle[2].setValue( graphicsbase[0] * meshScaling.x, graphicsbase[1] * meshScaling.y, graphicsbase[2] * meshScaling.z );
													callback.internalProcessTriangleIndex( triangle, part, gfxindex );
													tri_indices += 3;
												}
											}
											break;
										}
									default:
										Debug.Assert( ( gfxindextype == PHY_ScalarType.PHY_INTEGER ) || ( gfxindextype == PHY_ScalarType.PHY_SHORT ) );
										break;
								}
							}
							break;
						}
					case PHY_ScalarType.PHY_DOUBLE:
						{
							double[] tmp_data = vertexarray as double[];
							fixed ( double* _vertexbase = tmp_data )
							{
								double* graphicsbase;
								double* vertexbase = _vertexbase;
								switch( gfxindextype )
								{
									case PHY_ScalarType.PHY_INTEGER:
										{
											uint[] tmp_index = indexarray as uint[];
											fixed ( uint* index_base = tmp_index )
											{
												uint* tri_indices = ( index_base );
												for( gfxindex = 0; gfxindex < numtriangles; gfxindex++ )
												{
													//uint* tri_indices = (uint*)( indexbase + gfxindex * indexstride );
													graphicsbase = (double*)( vertexbase + tri_indices[0] );
													triangle[0].setValue( (double)graphicsbase[0] * meshScaling.x, (double)graphicsbase[1] * meshScaling.y, (double)graphicsbase[2] * meshScaling.z );
													graphicsbase = (double*)( vertexbase + tri_indices[1] );
													triangle[1].setValue( (double)graphicsbase[0] * meshScaling.x, (double)graphicsbase[1] * meshScaling.y, (double)graphicsbase[2] * meshScaling.z );
													graphicsbase = (double*)( vertexbase + tri_indices[2] );
													triangle[2].setValue( (double)graphicsbase[0] * meshScaling.x, (double)graphicsbase[1] * meshScaling.y, (double)graphicsbase[2] * meshScaling.z );
													callback.internalProcessTriangleIndex( triangle, part, gfxindex );
													tri_indices += 3;
												}
											}
											break;
										}
									case PHY_ScalarType.PHY_SHORT:
										{
											ushort[] tmp_index = indexarray as ushort[];
											fixed ( ushort* index_base = tmp_index )
											{
												ushort* tri_indices = ( index_base );
												for( gfxindex = 0; gfxindex < numtriangles; gfxindex++ )
												{
													//ushort int* tri_indices = ( ushort int*)( indexbase + gfxindex * indexstride );
													graphicsbase = (double*)( vertexbase + tri_indices[0] );
													triangle[0].setValue( (double)graphicsbase[0] * meshScaling.x, (double)graphicsbase[1] * meshScaling.y, (double)graphicsbase[2] * meshScaling.z );
													graphicsbase = (double*)( vertexbase + tri_indices[1] );
													triangle[1].setValue( (double)graphicsbase[0] * meshScaling.x, (double)graphicsbase[1] * meshScaling.y, (double)graphicsbase[2] * meshScaling.z );
													graphicsbase = (double*)( vertexbase + tri_indices[2] );
													triangle[2].setValue( (double)graphicsbase[0] * meshScaling.x, (double)graphicsbase[1] * meshScaling.y, (double)graphicsbase[2] * meshScaling.z );
													callback.internalProcessTriangleIndex( triangle, part, gfxindex );
													tri_indices += 3;
												}
											}
											break;
										}
									case PHY_ScalarType.PHY_UCHAR:
										{
											byte[] tmp_index = indexarray as byte[];
											fixed ( byte* index_base = tmp_index )
											{
												byte* tri_indices = ( index_base );
												for( gfxindex = 0; gfxindex < numtriangles; gfxindex++ )
												{
													//tri_indices = ( string( indexbase + gfxindex * indexstride );
													graphicsbase = (double*)( vertexbase + tri_indices[0] );
													triangle[0].setValue( (double)graphicsbase[0] * meshScaling.x, (double)graphicsbase[1] * meshScaling.y, (double)graphicsbase[2] * meshScaling.z );
													graphicsbase = (double*)( vertexbase + tri_indices[1] );
													triangle[1].setValue( (double)graphicsbase[0] * meshScaling.x, (double)graphicsbase[1] * meshScaling.y, (double)graphicsbase[2] * meshScaling.z );
													graphicsbase = (double*)( vertexbase + tri_indices[2] );
													triangle[2].setValue( (double)graphicsbase[0] * meshScaling.x, (double)graphicsbase[1] * meshScaling.y, (double)graphicsbase[2] * meshScaling.z );
													callback.internalProcessTriangleIndex( triangle, part, gfxindex );
													tri_indices += 3;
												}
											}
											break;
										}
									default:
										Debug.Assert( ( gfxindextype == PHY_ScalarType.PHY_INTEGER ) || ( gfxindextype == PHY_ScalarType.PHY_SHORT ) );
										break;
								}
							}
							break;
						}
					default:
						Debug.Assert( ( type == PHY_ScalarType.PHY_FLOAT ) || ( type == PHY_ScalarType.PHY_DOUBLE ) );
						break;
				}

				unLockReadOnlyVertexBase( part );
			}
		}

		public class AabbCalculationCallback : btInternalTriangleIndexCallback
		{
			public btVector3 m_aabbMin;
			public btVector3 m_aabbMax;

			public AabbCalculationCallback()
			{
				m_aabbMin.setValue( (double)( btScalar.BT_LARGE_FLOAT ), (double)( btScalar.BT_LARGE_FLOAT ), (double)( btScalar.BT_LARGE_FLOAT ) );
				m_aabbMax.setValue( (double)( -btScalar.BT_LARGE_FLOAT ), (double)( -btScalar.BT_LARGE_FLOAT ), (double)( -btScalar.BT_LARGE_FLOAT ) );
			}

			public override void internalProcessTriangleIndex( btVector3[] triangle, int partId, int triangleIndex )
			{
				//(void)partId;
				//(void)triangleIndex;
				m_aabbMin.setMin( ref triangle[0] );
				m_aabbMax.setMax( ref triangle[0] );
				m_aabbMin.setMin( ref triangle[1] );
				m_aabbMax.setMax( ref triangle[1] );
				m_aabbMin.setMin( ref triangle[2] );
				m_aabbMax.setMax( ref triangle[2] );
			}
		};

		///brute force method to calculate aabb
		public void calculateAabbBruteForce( out btVector3 aabbMin, out btVector3 aabbMax )
		{

			//first calculate the total aabb for all triangles
			AabbCalculationCallback aabbCallback = new AabbCalculationCallback();
			aabbMin = btVector3.Max;
			aabbMax = btVector3.Min;
			InternalProcessAllTriangles( aabbCallback, ref aabbMin, ref aabbMax );

			aabbMin = aabbCallback.m_aabbMin;
			aabbMax = aabbCallback.m_aabbMax;
		}


#if SERIALIZE_DONE

		///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
		struct btMeshPartData
		{
			btVector3FloatData* m_vertices3f;
			btVector3DoubleData* m_vertices3d;

			btIntIndexData* m_indices32;
			btShortIntIndexTripletData* m_3indices16;
			btCharIndexTripletData* m_3indices8;

			btShortIntIndexData* m_indices16;//backwards compatibility

			int m_numTriangles;//length of m_indices = m_numTriangles
			int m_numVertices;
		};


	
		struct btIntIndexData
		{
			int m_value;
		};

		struct btShortIntIndexData
		{
			short m_value;
			char m_pad[2];
		};

		struct btShortIntIndexTripletData
		{
			short m_values[3];
			char m_pad[2];
		};

		struct btCharIndexTripletData
		{
			byte m_values[3];
			char m_pad;
		};


	
	virtual int calculateSerializeBufferSize();
///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btStridingMeshInterfaceData
{
	btMeshPartData* m_meshPartsPtr;
	btVector3FloatData m_scaling;
	int m_numMeshParts;
	char m_padding[4];
};




public int btStridingMeshInterface::calculateSerializeBufferSize()
{
	return sizeof( btStridingMeshInterfaceData );
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
string btStridingMeshInterface::serialize( object dataBuffer, btSerializer* serializer )
{
	btStridingMeshInterfaceData* trimeshData = (btStridingMeshInterfaceData*)dataBuffer;

	trimeshData.m_numMeshParts = getNumSubParts();

	//object uniquePtr = 0;

	trimeshData.m_meshPartsPtr = 0;

	if( trimeshData.m_numMeshParts )
	{
		btChunk* chunk = serializer.allocate( sizeof( btMeshPartData ), trimeshData.m_numMeshParts );
		btMeshPartData* memPtr = (btMeshPartData*)chunk.m_oldPtr;
		trimeshData.m_meshPartsPtr = (btMeshPartData*)serializer.getUniquePointer( memPtr );


		//	int numtotalphysicsverts = 0;
		int part, graphicssubparts = getNumSubParts();
		string nsigned char* vertexbase;
		string nsigned char* indexbase;
		int indexstride;
		PHY_ScalarType type;
		PHY_ScalarType gfxindextype;
		int stride, numverts, numtriangles;
		int gfxindex;
		//	btVector3 triangle[3];

		//	btVector3 meshScaling = getScaling();

		///if the number of parts is big, the performance might drop due to the innerloop switch on indextype
		for( part = 0; part < graphicssubparts; part++, memPtr++ )
		{
			getLockedReadOnlyVertexIndexBase( &vertexbase, numverts, type, stride, &indexbase, indexstride, numtriangles, gfxindextype, part );
			memPtr.m_numTriangles = numtriangles;//indices = 3*numtriangles
			memPtr.m_numVertices = numverts;
			memPtr.m_indices16 = 0;
			memPtr.m_indices32 = 0;
			memPtr.m_3indices16 = 0;
			memPtr.m_3indices8 = 0;
			memPtr.m_vertices3f = 0;
			memPtr.m_vertices3d = 0;


			switch( gfxindextype )
			{
				case PHY_INTEGER:
					{
						int numindices = numtriangles * 3;

						if( numindices )
						{
							btChunk* chunk = serializer.allocate( sizeof( btIntIndexData ), numindices );
							btIntIndexData* tmpIndices = (btIntIndexData*)chunk.m_oldPtr;
							memPtr.m_indices32 = (btIntIndexData*)serializer.getUniquePointer( tmpIndices );
							for( gfxindex = 0; gfxindex < numtriangles; gfxindex++ )
							{
								uint* tri_indices = (uint*)( indexbase + gfxindex * indexstride );
								tmpIndices[gfxindex * 3].m_value = tri_indices[0];
								tmpIndices[gfxindex * 3 + 1].m_value = tri_indices[1];
								tmpIndices[gfxindex * 3 + 2].m_value = tri_indices[2];
							}
							serializer.finalizeChunk( chunk, "btIntIndexData", BT_ARRAY_CODE, (object)chunk.m_oldPtr );
						}
						break;
					}
				case PHY_SHORT:
					{
						if( numtriangles )
						{
							btChunk* chunk = serializer.allocate( sizeof( btShortIntIndexTripletData ), numtriangles );
							btShortIntIndexTripletData* tmpIndices = (btShortIntIndexTripletData*)chunk.m_oldPtr;
							memPtr.m_3indices16 = (btShortIntIndexTripletData*)serializer.getUniquePointer( tmpIndices );
							for( gfxindex = 0; gfxindex < numtriangles; gfxindex++ )
							{
								ushort int* tri_indices = ( ushort int*)( indexbase + gfxindex * indexstride );
		tmpIndices[gfxindex].m_values[0] = tri_indices[0];
		tmpIndices[gfxindex].m_values[1] = tri_indices[1];
		tmpIndices[gfxindex].m_values[2] = tri_indices[2];
	}
	serializer.finalizeChunk( chunk, "btShortIntIndexTripletData", BT_ARRAY_CODE, (object)chunk.m_oldPtr );
}
					break;
				}
				case PHY_UCHAR:
				{
					if (numtriangles)
					{
						btChunk* chunk = serializer.allocate( sizeof( btCharIndexTripletData ), numtriangles );
btCharIndexTripletData* tmpIndices = (btCharIndexTripletData*)chunk.m_oldPtr;
memPtr.m_3indices8 = (btCharIndexTripletData*) serializer.getUniquePointer(tmpIndices);
						for (gfxindex=0;gfxindex<numtriangles;gfxindex++)
						{
							btri_indices= (string (indexbase+gfxindex* indexstride);
							tmpIndices[gfxindex].m_values[0] = tri_indices[0];
							tmpIndices[gfxindex].m_values[1] = tri_indices[1];
							tmpIndices[gfxindex].m_values[2] = tri_indices[2];
						}
						serializer.finalizeChunk(chunk,"btCharIndexTripletData",BT_ARRAY_CODE,(object)chunk.m_oldPtr);
					}
					break;
				}
			default:
				{
					Debug.Assert(false);
					//unknown index type
				}
			}

			switch (type)
			{
			case PHY_FLOAT:
			 {
				 float* graphicsbase;

				 if (numverts)
				 {
					 btChunk* chunk = serializer.allocate( sizeof( btVector3FloatData ), numverts );
btVector3FloatData* tmpVertices = (btVector3FloatData*)chunk.m_oldPtr;
memPtr.m_vertices3f = (btVector3FloatData*)serializer.getUniquePointer(tmpVertices);
					 for (int i = 0; i<numverts;i++)
					 {
						 graphicsbase = (float*)(vertexbase+i* stride);
						 tmpVertices[i].m_floats[0] = graphicsbase[0];
						 tmpVertices[i].m_floats[1] = graphicsbase[1];
						 tmpVertices[i].m_floats[2] = graphicsbase[2];
					 }
					 serializer.finalizeChunk(chunk,"btVector3FloatData",BT_ARRAY_CODE,(object)chunk.m_oldPtr);
				 }
				 break;
				}

			case PHY_DOUBLE:
				{
					if (numverts)
					{
						btChunk* chunk = serializer.allocate( sizeof( btVector3DoubleData ), numverts );
btVector3DoubleData* tmpVertices = (btVector3DoubleData*)chunk.m_oldPtr;
memPtr.m_vertices3d = (btVector3DoubleData*) serializer.getUniquePointer(tmpVertices);
						for (int i = 0; i<numverts;i++)
					 {
						 double* graphicsbase = (double*)( vertexbase + i * stride );//for now convert to float, might leave it at double
tmpVertices[i].m_floats[0] = graphicsbase[0];
						 tmpVertices[i].m_floats[1] = graphicsbase[1];
						 tmpVertices[i].m_floats[2] = graphicsbase[2];
					 }
						serializer.finalizeChunk(chunk,"btVector3DoubleData",BT_ARRAY_CODE,(object)chunk.m_oldPtr);
					}
					break;
				}

			default:
				Debug.Assert((type == PHY_FLOAT) || (type == PHY_DOUBLE));
			}

			unLockReadOnlyVertexBase( part);
		}

		serializer.finalizeChunk(chunk,"btMeshPartData",BT_ARRAY_CODE,chunk.m_oldPtr);
	}


	m_scaling.serializeFloat(trimeshData.m_scaling);
	return "btStridingMeshInterfaceData";
}
#endif

	};

}

