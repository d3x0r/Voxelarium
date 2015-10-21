#define BT_USE_DOUBLE_PRECISION
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


	///The btIndexedMesh indexes a single vertex and index array. Multiple btIndexedMesh objects can be passed into a btTriangleIndexVertexArray using addIndexedMesh.
	///Instead of the number of indices, we pass the number of triangles.
	public class btIndexedMesh<Index, Data>
	{
		internal int m_numTriangles;
		internal Index[] m_triangleIndexBase;
		// Size in byte of the indices for one triangle (3*sizeof(index_type) if the indices are tightly packed)
		internal int m_triangleIndexStride;
		internal int m_numVertices;
		internal Data[] m_vertexBase;
		// Size of a vertex, in bytes
		internal int m_vertexStride;

		// The index type is set when adding an indexed mesh to the
		// btTriangleIndexVertexArray, do not set it manually
		internal PHY_ScalarType m_indexType;

		// The vertex type has a default type similar to Bullet's precision mode (float or double)
		// but can be set manually if you for example run Bullet with double precision but have
		// mesh data in single precision..
		internal PHY_ScalarType m_vertexType;


		internal btIndexedMesh()
		{
			m_indexType = ( PHY_ScalarType.PHY_INTEGER );
#if BT_USE_DOUBLE_PRECISION
			m_vertexType = ( PHY_ScalarType.PHY_DOUBLE );
#else // BT_USE_DOUBLE_PRECISION
			m_vertexType = ( PHY_FLOAT );
#endif // BT_USE_DOUBLE_PRECISION
		}
	}


	public class IndexedMeshArray<Index,Data> : btList<btIndexedMesh<Index, Data>>
	{
	}

	///The btTriangleIndexVertexArray allows to access multiple triangle meshes, by indexing into existing triangle/index arrays.
	///Additional meshes can be added using addIndexedMesh
	///No duplcate is made of the vertex/index data, it only indexes into external vertex/index arrays.
	///So keep those arrays around during the lifetime of this btTriangleIndexVertexArray.
	public class btTriangleIndexVertexArray<Index,Data> : btStridingMeshInterface<Index,Data>
	{
		protected IndexedMeshArray<Index, Data> m_indexedMeshes;
		protected bool m_hasAabb; // using int instead of bool to maintain alignment
		protected btVector3 m_aabbMin;
		protected btVector3 m_aabbMax;

		public btTriangleIndexVertexArray()
		{
			m_hasAabb = false;
		}

		public void addIndexedMesh( btIndexedMesh<Index, Data> mesh, PHY_ScalarType indexType = PHY_ScalarType.PHY_INTEGER )
		{
			m_indexedMeshes.Add( mesh );
			m_indexedMeshes[m_indexedMeshes.Count - 1].m_indexType = indexType;
		}

		/// unLockVertexBase finishes the access to a subpart of the triangle mesh
		/// make a call to unLockVertexBase when the read and write access (using getLockedVertexIndexBase) is finished
		public override void unLockVertexBase( int subpart ) {  }

		public override void unLockReadOnlyVertexBase( int subpart ) {  }

		/// getNumSubParts returns the number of seperate subparts
		/// each subpart has a continuous array of vertices and indices
		public override int getNumSubParts()
		{
			return (int)m_indexedMeshes.Count;
		}

		public IndexedMeshArray<Index, Data> getIndexedMeshArray()
		{
			return m_indexedMeshes;
		}

		public override void preallocateVertices( int numverts ) { }
		public override void preallocateIndices( int numindices ) { }


		//just to be backwards compatible
		public btTriangleIndexVertexArray( int numTriangles, Index[] triangleIndexBase, int triangleIndexStride, int numVertices, Data[] vertexBase, int vertexStride )
		{
			btIndexedMesh<Index, Data> mesh = new btIndexedMesh<Index, Data>();
			m_hasAabb = false;

			mesh.m_numTriangles = numTriangles;
			mesh.m_triangleIndexBase = triangleIndexBase;
			mesh.m_triangleIndexStride = triangleIndexStride;
			mesh.m_numVertices = numVertices;
			mesh.m_vertexBase = vertexBase;
			mesh.m_vertexStride = vertexStride;

			addIndexedMesh( mesh );

		}

		public override void getLockedVertexIndexBase( out Data[] vertexbase, out int numverts, out PHY_ScalarType type
			, out Index[] indexbase, out int numfaces, out PHY_ScalarType indicestype, int subpart = 0 )
		{
			Debug.Assert( subpart < getNumSubParts() );

			btIndexedMesh<Index,Data> mesh = m_indexedMeshes[subpart];

			numverts = mesh.m_numVertices;
			( vertexbase ) = mesh.m_vertexBase;

			type = mesh.m_vertexType;

			//vertexStride = mesh.m_vertexStride;

			numfaces = mesh.m_numTriangles;

			( indexbase ) = mesh.m_triangleIndexBase;
			//indexstride = mesh.m_triangleIndexStride;
			indicestype = mesh.m_indexType;
		}

		public override void getLockedReadOnlyVertexIndexBase( out Data[] vertexbase, out int numverts, out PHY_ScalarType type, out Index[] indexbase, out int numfaces, out PHY_ScalarType indicestype, int subpart = 0 )
		{
			btIndexedMesh<Index,Data>  mesh = m_indexedMeshes[subpart];

			numverts = mesh.m_numVertices;
			( vertexbase ) = mesh.m_vertexBase;

			type = mesh.m_vertexType;

			//vertexStride = mesh.m_vertexStride;

			numfaces = mesh.m_numTriangles;
			( indexbase ) = mesh.m_triangleIndexBase;
			//indexstride = mesh.m_triangleIndexStride;
			indicestype = mesh.m_indexType;
		}

		public override bool hasPremadeAabb()
		{
			return ( m_hasAabb == true );
		}


		public override void setPremadeAabb( ref btVector3 aabbMin, ref btVector3 aabbMax )
		{
			m_aabbMin = aabbMin;
			m_aabbMax = aabbMax;
			m_hasAabb = true; // this is intentionally an int see notes in header
		}

		public override void getPremadeAabb( out btVector3 aabbMin, out btVector3 aabbMax )
		{
			aabbMin = m_aabbMin;
			aabbMax = m_aabbMax;
		}



	}


}