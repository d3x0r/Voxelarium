
/*
Stan Melax Convex Hull Computation
Copyright (c) 2008 Stan Melax http://www.melax.com/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
///includes modifications/improvements by John Ratcliff, see BringOutYourDead below.
/// 
using System.Collections.Generic;
using Bullet.Types;

namespace Bullet.LinearMath
{

	internal class TUIntArray : btList<uint>
	{
		public TUIntArray() { }
		public TUIntArray( int count ) : base( count ) { }
	}

	internal class HullResult
	{
		public HullResult()
		{
			mPolygons = true;
			mNumOutputVertices = 0;
			mNumFaces = 0;
			mNumIndices = 0;
		}
		public bool mPolygons;                  // true if indices represents polygons, false indices are triangles
		public int mNumOutputVertices;         // number of vertices in the output hull
		public btList<btVector3> m_OutputVertices = new btList<btVector3>();            // array of vertices
		public int mNumFaces;                  // the number of faces produced
		public int mNumIndices;                // the total number of indices
		public btList<uint> m_Indices = new btList<uint>();                   // pointer to indices.

		// If triangles, then indices are array indexes into the vertex list.
		// If polygons, indices are in the form (number of points in face) (p1, p2, p3, ..) etc..
	};

	internal enum HullFlag
	{
		QF_TRIANGLES = ( 1 << 0 ),             // report results as triangles, not polygons.
		QF_REVERSE_ORDER = ( 1 << 1 ),             // reverse order of the triangle indices.
		QF_DEFAULT = QF_TRIANGLES
	};


	internal class HullDesc
	{
		public HullDesc()
		{
			mFlags = HullFlag.QF_DEFAULT;
			mVcount = 0;
			mVertices = null;
			mVertexStride = 0;
			mNormalEpsilon = 0.001f;
			mMaxVertices = 4096; // maximum number of points to be considered for a convex hull.
			mMaxFaces = 4096;
		}

		public HullDesc( HullFlag flag,
			 int vcount,
			 btList<btVector3> vertices,
			 int stride = 0 )
		{
			mFlags = flag;
			mVcount = vcount;
			mVertices = vertices;
			mVertexStride = stride;
			mNormalEpsilon = 0.001;
			mMaxVertices = 4096;
		}

		public bool HasHullFlag( HullFlag flag )
		{
			return ( ( mFlags & flag ) != 0 );
		}

		public void SetHullFlag( HullFlag flag )
		{
			mFlags |= flag;
		}

		public void ClearHullFlag( HullFlag flag )
		{
			mFlags &= ~flag;
		}

		public HullFlag mFlags;           // flags to use when generating the convex hull.
		public int mVcount;          // number of vertices in the input point cloud
		public btList<btVector3> mVertices;        // the array of vertices.
		public int mVertexStride;    // the stride of each vertex, in bytes.
		public double mNormalEpsilon;   // the epsilon for removing duplicates.  This is a normalized value, if normalized bit is on.
		public int mMaxVertices;     // maximum number of vertices to be considered for the hull!
		public int mMaxFaces;
	};

	enum HullError
	{
		QE_OK,            // success!
		QE_FAIL           // failed.
	};

	public partial struct btPlane
	{
		public btVector3 normal;
		public double dist;   // distance below origin - the D from plane equasion Ax+By+Cz+D=0
		public btPlane( btVector3 n, double d )
		{
			normal = n;
			dist = d;
		}

		//inline 
		public static void PlaneFlip( ref btPlane plane, out btPlane result )
		{
			plane.normal.Invert( out result.normal );
			result.dist = -plane.dist;
		}
		//inline 
		public static bool Equals( ref btPlane a, ref btPlane b )
		{ return ( a.normal .Equals( b.normal ) && a.dist == b.dist ); }

		public bool Equals( ref btPlane b )
		{ return ( normal.Equals( b.normal ) && dist == b.dist ); }
		//inline 
		public static bool coplanar( ref btPlane a, ref btPlane b )
		{
			btPlane tmp;
			if( a.Equals( ref b ) )
				return true;

			PlaneFlip( ref b, out tmp );
			if( a.Equals( ref tmp ) )
				return true;
			return false;
		}

	};



	internal partial class ConvexH
	{
		public class HalfEdge
		{
			public short ea;         // the other half of the edge (index into edges list)
			public byte v;  // the vertex at the start of this edge (index into vertices list)
			public byte p;  // the facet on which this edge lies (index into facets list)
			public HalfEdge( short _ea, byte _v, byte _p )

			{ ea = _ea; v = _v; p = _p; }
		};

		public btList<btVector3> vertices;
		public btList<HalfEdge> edges;
		public btList<btPlane> facets;
		//public ConvexH( int vertices_size, int edges_size, int facets_size );
	};


	internal class int4
	{
		public int x, y, z, w;
		public int4( int _x, int _y, int _z, int _w ) { x = _x; y = _y; z = _z; w = _w; }
		public int this[int i]
		{
			get { switch( i ) { default: case 0: return x; case 1: return y; case 2: return z; case 3: return w; } }
			set { switch( i ) { default: case 0: x = value; break; case 1: y = value; break; case 2: z = value; break; case 3: w = value; break; } }
		}

	};

	internal class PHullResult
	{
		public PHullResult()
		{
			mVcount = 0;
			mIndexCount = 0;
			mFaceCount = 0;
			mVertices = null;
		}

		public int mVcount;
		public int mIndexCount;
		public int mFaceCount;
		public btList<btVector3> mVertices;
		public TUIntArray m_Indices;
	};



	///The HullLibrary class can create a convex hull from a collection of vertices, using the ComputeHull method.
	///The btShapeHull class uses this HullLibrary to create a approximate convex mesh given a general (non-polyhedral) convex shape.


}
