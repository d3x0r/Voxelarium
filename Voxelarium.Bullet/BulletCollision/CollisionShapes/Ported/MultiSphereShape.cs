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
using Bullet.Types;

namespace Bullet.Collision.Shapes
{


	///The btMultiSphereShape represents the convex hull of a collection of spheres. You can create special capsules or other smooth volumes.
	///It is possible to animate the spheres for deformation, but call 'recalcLocalAabb' after changing any sphere position/radius
	public class btMultiSphereShape : btConvexInternalAabbCachingShape
	{

		btList<btVector3> m_localPositionArray;
		btList<double> m_radiArray;

		internal int getSphereCount()
		{
			return m_localPositionArray.Count;
		}

		public btIVector3 getSpherePosition( int index )
		{
			return m_localPositionArray[index];
		}

		public double getSphereRadius( int index )
		{
			return m_radiArray[index];
		}


		public override string ToString()
		{
			return "MultiSphere";
		}

		public btMultiSphereShape( btVector3[] positions, double[] radi, int numSpheres )
					: base()
		{
			m_shapeType = BroadphaseNativeTypes.MULTI_SPHERE_SHAPE_PROXYTYPE;
			//double startMargin = (double)(BT_LARGE_FLOAT);

			m_localPositionArray.Count = 0;
			m_localPositionArray.Capacity = positions.Length;
			m_radiArray.Count = 0;
			m_radiArray.Capacity = radi.Length;
			for( int i = 0; i < numSpheres; i++ )
			{
				m_localPositionArray.Add( positions[i] );
				m_radiArray.Add( radi[i] );
			}

			recalcLocalAabb();

		}

		public override void localGetSupportingVertexWithoutMargin( ref btVector3 vec0, out btVector3 result )
		{
			btVector3 supVec = btVector3.Zero;

			double maxDot = btScalar.BT_MIN_FLOAT;


			btVector3 vec = vec0;
			double lenSqr = vec.length2();
			if( lenSqr < ( btScalar.SIMD_EPSILON * btScalar.SIMD_EPSILON ) )
			{
				vec.setValue( 1, 0, 0 );
			}
			else
			{
				double rlen = btScalar.BT_ONE / btScalar.btSqrt( lenSqr );
				vec *= rlen;
			}

			btVector3 vtx;
			double newDot;

			btVector3[] pos = m_localPositionArray.InternalArray;
			double[] rad = m_radiArray.InternalArray;
			int numSpheres = m_localPositionArray.Count;

			for( int k = 0; k < numSpheres; k += 128 )
			{
				btVector3[] temp = new btVector3[128];
				int inner_count = btScalar.btMin( numSpheres - k, 128 );
				for( long i = 0; i < inner_count; i++ )
				{
					temp[i] = ( pos[i] ) + vec * m_localScaling * ( rad[i] ) - vec * getMargin();
				}
				long i_tmp = vec.maxDot( temp, inner_count, out newDot );
				if( newDot > maxDot )
				{
					maxDot = newDot;
					supVec = temp[i_tmp];
				}
			}
			result = supVec;

		}

		public override void batchedUnitVectorGetSupportingVertexWithoutMargin( btVector3[] vectors, btVector3[] supportVerticesOut, int numVectors )
		{

			for( int j = 0; j < numVectors; j++ )
			{
				double maxDot = btScalar.BT_MIN_FLOAT;

				btVector3 vec = vectors[j];

				btVector3 vtx;
				double newDot;

				btVector3[] pos = m_localPositionArray.InternalArray;
				double[] rad = m_radiArray.InternalArray;
				int numSpheres = m_localPositionArray.Count;

				for( int k = 0; k < numSpheres; k += 128 )
				{
					btVector3[] temp = new btVector3[128];
					int inner_count = btScalar.btMin( numSpheres - k, 128 );
					for( int i = 0; i < inner_count; i++ )
					{
						temp[i] = ( pos[i] ) + vec * m_localScaling * ( rad[i] ) - vec * getMargin();
					}
					long i_tmp = vec.maxDot( temp, inner_count, out newDot );
					if( newDot > maxDot )
					{
						maxDot = newDot;
						supportVerticesOut[j] = temp[i_tmp];
					}
				}

			}
		}



		public override void calculateLocalInertia( double mass, out btVector3 inertia )
		{
			//as an approximation, take the inertia of the box that bounds the spheres

			btVector3 localAabbMin, localAabbMax;
			getCachedLocalAabb( out localAabbMin, out localAabbMax );
			btVector3 halfExtents = ( localAabbMax - localAabbMin ) * (double)( 0.5 );

			double lx = btScalar.BT_TWO * ( halfExtents.x );
			double ly = btScalar.BT_TWO * ( halfExtents.y );
			double lz = btScalar.BT_TWO * ( halfExtents.z );

			inertia = new btVector3( mass / ( (double)( 12.0 ) ) * ( ly * ly + lz * lz ),
							mass / ( (double)( 12.0 ) ) * ( lx * lx + lz * lz ),
							mass / ( (double)( 12.0 ) ) * ( lx * lx + ly * ly ) );

		}

#if SERIALIZE_DONE
		///fills the dataBuffer and returns the struct name (and 0 on failure)
		string btMultiSphereShape::serialize( object dataBuffer, btSerializer* serializer )
		{
			btMultiSphereShapeData* shapeData = (btMultiSphereShapeData*)dataBuffer;
			btConvexInternalShape::serialize( shapeData.m_convexInternalShapeData, serializer );

			int numElem = m_localPositionArray.Count;
			shapeData.m_localPositionArrayPtr = numElem ? (btPositionAndRadius*)serializer.getUniquePointer( (object)&m_localPositionArray ) : 0;

			shapeData.m_localPositionArraySize = numElem;
			if( numElem )
			{
				btChunk* chunk = serializer.allocate( sizeof( btPositionAndRadius ), numElem );
				btPositionAndRadius* memPtr = (btPositionAndRadius*)chunk.m_oldPtr;
				for( int i = 0; i < numElem; i++, memPtr++ )
				{
					m_localPositionArray[i].serializeFloat( memPtr.m_pos );
					memPtr.m_radius = float( m_radiArray[i] );
				}
				serializer.finalizeChunk( chunk, "btPositionAndRadius", BT_ARRAY_CODE, (object)m_localPositionArray );
			}

			return "btMultiSphereShapeData";
		}


		virtual int calculateSerializeBufferSize();

		///fills the dataBuffer and returns the struct name (and 0 on failure)
		virtual string serialize( object dataBuffer, btSerializer* serializer );

#endif
	};

#if SERIALIZE_DONE
	struct btPositionAndRadius
	{
		btVector3FloatData m_pos;
		float m_radius;
	};

	struct btMultiSphereShapeData
	{
		btConvexInternalShapeData m_convexInternalShapeData;

		btPositionAndRadius* m_localPositionArrayPtr;
		int m_localPositionArraySize;
		char m_padding[4];
	};



	public int btMultiSphereShape::calculateSerializeBufferSize()
	{
		return sizeof( btMultiSphereShapeData );
	}
#endif

}

