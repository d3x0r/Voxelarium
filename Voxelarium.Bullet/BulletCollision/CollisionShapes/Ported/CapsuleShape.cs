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

namespace Bullet.Collision.Shapes
{


	///The btCapsuleShape represents a capsule around the Y axis, there is also the btCapsuleShapeX aligned around the X axis and btCapsuleShapeZ around the Z axis.
	///The total height is height+2*radius, so the height is just the height between the center of each 'sphere' of the capsule caps.
	///The btCapsuleShape is a convex hull of two spheres. The btMultiSphereShape is a more general collision shape that takes the convex hull of multiple sphere, so it can also represent a capsule when just using two spheres.
	public class btCapsuleShape : btConvexInternalShape
	{
		protected int m_upAxis;

		///only used for btCapsuleShapeZ and btCapsuleShapeX subclasses.
		protected btCapsuleShape()
		{
			m_shapeType = BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE;
		}

		public override void setMargin( double collisionMargin )
		{
			//correct the m_implicitShapeDimensions for the margin
			btVector3 oldMargin = new btVector3( getMargin(), getMargin(), getMargin() );
			btVector3 implicitShapeDimensionsWithMargin;
			m_implicitShapeDimensions.Add( ref oldMargin, out implicitShapeDimensionsWithMargin );

			base.setMargin( collisionMargin );
			btVector3 newMargin = new btVector3( getMargin(), getMargin(), getMargin() );
			implicitShapeDimensionsWithMargin.Sub( ref newMargin, out m_implicitShapeDimensions );

		}

		public override void getAabb( ref btTransform t, out btVector3 aabbMin, out btVector3 aabbMax )
		{
			btVector3 halfExtents = new btVector3( getRadius(), getRadius(), getRadius() );
			btVector3 tmp = halfExtents;
			halfExtents[m_upAxis] = getRadius() + getHalfHeight();

			halfExtents.Add( ref tmp, out halfExtents );

			btMatrix3x3 abs_b; t.m_basis.absolute( out abs_b );
			btVector3 extent; halfExtents.dot3( ref abs_b.m_el0, ref abs_b.m_el1, ref abs_b.m_el2, out extent );

			t.m_origin.Sub( ref extent, out aabbMin );
			t.m_origin.Add( ref extent, out aabbMax );
		}

		public override string ToString()
		{
			return "CapsuleShape";
		}

		public int getUpAxis()
		{
			return m_upAxis;
		}

		public double getRadius()
		{
			int radiusAxis = ( m_upAxis + 2 ) % 3;
			return m_implicitShapeDimensions[radiusAxis];
		}

		public double getHalfHeight()
		{
			return m_implicitShapeDimensions[m_upAxis];
		}

		public override void setLocalScaling( ref btVector3 scaling )
		{
			btVector3 oldMargin = new btVector3( getMargin(), getMargin(), getMargin() );
			btVector3 implicitShapeDimensionsWithMargin;
			m_implicitShapeDimensions.Add( ref oldMargin, out implicitShapeDimensionsWithMargin );
			btVector3 unScaledImplicitShapeDimensionsWithMargin;
			implicitShapeDimensionsWithMargin.Div( ref m_localScaling, out unScaledImplicitShapeDimensionsWithMargin );

			base.setLocalScaling( ref scaling );
			unScaledImplicitShapeDimensionsWithMargin.Mult( ref m_localScaling, out unScaledImplicitShapeDimensionsWithMargin );
            unScaledImplicitShapeDimensionsWithMargin.Sub(ref oldMargin, out m_implicitShapeDimensions );

		}

		public override void getAnisotropicRollingFrictionDirection( out btVector3 result )
		{
			result = btVector3.Zero;
			result[getUpAxis()] = 1;
		}



		public btCapsuleShape( double radius, double height )
		{
			m_shapeType = BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE;
			m_upAxis = 1;
			m_implicitShapeDimensions.setValue( radius, 0.5f * height, radius );
		}


		public override void localGetSupportingVertexWithoutMargin( ref btVector3 vec0, out btVector3 result )
		{
			result = btVector3.Zero;

			double maxDot = double.MinValue;

			btVector3 vec = vec0;
			double lenSqr = vec.length2();
			if( lenSqr < (double)( 0.0001 ) )
			{
				vec.setValue( 1, 0, 0 );
			}
			else
			{
				double rlen = (double)( 1.0) / btScalar.btSqrt( lenSqr );
				vec.Mult( rlen, out vec );
			}

			btVector3 vtx;
			double newDot;

			double radius = getRadius();

			btVector3 vec_rad;
			btVector3 vec_marg;
			vec.Mult( radius, out vec_rad );
			vec.Mult( getMargin(), out vec_marg );
			{
				btVector3 pos = btVector3.Zero;
				pos[getUpAxis()] = getHalfHeight();

				pos.Add( ref vec_rad, out vtx );
				vtx.Sub( ref vec_marg, out vtx );
				newDot = vec.dot( ref vtx );
				if( newDot > maxDot )
				{
					maxDot = newDot;
					result = vtx;
				}
			}
			{
				btVector3 pos = btVector3.Zero;
				pos[getUpAxis()] = -getHalfHeight();
				pos.Add( ref vec_rad, out vtx );
				vtx.Sub( ref vec_marg, out vtx );
				//vtx = pos + vec * ( radius ) - vec * getMargin();
				newDot = vec.dot( ref vtx );
				if( newDot > maxDot )
				{
					maxDot = newDot;
					result = vtx;
				}
			}


		}

		public override void batchedUnitVectorGetSupportingVertexWithoutMargin( btVector3[] vectors, btVector3[] supportVerticesOut, int numVectors )
		{


			double radius = getRadius();

			for( int j = 0; j < numVectors; j++ )
			{
				double maxDot= double.MinValue;
				btVector3 vec = vectors[j];

				btVector3 vtx;
				double newDot;
				btVector3 vec_rad;
				btVector3 vec_marg;
				vec.Mult( radius, out vec_rad );
				vec.Mult( getMargin(), out vec_marg );
				{
					btVector3 pos = btVector3.Zero;
					pos[getUpAxis()] = getHalfHeight();
					pos.Add( ref vec_rad, out vtx );
					vtx.Sub( ref vec_marg, out vtx );
					//vtx = pos + vec * ( radius ) - vec * getMargin();
					newDot = vec.dot( ref vtx );
					if( newDot > maxDot )
					{
						maxDot = newDot;
						supportVerticesOut[j] = vtx;
					}
				}
				{
					btVector3 pos = btVector3.Zero;
					pos[getUpAxis()] = -getHalfHeight();
					pos.Add( ref vec_rad, out vtx );
					vtx.Sub( ref vec_marg, out vtx );
					//vtx = pos + vec * ( radius ) - vec * getMargin();
					newDot = vec.dot( ref vtx );
					if( newDot > maxDot )
					{
						maxDot = newDot;
						supportVerticesOut[j] = vtx;
					}
				}
			}
		}


		public override void calculateLocalInertia( double mass, out btVector3 inertia )
		{
			//as an approximation, take the inertia of the box that bounds the spheres
			//btTransform ident = btTransform.Identity;

			double radius = getRadius();

			btVector3 halfExtents = new btVector3( radius, radius, radius );
			halfExtents[getUpAxis()] += getHalfHeight();

			double margin = btConvexInternalShape.CONVEX_DISTANCE_MARGIN;

			double lx = (double)( 2.0) * ( halfExtents[0] + margin );
			double ly = (double)( 2.0) * ( halfExtents[1] + margin );
			double lz = (double)( 2.0) * ( halfExtents[2] + margin );
			double x2 = lx * lx;
			double y2 = ly * ly;
			double z2 = lz * lz;
			double scaledmass = mass * (double)( .08333333 );
			btVector3.setValue( out inertia, 
						scaledmass * ( y2 + z2 ),
						scaledmass * ( x2 + z2 ),
						scaledmass * ( x2 + y2 ) );

		}



	};

	///btCapsuleShapeX represents a capsule around the Z axis
	///the total height is height+2*radius, so the height is just the height between the center of each 'sphere' of the capsule caps.
	class btCapsuleShapeX : btCapsuleShape
	{

		public btCapsuleShapeX( double radius, double height )
		{
			m_upAxis = 0;
			m_implicitShapeDimensions.setValue( 0.5f * height, radius, radius );
		}

		//debugging
		public override string ToString()
		{
			return "CapsuleX";
		}

	};

	///btCapsuleShapeZ represents a capsule around the Z axis
	///the total height is height+2*radius, so the height is just the height between the center of each 'sphere' of the capsule caps.
	public class btCapsuleShapeZ : btCapsuleShape
	{

		public btCapsuleShapeZ( double radius, double height )
		{
			m_upAxis = 2;
			m_implicitShapeDimensions.setValue( radius, radius, 0.5f * height );
		}
		//debugging
		public override string ToString()
		{
			return "CapsuleZ";
		}


	};

#if SERIALIZE_DONE
///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btCapsuleShapeData
{
	btConvexInternalShapeData m_convexInternalShapeData;

	int m_upAxis;

	char m_padding[4];
};

public int btCapsuleShape::calculateSerializeBufferSize()
{
	return sizeof( btCapsuleShapeData );
}




///fills the dataBuffer and returns the struct name (and 0 on failure)
public string btCapsuleShape::serialize( object dataBuffer, btSerializer* serializer )
{
	btCapsuleShapeData* shapeData = (btCapsuleShapeData*)dataBuffer;

	btConvexInternalShape::serialize( &shapeData.m_convexInternalShapeData, serializer );

	shapeData.m_upAxis = m_upAxis;

	return "btCapsuleShapeData";
}

public void deSerializeFloat( btCapsuleShapeData* dataBuffer )
{
	m_implicitShapeDimensions.deSerializeFloat( dataBuffer.m_convexInternalShapeData.m_implicitShapeDimensions );
	m_collisionMargin = dataBuffer.m_convexInternalShapeData.m_collisionMargin;
	m_localScaling.deSerializeFloat( dataBuffer.m_convexInternalShapeData.m_localScaling );
	//it is best to already pre-allocate the matching btCapsuleShape*(X/Z) version to match m_upAxis
	m_upAxis = dataBuffer.m_upAxis;
}
#endif
}
