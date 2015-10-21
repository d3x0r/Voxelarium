//Until Bullet 2.77 a box approximation was used, so uncomment this if you need backwards compatibility
//#define USE_BOX_INERTIA_APPROXIMATION

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

	/// The btCylinderShape class implements a cylinder shape primitive, centered around the origin. Its central axis aligned with the Y axis. btCylinderShapeX is aligned with the X axis and btCylinderShapeZ around the Z axis.
	public class btCylinderShape : btConvexInternalShape
	{

		//btVector3 m_implicitShapeDimensionsWithMargin;
		protected int m_upAxis;


		public void getHalfExtentsWithMargin( out btVector3 halfExtents )
		{
			getHalfExtentsWithoutMargin( out halfExtents );
			btVector3 margin = new btVector3( getMargin(), getMargin(), getMargin() );
			halfExtents.Add( ref margin, out halfExtents );
		}

		public void getHalfExtentsWithoutMargin( out btVector3 result )
		{
			result = m_implicitShapeDimensions;//changed in Bullet 2.63: assume the scaling and margin are included
		}


		public override void setMargin( double collisionMargin )
		{
			//correct the m_implicitShapeDimensions for the margin
			btVector3 oldMargin = new btVector3( getMargin(), getMargin(), getMargin() );
			btVector3 implicitShapeDimensionsWithMargin; oldMargin.Add( ref m_implicitShapeDimensions, out implicitShapeDimensionsWithMargin );

			base.setMargin( collisionMargin );
			btVector3 newMargin = new btVector3( getMargin(), getMargin(), getMargin() );
			implicitShapeDimensionsWithMargin.Sub( ref newMargin, out m_implicitShapeDimensions );

		}

		public override void localGetSupportingVertex( ref btVector3 vec, out btVector3 supVertex )
		{

			localGetSupportingVertexWithoutMargin( ref vec, out supVertex );

			if( getMargin() != btScalar.BT_ZERO )
			{
				btVector3 vecnorm = vec;
				if( vecnorm.length2() < ( btScalar.SIMD_EPSILON * btScalar.SIMD_EPSILON ) )
				{
					vecnorm.setValue( (double)( -1.0 ), (double)( -1.0 ), (double)( -1.0 ) );
				}
				vecnorm.normalize();
				supVertex.AddScale( ref vecnorm, getMargin(), out supVertex );
			}
		}


		//use box inertia
		//	virtual void	calculateLocalInertia(double mass,ref btVector3 inertia);


		public int getUpAxis()
		{
			return m_upAxis;
		}

		public override void getAnisotropicRollingFrictionDirection( out btVector3 result )
		{
			result = btVector3.Zero;
			result[getUpAxis()] = 1;
		}

		public virtual double getRadius()
		{
			btVector3 bounds;
			getHalfExtentsWithMargin( out bounds );
			return bounds.x;
		}

		public override void setLocalScaling( ref btVector3 scaling )
		{
			btVector3 oldMargin = new btVector3( getMargin() );
			btVector3 implicitShapeDimensionsWithMargin; m_implicitShapeDimensions.Add( ref oldMargin, out implicitShapeDimensionsWithMargin );
			btVector3 unScaledImplicitShapeDimensionsWithMargin; implicitShapeDimensionsWithMargin.Div( ref m_localScaling, out unScaledImplicitShapeDimensionsWithMargin );

			base.setLocalScaling( ref scaling );
			//btVector3 tmp;
			unScaledImplicitShapeDimensionsWithMargin.Mult( ref m_localScaling, out unScaledImplicitShapeDimensionsWithMargin );
			unScaledImplicitShapeDimensionsWithMargin.Sub( ref oldMargin, out m_implicitShapeDimensions );
			//m_implicitShapeDimensions = ( unScaledImplicitShapeDimensionsWithMargin * m_localScaling ) - oldMargin;

		}

		//debugging
		public override string ToString()
		{
			return "CylinderY";
		}
#if SERIALIZE_DONE
		virtual int calculateSerializeBufferSize();

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual string serialize( object dataBuffer, btSerializer* serializer );
#endif

		public btCylinderShape( ref btVector3 halfExtents )
		{
			m_upAxis = ( 1 );
			setSafeMargin( ref halfExtents );

			btVector3 margin = new btVector3( getMargin() );
			btVector3 tmp;
			halfExtents.Mult( ref m_localScaling, out tmp );
			tmp.Sub( ref margin, out m_implicitShapeDimensions );
			//m_implicitShapeDimensions = ( halfExtents * m_localScaling ) - margin;
			m_shapeType = BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE;
		}



		public override void getAabb( ref btTransform t, out btVector3 aabbMin, out btVector3 aabbMax )
		{
			btAabbUtil.btTransformAabb( ref m_implicitShapeDimensions, getMargin(), ref t, out aabbMin, out aabbMax );
		}

		public override void calculateLocalInertia( double mass, out btVector3 inertia )
		{

			//Until Bullet 2.77 a box approximation was used, so uncomment this if you need backwards compatibility
			//#define USE_BOX_INERTIA_APPROXIMATION 1
#if !USE_BOX_INERTIA_APPROXIMATION

			/*
			cylinder is defined as following:
			*
			* - principle axis aligned along y by default, radius in x, z-value not used
			* - for btCylinderShapeX: principle axis aligned along x, radius in y direction, z-value not used
			* - for btCylinderShapeZ: principle axis aligned along z, radius in x direction, y-value not used
			*
			*/

			double radius2; // square of cylinder radius
			double height2; // square of cylinder height
			btVector3 halfExtents; getHalfExtentsWithMargin( out halfExtents ); // get cylinder dimension
			double div12 = mass / 12.0f;
			double div4 = mass / 4.0f;
			double div2 = mass / 2.0f;
			int idxRadius, idxHeight;

			switch( m_upAxis )  // get indices of radius and height of cylinder
			{
				case 0:     // cylinder is aligned along x
					idxRadius = 1;
					idxHeight = 0;
					break;
				case 2:     // cylinder is aligned along z
					idxRadius = 0;
					idxHeight = 2;
					break;
				default:    // cylinder is aligned along y
					idxRadius = 0;
					idxHeight = 1;
					break;
			}

			// calculate squares
			radius2 = halfExtents[idxRadius] * halfExtents[idxRadius];
			height2 = (double)( 4.0 ) * halfExtents[idxHeight] * halfExtents[idxHeight];

			// calculate tensor terms
			double t1 = div12 * height2 + div4 * radius2;
			double t2 = div2 * radius2;

			switch( m_upAxis )  // set diagonal elements of inertia tensor
			{
				case 0:     // cylinder is aligned along x
					btVector3.setValue( out inertia, t2, t1, t1 );
					break;
				case 2:     // cylinder is aligned along z
					btVector3.setValue( out inertia, t1, t1, t2 );
					break;
				default:    // cylinder is aligned along y
					btVector3.setValue( out inertia, t1, t2, t1 );
					break;
			}
#else //USE_BOX_INERTIA_APPROXIMATION
			//approximation of box shape
			btVector3 halfExtents; getHalfExtentsWithMargin( out halfExtents );

			double lx = (double)( 2.0) * ( halfExtents.x );
			double ly = (double)( 2.0) * ( halfExtents.y );
			double lz = (double)( 2.0) * ( halfExtents.z );

			inertia.setValue( mass / ( (double)( 12.0 ) ) * ( ly * ly + lz * lz ),
							mass / ( (double)( 12.0 ) ) * ( lx * lx + lz * lz ),
							mass / ( (double)( 12.0 ) ) * ( lx * lx + ly * ly ) );
#endif //USE_BOX_INERTIA_APPROXIMATION
		}

		public override void localGetSupportingVertexWithoutMargin( ref btVector3 dir, out btVector3 result )
		{
			CylinderLocalSupportY( ref m_implicitShapeDimensions, ref dir, out result );
		}

		public override void batchedUnitVectorGetSupportingVertexWithoutMargin( btVector3[] vectors, btVector3[] supportVerticesOut, int numVectors )
		{
			for( int i = 0; i < numVectors; i++ )
			{
				CylinderLocalSupportY( ref m_implicitShapeDimensions, ref vectors[i], out supportVerticesOut[i] );
			}
		}

		static void CylinderLocalSupportY( ref btVector3 halfExtents, ref btVector3 v, out btVector3 result )
		{

			int cylinderUpAxis = 1;
			int XX = 0;
			int YY = 1;
			int ZZ = 2;


			double radius = halfExtents[XX];
			double halfHeight = halfExtents[cylinderUpAxis];


			result = btVector3.Zero;
			double d;

			double s = btScalar.btSqrt( v[XX] * v[XX] + v[ZZ] * v[ZZ] );
			if( s != btScalar.BT_ZERO )
			{
				d = radius / s;
				result[XX] = v[XX] * d;
				result[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
				result[ZZ] = v[ZZ] * d;
			}
			else
			{
				result[XX] = radius;
				result[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
				result[ZZ] = btScalar.BT_ZERO;
			}

		}
	};

	public class btCylinderShapeX : btCylinderShape
	{
		//debugging
		public override string ToString()
		{
			return "CylinderX";
		}

		public btCylinderShapeX( ref btVector3 halfExtents ) : base( ref halfExtents )
		{
			m_upAxis = 0;
		}


		public override void batchedUnitVectorGetSupportingVertexWithoutMargin( btVector3[] vectors, btVector3[] supportVerticesOut, int numVectors )
		{
			for( int i = 0; i < numVectors; i++ )
			{
				CylinderLocalSupportX( ref m_implicitShapeDimensions, ref vectors[i], out supportVerticesOut[i] );
			}
		}


		public override double getRadius()
		{
			btVector3 tmp;
			getHalfExtentsWithMargin( out tmp );
			return tmp.y;
		}
		public void CylinderLocalSupportX( ref btVector3 halfExtents, ref btVector3 v, out btVector3 result )
		{
			int cylinderUpAxis = 0;
			int XX = 1;
			int YY = 0;
			int ZZ = 2;

			//mapping depends on how cylinder local orientation is
			// extents of the cylinder is: X,Y is for radius, and Z for height


			double radius = halfExtents[XX];
			double halfHeight = halfExtents[cylinderUpAxis];

			result = btVector3.Zero;
			double d;

			double s = btScalar.btSqrt( v[XX] * v[XX] + v[ZZ] * v[ZZ] );
			if( s != btScalar.BT_ZERO )
			{
				d = radius / s;
				result[XX] = v[XX] * d;
				result[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
				result[ZZ] = v[ZZ] * d;
			}
			else
			{
				result[XX] = radius;
				result[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
				result[ZZ] = btScalar.BT_ZERO;
			}
		}

		public override void localGetSupportingVertexWithoutMargin( ref btVector3 vec, out btVector3 result )
		{
			CylinderLocalSupportX( ref m_implicitShapeDimensions, ref vec, out result );
		}
	};

	class btCylinderShapeZ : btCylinderShape
	{

		//debugging
		public override string ToString()
		{
			return "CylinderZ";
		}
		public btCylinderShapeZ( ref btVector3 halfExtents ) : base( ref halfExtents )
		{
			m_upAxis = 2;

		}

		public override double getRadius()
		{
			btVector3 tmp;
			getHalfExtentsWithMargin( out tmp );
			return tmp.x;
		}

		public override void batchedUnitVectorGetSupportingVertexWithoutMargin( btVector3[] vectors, btVector3[] supportVerticesOut, int numVectors )
		{
			for( int i = 0; i < numVectors; i++ )
			{
				CylinderLocalSupportZ( ref m_implicitShapeDimensions, ref vectors[i], out supportVerticesOut[i] );
			}
		}



		static void CylinderLocalSupportZ( ref btVector3 halfExtents, ref btVector3 v, out btVector3 result )
		{
			int cylinderUpAxis = 2;
			int XX = 0;
			int YY = 2;
			int ZZ = 1;

			//mapping depends on how cylinder local orientation is
			// extents of the cylinder is: X,Y is for radius, and Z for height


			double radius = halfExtents[XX];
			double halfHeight = halfExtents[cylinderUpAxis];

			result = btVector3.Zero;
			double d;

			double s = btScalar.btSqrt( v[XX] * v[XX] + v[ZZ] * v[ZZ] );
			if( s != btScalar.BT_ZERO )
			{
				d = radius / s;
				result[XX] = v[XX] * d;
				result[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
				result[ZZ] = v[ZZ] * d;
			}
			else
			{
				result[XX] = radius;
				result[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
				result[ZZ] = btScalar.BT_ZERO;
			}


		}



		public override void localGetSupportingVertexWithoutMargin( ref btVector3 vec, out btVector3 result )
		{
			CylinderLocalSupportZ( ref m_implicitShapeDimensions, ref vec, out result );
		}



	};

#if asdfasdf
///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btCylinderShapeData
{
	btConvexInternalShapeData	m_convexInternalShapeData;

	int	m_upAxis;

	char	m_padding[4];
};

public	int	btCylinderShape::calculateSerializeBufferSize()
{
	return sizeof(btCylinderShapeData);
}

	///fills the dataBuffer and returns the struct name (and 0 on failure)
public	string	btCylinderShape::serialize(object dataBuffer, btSerializer* serializer)
{
	btCylinderShapeData* shapeData = (btCylinderShapeData*) dataBuffer;
	
	btConvexInternalShape::serialize(&shapeData.m_convexInternalShapeData,serializer);

	shapeData.m_upAxis = m_upAxis;
	
	return "btCylinderShapeData";
}
#endif

}