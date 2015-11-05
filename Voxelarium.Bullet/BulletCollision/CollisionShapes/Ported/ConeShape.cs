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

	///The btConeShape implements a cone shape primitive, centered around the origin and aligned with the Y axis. The btConeShapeX is aligned around the X axis and btConeShapeZ around the Z axis.
	public class btConeShape : btConvexInternalShape
	{

		double m_sinAngle;
		double m_radius;
		double m_height;
		int[] m_coneIndices = new int[3];


		public double getRadius() { return m_radius; }
		public double getHeight() { return m_height; }

		public override void calculateLocalInertia( double mass, out btVector3 inertia )
		{
			btVector3 aabbMin, aabbMax;
			getAabb( btTransform.Identity, out aabbMin, out aabbMax );
			btVector3 tmp;
			aabbMax.Sub( ref aabbMin, out tmp );
			btVector3 halfExtents;
			tmp.Mult( btScalar.BT_HALF, out halfExtents );

			double margin = getMargin();

			double lx = (double)( 2.0 ) * ( halfExtents.x + margin );
			double ly = (double)( 2.0 ) * ( halfExtents.y + margin );
			double lz = (double)( 2.0 ) * ( halfExtents.z + margin );
			double x2 = lx * lx;
			double y2 = ly * ly;
			double z2 = lz * lz;
			double scaledmass = mass * (double)( 0.08333333 );

			tmp = new btVector3( y2 + z2, x2 + z2, x2 + y2 );
			tmp.Mult( scaledmass, out inertia );
		}

		public override string ToString()
		{
			return "Cone";
		}


		public int getConeUpIndex()
		{
			return m_coneIndices[1];
		}

		public override void getAnisotropicRollingFrictionDirection( out btVector3 result )
		{
			result = btVector3.yAxis;
		}



		public btConeShape( double radius, double height )
		{
			m_radius = ( radius );
			m_height = ( height );
			m_shapeType = BroadPhase.BroadphaseNativeTypes.CONE_SHAPE_PROXYTYPE;
			setConeUpIndex( 1 );
			//btVector3 halfExtents;
			m_sinAngle = ( m_radius / btScalar.btSqrt( m_radius * m_radius + m_height * m_height ) );
		}

		///choose upAxis index
		internal void setConeUpIndex( int upIndex )
		{
			switch( upIndex )
			{
				case 0:
					m_coneIndices[0] = 1;
					m_coneIndices[1] = 0;
					m_coneIndices[2] = 2;
					break;
				case 1:
					m_coneIndices[0] = 0;
					m_coneIndices[1] = 1;
					m_coneIndices[2] = 2;
					break;
				case 2:
					m_coneIndices[0] = 0;
					m_coneIndices[1] = 2;
					m_coneIndices[2] = 1;
					break;
				default:
					Debug.Assert( false );
					break;
			};

			m_implicitShapeDimensions[m_coneIndices[0]] = m_radius;
			m_implicitShapeDimensions[m_coneIndices[1]] = m_height;
			m_implicitShapeDimensions[m_coneIndices[2]] = m_radius;
		}

		void coneLocalSupport( ref btVector3 v, out btVector3 result )
		{
			btVector3 tmp = btVector3.Zero;

			double halfHeight = m_height * (double)( 0.5 );

			if( v[m_coneIndices[1]] > v.length() * m_sinAngle )
			{

				tmp[m_coneIndices[0]] = btScalar.BT_ZERO;
				tmp[m_coneIndices[1]] = halfHeight;
				tmp[m_coneIndices[2]] = btScalar.BT_ZERO;
				result = tmp;
			}
			else
			{
				double s = btScalar.btSqrt( v[m_coneIndices[0]] * v[m_coneIndices[0]] + v[m_coneIndices[2]] * v[m_coneIndices[2]] );
				if( s > btScalar.SIMD_EPSILON )
				{
					double d = m_radius / s;
					tmp[m_coneIndices[0]] = v[m_coneIndices[0]] * d;
					tmp[m_coneIndices[1]] = -halfHeight;
					tmp[m_coneIndices[2]] = v[m_coneIndices[2]] * d;
					result = tmp;
				}
				else
				{
					tmp[m_coneIndices[0]] = btScalar.BT_ZERO;
					tmp[m_coneIndices[1]] = -halfHeight;
					tmp[m_coneIndices[2]] = btScalar.BT_ZERO;
					result = tmp;
				}
			}

		}

		public override void localGetSupportingVertexWithoutMargin( ref btVector3 vec, out btVector3 result )
		{
			coneLocalSupport( ref vec, out result );
		}

		public override void batchedUnitVectorGetSupportingVertexWithoutMargin( btVector3[] vectors, btVector3[] supportVerticesOut, int numVectors )
		{
			for( int i = 0; i < numVectors; i++ )
			{
				 coneLocalSupport( ref vectors[i], out supportVerticesOut[i] );
			}
		}


		public override void localGetSupportingVertex( ref btVector3 vec, out btVector3 result )
		{
			coneLocalSupport( ref vec, out result );
			if( getMargin() != btScalar.BT_ZERO )
			{
				btVector3 vecnorm = vec;
				if( vecnorm.length2() < ( btScalar.SIMD_EPSILON * btScalar.SIMD_EPSILON ) )
				{
					vecnorm = btVector3.NegOne;
				}
				vecnorm.normalize();
				vecnorm.Mult( getMargin(), out vecnorm );
				result.Add( ref vecnorm, out result );
			}
		}


		public override void setLocalScaling( ref btVector3 scaling )
		{
			int axis = m_coneIndices[1];
			int r1 = m_coneIndices[0];
			int r2 = m_coneIndices[2];
			m_height *= scaling[axis] / m_localScaling[axis];
			m_radius *= ( scaling[r1] / m_localScaling[r1] + scaling[r2] / m_localScaling[r2] ) / 2;
			m_sinAngle = ( m_radius / btScalar.btSqrt( m_radius * m_radius + m_height * m_height ) );
			base.setLocalScaling( ref scaling );
		}

#if SERIALIZE_DONE
///fills the dataBuffer and returns the struct name (and 0 on failure)
virtual	string	serialize(object dataBuffer, btSerializer* serializer);
///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btConeShapeData
{
	btConvexInternalShapeData m_convexInternalShapeData;

	int m_upIndex;

	char m_padding[4];
};

public int calculateSerializeBufferSize()
{
	return sizeof( btConeShapeData );
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
public string serialize( object dataBuffer, btSerializer* serializer )
{
	btConeShapeData* shapeData = (btConeShapeData*)dataBuffer;

	btConvexInternalShape::serialize( &shapeData.m_convexInternalShapeData, serializer );

	shapeData.m_upIndex = m_coneIndices[1];

	return "btConeShapeData";
}
	
#endif




	};

	///btConeShape implements a Cone shape, around the X axis
	public class btConeShapeX : btConeShape
	{
		public override void getAnisotropicRollingFrictionDirection( out btVector3 result )
		{
			result = btVector3.xAxis;
		}

		public btConeShapeX( double radius, double height ) : base( radius, height )
		{
			setConeUpIndex( 0 );
		}

		//debugging
		public override string ToString()
		{
			return "ConeX";
		}


	};

	///btConeShapeZ implements a Cone shape, around the Z axis
	public class btConeShapeZ : btConeShape
	{

		public override void getAnisotropicRollingFrictionDirection( out btVector3 result )
		{
			result = btVector3.zAxis;
		}

		//debugging
		public override string ToString()
		{
			return "ConeZ";
		}

		public btConeShapeZ( double radius, double height ) : base( radius, height )
		{
			setConeUpIndex( 2 );
		}

	};

}


