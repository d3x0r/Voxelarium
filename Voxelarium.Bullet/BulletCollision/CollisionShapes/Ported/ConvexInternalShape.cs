#define NORMALLY_SET
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


	///The btConvexInternalShape is an internal base class, shared by most convex shape implementations.
	///The btConvexInternalShape uses a default collision margin set to CONVEX_DISTANCE_MARGIN.
	///This collision margin used by Gjk and some other algorithms, see also btCollisionMargin.h
	///Note that when creating small shapes (derived from btConvexInternalShape), 
	///you need to make sure to set a smaller collision margin, using the 'setMargin' API
	///There is a automatic mechanism 'setSafeMargin' used by btBoxShape and btCylinderShape
	public abstract class btConvexInternalShape : btConvexShape
	{
		///The CONVEX_DISTANCE_MARGIN is a default collision margin for convex collision shapes derived from btConvexInternalShape.
		///This collision margin is used by Gjk and some other algorithms
		///Note that when creating small objects, you need to make sure to set a smaller collision margin, using the 'setMargin' API
		public const double CONVEX_DISTANCE_MARGIN = (double)0.04;// double(0.1)//;//double(0.01)


		//local scaling. collisionMargin is not scaled !
		protected btVector3 m_localScaling;

		protected btVector3 m_implicitShapeDimensions;

		protected double m_collisionMargin;

		protected double m_padding;

		public btVector3 getImplicitShapeDimensions()
		{
			return m_implicitShapeDimensions;
		}

		///warning: use setImplicitShapeDimensions with care
		///changing a collision shape while the body is in the world is not recommended,
		///it is best to remove the body from the world, then make the change, and re-add it
		///alternatively flush the contact points, see documentation for 'cleanProxyFromPairs'
		public void setImplicitShapeDimensions( ref btVector3 dimensions )
		{
			m_implicitShapeDimensions = dimensions;
		}

		public void setSafeMargin( double minDimension, double defaultMarginMultiplier = 0.1f )
		{
			double safeMargin = defaultMarginMultiplier * minDimension;
			if( safeMargin < getMargin() )
			{
				setMargin( safeMargin );
			}
		}
		public void setSafeMargin( ref btVector3 halfExtents, double defaultMarginMultiplier = 0.1f )
		{
			//see http://code.google.com/p/bullet/issues/detail?id=349
			//this margin check could could be added to other collision shapes too,
			//or add some assert/warning somewhere
			double minDimension = halfExtents[halfExtents.minAxis()];
			setSafeMargin( minDimension, defaultMarginMultiplier );
		}

		///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
		public override void getAabb( ref btTransform t, out btVector3 aabbMin, out btVector3 aabbMax )
		{
			getAabbSlow( ref t, out aabbMin, out aabbMax );
		}



		public override void getLocalScaling( out btVector3 result )
		{
			result = m_localScaling;
		}

		public void getLocalScalingNV( out btVector3 result )
		{
			result = m_localScaling;
		}

		public override void setMargin( double margin )
		{
			m_collisionMargin = margin;
		}
		public override double getMargin()
		{
			return m_collisionMargin;
		}

		public double getMarginNV()
		{
			return m_collisionMargin;
		}

		public override int getNumPreferredPenetrationDirections()
		{
			return 0;
		}

		public override void getPreferredPenetrationDirection( int index, out btVector3 penetrationVector )
		{
			penetrationVector = btVector3.Zero;
			//(void)penetrationVector;
			//(void)index;
			Debug.Assert( false );
		}


		public btConvexInternalShape()
		{
			m_localScaling = new btVector3( (double)( 1.0 ), (double)( 1.0 ), (double)( 1.0 ) );
			m_collisionMargin = ( CONVEX_DISTANCE_MARGIN );
		}


		public override void setLocalScaling( ref btVector3 scaling )
		{
			m_localScaling = scaling.absolute();
		}



		public override void getAabbSlow( ref btTransform trans, out btVector3 minAabb, out btVector3 maxAabb )
		{
			//use localGetSupportingVertexWithoutMargin?
			double margin = getMargin();
			double[] max = new double[3];
			double[] min = new double[3];
			for( int i = 0; i < 3; i++ )
			{
				btVector3 vec = btVector3.Zero;
				vec[i] = (double)( 1.0 );
				btVector3 tmp1;
				trans.m_basis.ApplyInverse( ref vec, out tmp1 );
				btVector3 sv; localGetSupportingVertex( ref tmp1, out sv );

				btVector3 tmp; trans.Apply( ref sv, out tmp );
				max[i] = tmp[i] + margin;
				vec[i] = (double)( -1.0 );
				trans.m_basis.ApplyInverse( ref vec, out tmp1 );
				localGetSupportingVertex( ref tmp1, out tmp );
				trans.Apply( ref tmp, out tmp1 );
				min[i] = tmp1[i] - margin;
			}
			maxAabb = new btVector3( max[0], max[1], max[2] );
			minAabb = new btVector3( min[0], min[1], min[2] );
		}

		public override void localGetSupportingVertex( ref btVector3 vec, out btVector3 result )
		{

			localGetSupportingVertexWithoutMargin( ref vec, out result );

			if( getMargin() != btScalar.BT_ZERO )
			{
				btVector3 vecnorm = vec;
				if( vecnorm.length2() < ( btScalar.SIMD_EPSILON * btScalar.SIMD_EPSILON ) )
				{
					vecnorm.setValue( (double)( -1.0 ), (double)( -1.0 ), (double)( -1.0 ) );
				}
				vecnorm.normalize();
				result.AddScale( ref vecnorm, getMargin(), out result );
			}
			//return supVertex;


		}


	};

#if SERIALIZE_DONE
	///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
	struct btConvexInternalShapeData
	{
		btCollisionShapeData m_collisionShapeData;

		btVector3FloatData m_localScaling;

		btVector3FloatData m_implicitShapeDimensions;

		float m_collisionMargin;

		int m_padding;

	};



	public virtual int calculateSerializeBufferSize()
	{
		return sizeof( btConvexInternalShapeData );
	}

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	public virtual string serialize( object dataBuffer, btSerializer* serializer )
	{
		btConvexInternalShapeData* shapeData = (btConvexInternalShapeData*)dataBuffer;
		btCollisionShape::serialize( &shapeData.m_collisionShapeData, serializer );

		m_implicitShapeDimensions.serializeFloat( shapeData.m_implicitShapeDimensions );
		m_localScaling.serializeFloat( shapeData.m_localScaling );
		shapeData.m_collisionMargin = float( m_collisionMargin );

		return "btConvexInternalShapeData";
	}


#endif

	///btConvexInternalAabbCachingShape adds local aabb caching for convex shapes, to avoid expensive bounding box calculations
	public abstract class btConvexInternalAabbCachingShape : btConvexInternalShape
	{
		btVector3 m_localAabbMin;
		btVector3 m_localAabbMax;
		bool m_isLocalAabbValid;

		protected void setCachedLocalAabb( ref btVector3 aabbMin, ref btVector3 aabbMax )
		{
			m_isLocalAabbValid = true;
			m_localAabbMin = aabbMin;
			m_localAabbMax = aabbMax;
		}

		protected void getCachedLocalAabb( out btVector3 aabbMin, out btVector3 aabbMax )
		{
			Debug.Assert( m_isLocalAabbValid );
			aabbMin = m_localAabbMin;
			aabbMax = m_localAabbMax;
		}

		protected void getNonvirtualAabb( ref btTransform trans, out btVector3 aabbMin, out btVector3 aabbMax, double margin )
		{

			//lazy evaluation of local aabb
			Debug.Assert( m_isLocalAabbValid );
			btAabbUtil.btTransformAabb( ref m_localAabbMin, ref m_localAabbMax, margin, ref trans, out aabbMin, out aabbMax );
		}


		public btConvexInternalAabbCachingShape()
		{
			//btConvexInternalShape()
			m_localAabbMin = new btVector3( 1, 1, 1 );
			m_localAabbMax = new btVector3( -1, -1, -1 );
			m_isLocalAabbValid = ( false );
		}


		public override void getAabb( ref btTransform trans, out btVector3 aabbMin, out btVector3 aabbMax )
		{
			getNonvirtualAabb( ref trans, out aabbMin, out aabbMax, getMargin() );
		}

		public override void setLocalScaling( ref btVector3 scaling )
		{
			setLocalScaling( ref scaling );
			recalcLocalAabb();
		}


#if NORMALLY_SET
		static btVector3[] _directions = new btVector3[]
		{
			new btVector3( 1.0,  0.0,   0.0),
			new btVector3( 0.0,  1.0,   0.0),
			new btVector3( 0.0,  0.0,  1.0),
			new btVector3( -1.0, 0.0,   0.0),
			new btVector3( 0.0, -1.0,   0.0),
			new btVector3( 0.0,  0.0, -1.0)
		};
#endif

		public void recalcLocalAabb()
		{
			m_isLocalAabbValid = true;

#if NORMALLY_SET

			btVector3[] _supporting = new btVector3[6];
			/*
			{
				btVector3( 0., 0., 0.0 ),
				btVector3( 0., 0., 0.0 ),
				btVector3( 0., 0., 0.0 ),
				btVector3( 0., 0., 0.0 ),
				btVector3( 0., 0., 0.0 ),
				btVector3( 0., 0., 0.0 )
            };
			*/
			batchedUnitVectorGetSupportingVertexWithoutMargin( _directions, _supporting, 6 );

			for( int i = 0; i < 3; ++i )
			{
				m_localAabbMax[i] = _supporting[i][i] + m_collisionMargin;
				m_localAabbMin[i] = _supporting[i + 3][i] - m_collisionMargin;
			}

#else

			for( int i = 0; i < 3; i++ )
			{
				btVector3 vec( btScalar.BT_ZERO, btScalar.BT_ZERO, btScalar.BT_ZERO);
			vec[i] = (double)( 1.0 );
			btVector3 tmp = localGetSupportingVertex( vec );
			m_localAabbMax[i] = tmp[i] + m_collisionMargin;
			vec[i] = (double)(-1.0);
			tmp = localGetSupportingVertex( vec );
			m_localAabbMin[i] = tmp[i] - m_collisionMargin;
		}
#endif
		}

	};

}
