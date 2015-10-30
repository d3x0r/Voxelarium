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


	///The btStaticPlaneShape simulates an infinite non-moving (static) collision plane.
	internal class btStaticPlaneShape : btConcaveShape
	{
		protected btVector3 m_localAabbMin;
		protected btVector3 m_localAabbMax;

		protected btVector3 m_planeNormal;
		protected double m_planeConstant;
		protected btVector3 m_localScaling;


		/*
			virtual void getAabb(ref btTransform t,ref btVector3 aabbMin,ref btVector3 aabbMax);

			virtual void	processAllTriangles(btTriangleCallback* callback,ref btVector3 aabbMin,ref btVector3 aabbMax);

			virtual void	calculateLocalInertia(double mass,ref btVector3 inertia);

			virtual void	setLocalScaling(ref btVector3 scaling);
			virtual ref btVector3 getLocalScaling();
		*/

		internal void getPlaneNormal( out btVector3 result )
		{
			result = m_planeNormal;
		}

		internal double getPlaneConstant()
		{
			return m_planeConstant;
		}

		//debugging
		public override string ToString() { return "STATICPLANE"; }
		btStaticPlaneShape( ref btVector3 planeNormal, double planeConstant ) : base()
		{
			planeNormal.normalized( out m_planeNormal );
			m_planeConstant = ( planeConstant );
			m_localScaling = btVector3.Zero;
            m_shapeType = BroadphaseNativeTypes.STATIC_PLANE_PROXYTYPE;
			//	Debug.Assert( btFuzzyZero(m_planeNormal.length() - btScalar.BT_ONE) );
		}

		public override void getAabb( ref btTransform t, out btVector3 aabbMin, out btVector3 aabbMax )
		{
			//(void)t;
			/*
			btVector3 infvec ((double)(BT_LARGE_FLOAT),(double)(BT_LARGE_FLOAT),(double)(BT_LARGE_FLOAT));

			btVector3 center = m_planeNormal*m_planeConstant;
			aabbMin = center + infvec*m_planeNormal;
			aabbMax = aabbMin;
			aabbMin.setMin(center - infvec*m_planeNormal);
			aabbMax.setMax(center - infvec*m_planeNormal); 
			*/

			aabbMin = btVector3.Min;
			aabbMax = btVector3.Max;

		}


		internal override void processAllTriangles( btTriangleCallback callback, ref btVector3 aabbMin, ref btVector3 aabbMax )
		{

			btVector3 halfExtents = ( aabbMax - aabbMin ) * (double)( 0.5 );
			double radius = halfExtents.length();
			btVector3 center = ( aabbMax + aabbMin ) * (double)( 0.5 );

			//this is where the triangles are generated, given AABB and plane equation (normal/constant)

			btVector3 tangentDir0, tangentDir1;

			//tangentDir0/tangentDir1 can be precalculated
			btVector3.btPlaneSpace1( ref m_planeNormal, out tangentDir0, out tangentDir1 );

			//btVector3 supVertex0, supVertex1;

			btVector3 projectedCenter;// = center - ( m_planeNormal.dot( center ) - m_planeConstant ) * m_planeNormal;
			center.SubScale( ref m_planeNormal, ( m_planeNormal.dot( center ) - m_planeConstant ), out projectedCenter );

			btVector3[] triangle = new btVector3[3];
			triangle[0] = projectedCenter + tangentDir0 * radius + tangentDir1 * radius;
			triangle[1] = projectedCenter + tangentDir0 * radius - tangentDir1 * radius;
			triangle[2] = projectedCenter - tangentDir0 * radius - tangentDir1 * radius;

			callback.processTriangle( triangle, 0, 0 );

			triangle[0] = projectedCenter - tangentDir0 * radius - tangentDir1 * radius;
			triangle[1] = projectedCenter - tangentDir0 * radius + tangentDir1 * radius;
			triangle[2] = projectedCenter + tangentDir0 * radius + tangentDir1 * radius;

			callback.processTriangle( triangle, 0, 1 );

		}

		public override void calculateLocalInertia( double mass, out btVector3 inertia )
		{
			//(void)mass;

			//moving concave objects not supported

			inertia = btVector3.Zero;
		}

		public override void setLocalScaling( ref btVector3 scaling )
		{
			m_localScaling = scaling;
		}
		public override void getLocalScaling( out btVector3 result )
		{
			result = m_localScaling;
		}

#if SERIALIZE_DONE
		virtual int	calculateSerializeBufferSize();

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	string	serialize(object dataBuffer, btSerializer* serializer);

#endif
	};

#if SERIALIZE_DONE
	///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
	struct	btStaticPlaneShapeData
{
	btCollisionShapeData	m_collisionShapeData;

	btVector3FloatData	m_localScaling;
	btVector3FloatData	m_planeNormal;
	float			m_planeConstant;
	char	m_pad[4];
};


public	int	calculateSerializeBufferSize()
{
	return sizeof(btStaticPlaneShapeData);
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
public	string	serialize(object dataBuffer, btSerializer* serializer)
{
	btStaticPlaneShapeData* planeData = (btStaticPlaneShapeData*) dataBuffer;
	btCollisionShape::serialize(&planeData.m_collisionShapeData,serializer);

	m_localScaling.serializeFloat(planeData.m_localScaling);
	m_planeNormal.serializeFloat(planeData.m_planeNormal);
	planeData.m_planeConstant = float(m_planeConstant);
		
	return "btStaticPlaneShapeData";
}
#endif

}
