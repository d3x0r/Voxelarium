/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

namespace Bullet.Dynamics.ConstraintSolver
{
	/*
# ifdef BT_USE_DOUBLE_PRECISION
#define btPoint2PointConstraintData2	btPoint2PointConstraintDoubleData2
#define btPoint2PointConstraintDataName	"btPoint2PointConstraintDoubleData2"
#else
#define btPoint2PointConstraintData2	btPoint2PointConstraintFloatData
#define btPoint2PointConstraintDataName	"btPoint2PointConstraintFloatData"
#endif //BT_USE_DOUBLE_PRECISION
*/

	struct btConstraintSetting
	{
		btConstraintSetting() :
        m_tau((double)(0.3)),
        m_damping(btScalar.BT_ONE),
        m_impulseClamp(btScalar.BT_ZERO)
	{
	}
	double m_tau;
		double m_damping;
		double m_impulseClamp;
	};

	enum btPoint2PointFlags
	{
		BT_P2P_FLAGS_ERP = 1,
		BT_P2P_FLAGS_CFM = 2
	};

	/// point to point constraint between two rigidbodies each with a pivotpoint that descibes the 'ballsocket' location in local space
	internal class btPoint2PointConstraint : btTypedConstraint
	{
# ifdef IN_PARALLELL_SOLVER
		public:
#endif
	btJacobianEntry m_jac[3]; //3 orthogonal linear constraints

		btVector3 m_pivotInA;
		btVector3 m_pivotInB;

		int m_flags;
		double m_erp;
		double m_cfm;

		public:

	

	///for backwards compatibility during the transition to 'getInfo/getInfo2'
	bool m_useSolveConstraintObsolete;

		btConstraintSetting m_setting;

		btPoint2PointConstraint( btRigidBody rbA, btRigidBody rbB, ref btVector3 pivotInA, ref btVector3 pivotInB );

		btPoint2PointConstraint( btRigidBody rbA, ref btVector3 pivotInA );


		virtual void buildJacobian();

		virtual void getInfo1( btConstraintInfo1* info );

		void getInfo1NonVirtual( btConstraintInfo1* info );

		virtual void getInfo2( btConstraintInfo2* info );

		void getInfo2NonVirtual( btConstraintInfo2* info, ref btTransform body0_trans, ref btTransform body1_trans );

		void updateRHS( double timeStep );

		void setPivotA( ref btVector3 pivotA )
		{
			m_pivotInA = pivotA;
		}

		void setPivotB( ref btVector3 pivotB )
		{
			m_pivotInB = pivotB;
		}

	ref btVector3 getPivotInA()
		{
			return m_pivotInA;
		}

	ref btVector3 getPivotInB()
		{
			return m_pivotInB;
		}

		///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
		///If no axis is provided, it uses the default axis for this constraint.
		virtual void setParam( int num, double value, int axis = -1 );
		///return the local value of parameter
		virtual double getParam( int num, int axis = -1 );

		virtual int calculateSerializeBufferSize();

		///fills the dataBuffer and returns the struct name (and 0 on failure)
		virtual string serialize( object dataBuffer, btSerializer* serializer );


	};

	///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
	struct btPoint2PointConstraintFloatData
	{
		btTypedConstraintData m_typeConstraintData;
		btVector3FloatData m_pivotInA;
		btVector3FloatData m_pivotInB;
	};

	///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
	struct btPoint2PointConstraintDoubleData2
	{
		btTypedConstraintDoubleData m_typeConstraintData;
		btVector3DoubleData m_pivotInA;
		btVector3DoubleData m_pivotInB;
	};

# ifdef BT_BACKWARDS_COMPATIBLE_SERIALIZATION
	///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
	///this structure is not used, except for loading pre-2.82 .bullet files
	///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
	struct btPoint2PointConstraintDoubleData
	{
		btTypedConstraintData m_typeConstraintData;
		btVector3DoubleData m_pivotInA;
		btVector3DoubleData m_pivotInB;
	};
#endif //BT_BACKWARDS_COMPATIBLE_SERIALIZATION


	public int btPoint2PointConstraint::calculateSerializeBufferSize()
	{
		return sizeof( btPoint2PointConstraintData2 );

	}

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	public string btPoint2PointConstraint::serialize( object dataBuffer, btSerializer* serializer )
	{
		btPoint2PointConstraintData2* p2pData = (btPoint2PointConstraintData2*)dataBuffer;

		btTypedConstraint::serialize( &p2pData.m_typeConstraintData, serializer );
		m_pivotInA.serialize( p2pData.m_pivotInA );
		m_pivotInB.serialize( p2pData.m_pivotInB );

		return btPoint2PointConstraintDataName;
	}

}
