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

using Bullet.LinearMath;
using Bullet.Types;
using System.Diagnostics;

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


	/// point to point constraint between two rigidbodies each with a pivotpoint that descibes the 'ballsocket' location in local space
	internal class btPoint2PointConstraint : btTypedConstraint
	{
		class btConstraintSetting
		{
			internal double m_tau = 0.3;
			internal double m_damping = btScalar.BT_ONE;
			internal double m_impulseClamp = btScalar.BT_ZERO;
		};

		internal enum btPoint2PointFlags
		{
			BT_P2P_FLAGS_ERP = 1,
			BT_P2P_FLAGS_CFM = 2
		};

		//btJacobianEntry[] m_jac = new btJacobianEntry[3]; //3 orthogonal linear constraints

		btVector3 m_pivotInA;
		btVector3 m_pivotInB;

		btPoint2PointFlags m_flags;
		double m_erp;
		double m_cfm;



		///for backwards compatibility during the transition to 'getInfo/getInfo2'
		bool m_useSolveConstraintObsolete;

		btConstraintSetting m_setting = new btConstraintSetting();
		/*
		btPoint2PointConstraint( btRigidBody rbA, btRigidBody rbB, ref btVector3 pivotInA, ref btVector3 pivotInB );

		btPoint2PointConstraint( btRigidBody rbA, ref btVector3 pivotInA );


		virtual void buildJacobian();

		virtual void getInfo1( btConstraintInfo1* info );

		void getInfo1NonVirtual( btConstraintInfo1* info );

		virtual void getInfo2( btConstraintInfo2* info );

		void getInfo2NonVirtual( btConstraintInfo2* info, ref btTransform body0_trans, ref btTransform body1_trans );

		void updateRHS( double timeStep );
		*/

		void setPivotA( ref btVector3 pivotA )
		{
			m_pivotInA = pivotA;
		}

		void setPivotB( ref btVector3 pivotB )
		{
			m_pivotInB = pivotB;
		}

		internal btIVector3 getPivotInA()
		{
			return m_pivotInA;
		}

		internal btIVector3 getPivotInB()
		{
			return m_pivotInB;
		}
		internal void getPivotInA( out btVector3 result )
		{
			result = m_pivotInA;
		}

		internal void getPivotInB( out btVector3 result )
		{
			result = m_pivotInB;
		}

		/*
		///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
		///If no axis is provided, it uses the default axis for this constraint.
		virtual void setParam( int num, double value, int axis = -1 );
		///return the local value of parameter
		virtual double getParam( int num, int axis = -1 );
		*/


		btPoint2PointConstraint( btRigidBody rbA, btRigidBody rbB, ref btVector3 pivotInA, ref btVector3 pivotInB )
				: base( btObjectTypes.POINT2POINT_CONSTRAINT_TYPE, rbA, rbB )
		{
			m_pivotInA = ( pivotInA );
			m_pivotInB = ( pivotInB );
			m_flags = ( 0 );
			m_useSolveConstraintObsolete = ( false );

		}


		btPoint2PointConstraint( btRigidBody rbA, ref btVector3 pivotInA )
			: base( btObjectTypes.POINT2POINT_CONSTRAINT_TYPE, rbA )
		{
			m_pivotInA = ( pivotInA );
			rbA.getCenterOfMassTransform().Apply( ref pivotInA, out m_pivotInB );
			m_flags = ( 0 );
			m_useSolveConstraintObsolete = ( false );

		}

		internal override void getInfo1( ref btConstraintInfo1 info )
		{
			getInfo1NonVirtual( ref info );
		}

		void getInfo1NonVirtual( ref btConstraintInfo1 info )
		{
			info.m_numConstraintRows = 3;
			info.nub = 3;
		}


		internal override void getInfo2( btConstraintInfo2 info )
		{
			getInfo2NonVirtual( info, ref m_rbA.m_worldTransform, ref m_rbB.m_worldTransform );
		}

		void getInfo2NonVirtual( btConstraintInfo2 info, ref btTransform body0_trans, ref btTransform body1_trans )
		{
			Debug.Assert( !m_useSolveConstraintObsolete );

			//retrieve matrices

			// anchor points in global coordinates with respect to body PORs.

			// set jacobian


			info.m_solverConstraints[0].m_contactNormal1 = btVector3.xAxis;
			info.m_solverConstraints[1].m_contactNormal1 = btVector3.yAxis;
			info.m_solverConstraints[2].m_contactNormal1 = btVector3.zAxis;


			btVector3 a1; body0_trans.m_basis.Apply( ref m_pivotInA, out a1 ) ;
			{
				//btVector3* angular0 = (btVector3*)( info.m_J1angularAxis );
				//btVector3* angular1 = (btVector3*)( info.m_J1angularAxis + info.rowskip );
				//btVector3* angular2 = (btVector3*)( info.m_J1angularAxis + 2 * info.rowskip );
				btVector3 a1neg;
				a1.Invert( out a1neg );
				a1neg.getSkewSymmetricMatrix( out info.m_solverConstraints[0].m_relpos1CrossNormal
					, out info.m_solverConstraints[1].m_relpos1CrossNormal
					, out info.m_solverConstraints[2].m_relpos1CrossNormal );
			}

			btVector3.xAxis.Invert( out info.m_solverConstraints[0].m_contactNormal2 );
			btVector3.yAxis.Invert( out info.m_solverConstraints[1].m_contactNormal2 );
			btVector3.zAxis.Invert( out info.m_solverConstraints[2].m_contactNormal2 );
			//info.m_J2linearAxis[0] = -1;
			//info.m_J2linearAxis[info.rowskip + 1] = -1;
			//info.m_J2linearAxis[2 * info.rowskip + 2] = -1;

			btVector3 a2; body1_trans.m_basis.Apply( ref m_pivotInB, out a2 );
				//getBasis() * getPivotInB();

			{
				//	btVector3 a2n = -a2;
				//btVector3* angular0 = (btVector3*)( info.m_J2angularAxis );
				//btVector3* angular1 = (btVector3*)( info.m_J2angularAxis + info.rowskip );
				//btVector3* angular2 = (btVector3*)( info.m_J2angularAxis + 2 * info.rowskip );
				a2.getSkewSymmetricMatrix( out info.m_solverConstraints[0].m_relpos2CrossNormal
					, out info.m_solverConstraints[1].m_relpos2CrossNormal
					, out info.m_solverConstraints[2].m_relpos2CrossNormal );
			}



			// set right hand side
			double currERP = ( m_flags & btPoint2PointFlags.BT_P2P_FLAGS_ERP )!= 0 ? m_erp : info.erp;
			double k = info.fps * currERP;
			int j;
			for( j = 0; j < 3; j++ )
			{
				info.m_solverConstraints[j].m_rhs = k * ( a2[j] + body1_trans.getOrigin()[j] - a1[j] - body0_trans.getOrigin()[j] );
				//Console.WriteLine("info.m_constraintError[%d]=%f\n",j,info.m_constraintError[j]);
			}
			if( ( m_flags & btPoint2PointFlags.BT_P2P_FLAGS_CFM ) != 0 )
			{
				for( j = 0; j < 3; j++ )
				{
					info.m_solverConstraints[j].m_cfm = m_cfm;
				}
			}

			double impulseClamp = m_setting.m_impulseClamp;//
			for( j = 0; j < 3; j++ )
			{
				if( m_setting.m_impulseClamp > 0 )
				{
					info.m_solverConstraints[j]. m_lowerLimit = -impulseClamp;
					info.m_solverConstraints[j].m_upperLimit = impulseClamp;
				}
			}
			info.m_damping = m_setting.m_damping;

		}



		void updateRHS( double timeStep )
		{
			//(void)timeStep;

		}

		///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
		///If no axis is provided, it uses the default axis for this constraint.
		internal override void setParam( btConstraintParams num, double value, int axis = -1 )
		{
			if( axis != -1 )
			{
				btAssertConstrParams( false );
			}
			else
			{
				switch( num )
				{
					case btConstraintParams.BT_CONSTRAINT_ERP:
					case btConstraintParams.BT_CONSTRAINT_STOP_ERP:
						m_erp = value;
						m_flags |= btPoint2PointFlags.BT_P2P_FLAGS_ERP;
						break;
					case btConstraintParams.BT_CONSTRAINT_CFM:
					case btConstraintParams.BT_CONSTRAINT_STOP_CFM:
						m_cfm = value;
						m_flags |= btPoint2PointFlags.BT_P2P_FLAGS_CFM;
						break;
					default:
						btAssertConstrParams( false );
						break;
				}
			}
		}

		///return the local value of parameter
		internal override double getParam( btConstraintParams num, int axis = -1 )
		{
			double retVal = ( btScalar.SIMD_INFINITY );
			if( axis != -1 )
			{
				btAssertConstrParams( false );
			}
			else
			{
				switch( num )
				{
					case btConstraintParams.BT_CONSTRAINT_ERP:
					case btConstraintParams.BT_CONSTRAINT_STOP_ERP:
						btAssertConstrParams( ( m_flags & btPoint2PointFlags.BT_P2P_FLAGS_ERP ) != 0 );
						retVal = m_erp;
						break;
					case btConstraintParams.BT_CONSTRAINT_CFM:
					case btConstraintParams.BT_CONSTRAINT_STOP_CFM:
						btAssertConstrParams( ( m_flags & btPoint2PointFlags.BT_P2P_FLAGS_CFM ) != 0 );
						retVal = m_cfm;
						break;
					default:
						btAssertConstrParams( false );
						break;
				}
			}
			return retVal;
		}




#if SERIALIZE_DONE
		virtual int calculateSerializeBufferSize();

		///fills the dataBuffer and returns the struct name (and 0 on failure)
		virtual string serialize( object dataBuffer, btSerializer* serializer );
#endif

	};
#if SERIALIZE_DONE

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

#if BT_BACKWARDS_COMPATIBLE_SERIALIZATION
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


	public int calculateSerializeBufferSize()
	{
		return sizeof( btPoint2PointConstraintData2 );

	}

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	public string serialize( object dataBuffer, btSerializer* serializer )
	{
		btPoint2PointConstraintData2* p2pData = (btPoint2PointConstraintData2*)dataBuffer;

		btTypedConstraint::serialize( &p2pData.m_typeConstraintData, serializer );
		m_pivotInA.serialize( p2pData.m_pivotInA );
		m_pivotInB.serialize( p2pData.m_pivotInB );

		return btPoint2PointConstraintDataName;
	}
#endif

}
