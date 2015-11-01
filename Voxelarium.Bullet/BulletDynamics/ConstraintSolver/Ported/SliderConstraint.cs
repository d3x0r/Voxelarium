#define USE_OFFSET_FOR_CONSTANT_FRAME
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

/*
Added by Roman Ponomarev (rponom@gmail.com)
April 04, 2008

TODO:
 - add clamping od accumulated impulse to improve stability
 - add conversion for ODE constraint solver
*/

using Bullet.LinearMath;
using Bullet.Types;
using System;
using System.Diagnostics;

namespace Bullet.Dynamics.ConstraintSolver
{
	/*
#if BT_USE_DOUBLE_PRECISION
#define btSliderConstraintData2		btSliderConstraintDoubleData
#define btSliderConstraintDataName  "btSliderConstraintDoubleData"
#else
#define btSliderConstraintData2		btSliderConstraintData 
#define btSliderConstraintDataName	"btSliderConstraintData"
#endif //BT_USE_DOUBLE_PRECISION


*/



	[Flags]
	internal enum btSliderFlags
	{
		BT_SLIDER_FLAGS_CFM_DIRLIN = ( 1 << 0 ),
		BT_SLIDER_FLAGS_ERP_DIRLIN = ( 1 << 1 ),
		BT_SLIDER_FLAGS_CFM_DIRANG = ( 1 << 2 ),
		BT_SLIDER_FLAGS_ERP_DIRANG = ( 1 << 3 ),
		BT_SLIDER_FLAGS_CFM_ORTLIN = ( 1 << 4 ),
		BT_SLIDER_FLAGS_ERP_ORTLIN = ( 1 << 5 ),
		BT_SLIDER_FLAGS_CFM_ORTANG = ( 1 << 6 ),
		BT_SLIDER_FLAGS_ERP_ORTANG = ( 1 << 7 ),
		BT_SLIDER_FLAGS_CFM_LIMLIN = ( 1 << 8 ),
		BT_SLIDER_FLAGS_ERP_LIMLIN = ( 1 << 9 ),
		BT_SLIDER_FLAGS_CFM_LIMANG = ( 1 << 10 ),
		BT_SLIDER_FLAGS_ERP_LIMANG = ( 1 << 11 )
	};


	internal class btSliderConstraint : btTypedConstraint
	{
		const double SLIDER_CONSTRAINT_DEF_SOFTNESS = ( (double)( 1.0 ) );
		const double SLIDER_CONSTRAINT_DEF_DAMPING = ( (double)( 1.0 ) );
		const double SLIDER_CONSTRAINT_DEF_RESTITUTION = ( (double)( 0.7 ) );
		const double SLIDER_CONSTRAINT_DEF_CFM = ( (double)( 0 ) );

		///for backwards compatibility during the transition to 'getInfo/getInfo2'
		protected bool m_useSolveConstraintObsolete;
		protected bool m_useOffsetForConstraintFrame;
		protected btTransform m_frameInA;
		protected btTransform m_frameInB;
		// use frameA fo define limits, if true
		protected bool m_useLinearReferenceFrameA;
		// linear limits
		protected double m_lowerLinLimit;
		protected double m_upperLinLimit;
		// angular limits
		protected double m_lowerAngLimit;
		protected double m_upperAngLimit;
		// softness, restitution and damping for different cases
		// DirLin - moving inside linear limits
		// LimLin - hitting linear limit
		// DirAng - moving inside angular limits
		// LimAng - hitting angular limit
		// OrthoLin, OrthoAng - against constraint axis
		protected double m_softnessDirLin;
		protected double m_restitutionDirLin;
		protected double m_dampingDirLin;
		protected double m_cfmDirLin;

		protected double m_softnessDirAng;
		protected double m_restitutionDirAng;
		protected double m_dampingDirAng;
		protected double m_cfmDirAng;

		protected double m_softnessLimLin;
		protected double m_restitutionLimLin;
		protected double m_dampingLimLin;
		protected double m_cfmLimLin;

		protected double m_softnessLimAng;
		protected double m_restitutionLimAng;
		protected double m_dampingLimAng;
		protected double m_cfmLimAng;

		protected double m_softnessOrthoLin;
		protected double m_restitutionOrthoLin;
		protected double m_dampingOrthoLin;
		protected double m_cfmOrthoLin;

		protected double m_softnessOrthoAng;
		protected double m_restitutionOrthoAng;
		protected double m_dampingOrthoAng;
		protected double m_cfmOrthoAng;

		// for interlal use
		protected bool m_solveLinLim;
		protected bool m_solveAngLim;

		protected btSliderFlags m_flags;

		protected btJacobianEntry[] m_jacLin = new btJacobianEntry[3];
		protected double[] m_jacLinDiagABInv = new double[3];

		protected btJacobianEntry[] m_jacAng = new btJacobianEntry[3];

		protected double m_timeStep;
		protected btTransform m_calculatedTransformA;
		protected btTransform m_calculatedTransformB;

		protected btVector3 m_sliderAxis;
		protected btVector3 m_realPivotAInW;
		protected btVector3 m_realPivotBInW;
		protected btVector3 m_projPivotInW;
		protected btVector3 m_delta;
		protected btVector3 m_depth;
		protected btVector3 m_relPosA;
		protected btVector3 m_relPosB;

		protected double m_linPos;
		protected double m_angPos;

		protected double m_angDepth;
		protected double m_kAngle;

		protected bool m_poweredLinMotor;
		protected double m_targetLinMotorVelocity;
		protected double m_maxLinMotorForce;
		protected double m_accumulatedLinMotorImpulse;

		protected bool m_poweredAngMotor;
		protected double m_targetAngMotorVelocity;
		protected double m_maxAngMotorForce;
		protected double m_accumulatedAngMotorImpulse;

		//------------------------    
		///protected void initParams();


#if asdfasdf
			// constructors
			btSliderConstraint( btRigidBody rbA, btRigidBody rbB, ref btTransform frameInA, ref btTransform frameInB, bool useLinearReferenceFrameA );
		btSliderConstraint( btRigidBody rbB, ref btTransform frameInB, bool useLinearReferenceFrameA );

		// overrides

		virtual void getInfo1( btConstraintInfo1* info );

		void getInfo1NonVirtual( btConstraintInfo1* info );

		virtual void getInfo2( btConstraintInfo2* info );

		void getInfo2NonVirtual( btConstraintInfo2* info, ref btTransform transA, ref btTransform transB, ref btVector3 linVelA, ref btVector3 linVelB, double rbAinvMass, double rbBinvMass );
#endif

		// access
		public btITransform getCalculatedTransformA() { return m_calculatedTransformA; }
		public btITransform getCalculatedTransformB() { return m_calculatedTransformB; }
		public btITransform getFrameOffsetA() { return m_frameInA; }
		public btITransform getFrameOffsetB() { return m_frameInB; }
		public double getLowerLinLimit() { return m_lowerLinLimit; }
		public void setLowerLinLimit( double lowerLimit ) { m_lowerLinLimit = lowerLimit; }
		public double getUpperLinLimit() { return m_upperLinLimit; }
		public void setUpperLinLimit( double upperLimit ) { m_upperLinLimit = upperLimit; }
		public double getLowerAngLimit() { return m_lowerAngLimit; }
		public void setLowerAngLimit( double lowerLimit ) { m_lowerAngLimit = btScalar.btNormalizeAngle( lowerLimit ); }
		public double getUpperAngLimit() { return m_upperAngLimit; }
		public void setUpperAngLimit( double upperLimit ) { m_upperAngLimit = btScalar.btNormalizeAngle( upperLimit ); }
		public bool getUseLinearReferenceFrameA() { return m_useLinearReferenceFrameA; }
		public double getSoftnessDirLin() { return m_softnessDirLin; }
		public double getRestitutionDirLin() { return m_restitutionDirLin; }
		public double getDampingDirLin() { return m_dampingDirLin; }
		public double getSoftnessDirAng() { return m_softnessDirAng; }
		public double getRestitutionDirAng() { return m_restitutionDirAng; }
		public double getDampingDirAng() { return m_dampingDirAng; }
		public double getSoftnessLimLin() { return m_softnessLimLin; }
		public double getRestitutionLimLin() { return m_restitutionLimLin; }
		public double getDampingLimLin() { return m_dampingLimLin; }
		public double getSoftnessLimAng() { return m_softnessLimAng; }
		public double getRestitutionLimAng() { return m_restitutionLimAng; }
		public double getDampingLimAng() { return m_dampingLimAng; }
		public double getSoftnessOrthoLin() { return m_softnessOrthoLin; }
		public double getRestitutionOrthoLin() { return m_restitutionOrthoLin; }
		public double getDampingOrthoLin() { return m_dampingOrthoLin; }
		public double getSoftnessOrthoAng() { return m_softnessOrthoAng; }
		public double getRestitutionOrthoAng() { return m_restitutionOrthoAng; }
		public double getDampingOrthoAng() { return m_dampingOrthoAng; }
		public void setSoftnessDirLin( double softnessDirLin ) { m_softnessDirLin = softnessDirLin; }
		public void setRestitutionDirLin( double restitutionDirLin ) { m_restitutionDirLin = restitutionDirLin; }
		public void setDampingDirLin( double dampingDirLin ) { m_dampingDirLin = dampingDirLin; }
		public void setSoftnessDirAng( double softnessDirAng ) { m_softnessDirAng = softnessDirAng; }
		public void setRestitutionDirAng( double restitutionDirAng ) { m_restitutionDirAng = restitutionDirAng; }
		public void setDampingDirAng( double dampingDirAng ) { m_dampingDirAng = dampingDirAng; }
		public void setSoftnessLimLin( double softnessLimLin ) { m_softnessLimLin = softnessLimLin; }
		public void setRestitutionLimLin( double restitutionLimLin ) { m_restitutionLimLin = restitutionLimLin; }
		public void setDampingLimLin( double dampingLimLin ) { m_dampingLimLin = dampingLimLin; }
		public void setSoftnessLimAng( double softnessLimAng ) { m_softnessLimAng = softnessLimAng; }
		public void setRestitutionLimAng( double restitutionLimAng ) { m_restitutionLimAng = restitutionLimAng; }
		public void setDampingLimAng( double dampingLimAng ) { m_dampingLimAng = dampingLimAng; }
		public void setSoftnessOrthoLin( double softnessOrthoLin ) { m_softnessOrthoLin = softnessOrthoLin; }
		public void setRestitutionOrthoLin( double restitutionOrthoLin ) { m_restitutionOrthoLin = restitutionOrthoLin; }
		public void setDampingOrthoLin( double dampingOrthoLin ) { m_dampingOrthoLin = dampingOrthoLin; }
		public void setSoftnessOrthoAng( double softnessOrthoAng ) { m_softnessOrthoAng = softnessOrthoAng; }
		public void setRestitutionOrthoAng( double restitutionOrthoAng ) { m_restitutionOrthoAng = restitutionOrthoAng; }
		public void setDampingOrthoAng( double dampingOrthoAng ) { m_dampingOrthoAng = dampingOrthoAng; }
		public void setPoweredLinMotor( bool onOff ) { m_poweredLinMotor = onOff; }
		public bool getPoweredLinMotor() { return m_poweredLinMotor; }
		public void setTargetLinMotorVelocity( double targetLinMotorVelocity ) { m_targetLinMotorVelocity = targetLinMotorVelocity; }
		public double getTargetLinMotorVelocity() { return m_targetLinMotorVelocity; }
		public void setMaxLinMotorForce( double maxLinMotorForce ) { m_maxLinMotorForce = maxLinMotorForce; }
		public double getMaxLinMotorForce() { return m_maxLinMotorForce; }
		public void setPoweredAngMotor( bool onOff ) { m_poweredAngMotor = onOff; }
		public bool getPoweredAngMotor() { return m_poweredAngMotor; }
		public void setTargetAngMotorVelocity( double targetAngMotorVelocity ) { m_targetAngMotorVelocity = targetAngMotorVelocity; }
		public double getTargetAngMotorVelocity() { return m_targetAngMotorVelocity; }
		public void setMaxAngMotorForce( double maxAngMotorForce ) { m_maxAngMotorForce = maxAngMotorForce; }
		public double getMaxAngMotorForce() { return m_maxAngMotorForce; }

		public double getLinearPos() { return m_linPos; }
		public double getAngularPos() { return m_angPos; }



		// access for ODE solver
		public bool getSolveLinLimit() { return m_solveLinLim; }
		public double getLinDepth() { return m_depth.x; }
		public bool getSolveAngLimit() { return m_solveAngLim; }
		public double getAngDepth() { return m_angDepth; }
		// shared code used by ODE solver
		//void calculateTransforms( ref btTransform transA, ref btTransform transB );
		//void testLinLimits();
		//void testAngLimits();
		// access for PE Solver
		//btIVector3 getAncorInA();
		//btIVector3 getAncorInB();
		// access for UseFrameOffset
		public bool getUseFrameOffset() { return m_useOffsetForConstraintFrame; }
		public void setUseFrameOffset( bool frameOffsetOnOff ) { m_useOffsetForConstraintFrame = frameOffsetOnOff; }

		public void setFrames( ref btTransform frameA, ref btTransform frameB )
		{
			m_frameInA = frameA;
			m_frameInB = frameB;
			calculateTransforms( ref m_rbA.m_worldTransform, ref m_rbB.m_worldTransform );
			buildJacobian();
		}




		internal void initParams()
		{
			m_lowerLinLimit = (double)( 1.0 );
			m_upperLinLimit = (double)( -1.0 );
			m_lowerAngLimit = btScalar.BT_ZERO;
			m_upperAngLimit = btScalar.BT_ZERO;
			m_softnessDirLin = SLIDER_CONSTRAINT_DEF_SOFTNESS;
			m_restitutionDirLin = SLIDER_CONSTRAINT_DEF_RESTITUTION;
			m_dampingDirLin = btScalar.BT_ZERO;
			m_cfmDirLin = SLIDER_CONSTRAINT_DEF_CFM;
			m_softnessDirAng = SLIDER_CONSTRAINT_DEF_SOFTNESS;
			m_restitutionDirAng = SLIDER_CONSTRAINT_DEF_RESTITUTION;
			m_dampingDirAng = btScalar.BT_ZERO;
			m_cfmDirAng = SLIDER_CONSTRAINT_DEF_CFM;
			m_softnessOrthoLin = SLIDER_CONSTRAINT_DEF_SOFTNESS;
			m_restitutionOrthoLin = SLIDER_CONSTRAINT_DEF_RESTITUTION;
			m_dampingOrthoLin = SLIDER_CONSTRAINT_DEF_DAMPING;
			m_cfmOrthoLin = SLIDER_CONSTRAINT_DEF_CFM;
			m_softnessOrthoAng = SLIDER_CONSTRAINT_DEF_SOFTNESS;
			m_restitutionOrthoAng = SLIDER_CONSTRAINT_DEF_RESTITUTION;
			m_dampingOrthoAng = SLIDER_CONSTRAINT_DEF_DAMPING;
			m_cfmOrthoAng = SLIDER_CONSTRAINT_DEF_CFM;
			m_softnessLimLin = SLIDER_CONSTRAINT_DEF_SOFTNESS;
			m_restitutionLimLin = SLIDER_CONSTRAINT_DEF_RESTITUTION;
			m_dampingLimLin = SLIDER_CONSTRAINT_DEF_DAMPING;
			m_cfmLimLin = SLIDER_CONSTRAINT_DEF_CFM;
			m_softnessLimAng = SLIDER_CONSTRAINT_DEF_SOFTNESS;
			m_restitutionLimAng = SLIDER_CONSTRAINT_DEF_RESTITUTION;
			m_dampingLimAng = SLIDER_CONSTRAINT_DEF_DAMPING;
			m_cfmLimAng = SLIDER_CONSTRAINT_DEF_CFM;

			m_poweredLinMotor = false;
			m_targetLinMotorVelocity = btScalar.BT_ZERO;
			m_maxLinMotorForce = btScalar.BT_ZERO;
			m_accumulatedLinMotorImpulse = (double)( 0.0 );

			m_poweredAngMotor = false;
			m_targetAngMotorVelocity = btScalar.BT_ZERO;
			m_maxAngMotorForce = btScalar.BT_ZERO;
			m_accumulatedAngMotorImpulse = (double)( 0.0 );

			m_flags = 0;
			m_flags = 0;

			m_useOffsetForConstraintFrame = true;// USE_OFFSET_FOR_CONSTANT_FRAME;

			calculateTransforms( ref m_rbA.m_worldTransform, ref m_rbB.m_worldTransform );
		}





		internal btSliderConstraint( btRigidBody rbA, btRigidBody rbB, ref btTransform frameInA, ref btTransform frameInB, bool useLinearReferenceFrameA )
			 : base( btObjectTypes.SLIDER_CONSTRAINT_TYPE, rbA, rbB )
		{

			//m_useSolveConstraintObsolete = ( false ),
			m_frameInA = ( frameInA );
			m_frameInB = ( frameInB );
			m_useLinearReferenceFrameA = ( useLinearReferenceFrameA );
            initParams();
		}



		internal btSliderConstraint( btRigidBody rbB, ref btTransform frameInB, bool useLinearReferenceFrameA )
			: base( btObjectTypes.SLIDER_CONSTRAINT_TYPE, getFixedBody(), rbB )
		{
			//m_useSolveConstraintObsolete = (false),
			m_frameInB = ( frameInB );
			m_useLinearReferenceFrameA = ( useLinearReferenceFrameA );
				///not providing rigidbody A means implicitly using worldspace for body A
			rbB.m_worldTransform.Apply( ref m_frameInB, out m_frameInA );
			//	m_frameInA.m_origin = m_rbA.m_worldTransform(m_frameInA.m_origin);

			initParams();
		}






		internal override void getInfo1( ref btConstraintInfo1 info )
		{
			if( m_useSolveConstraintObsolete )
			{
				info.m_numConstraintRows = 0;
				info.nub = 0;
			}
			else
			{
				info.m_numConstraintRows = 4; // Fixed 2 linear + 2 angular
				info.nub = 2;
				//prepare constraint
				calculateTransforms( ref m_rbA.m_worldTransform, ref m_rbB.m_worldTransform );
				testAngLimits();
				testLinLimits();
				if( getSolveLinLimit() || getPoweredLinMotor() )
				{
					info.m_numConstraintRows++; // limit 3rd linear as well
					info.nub--;
				}
				if( getSolveAngLimit() || getPoweredAngMotor() )
				{
					info.m_numConstraintRows++; // limit 3rd angular as well
					info.nub--;
				}
			}
		}

		internal void getInfo1NonVirtual( ref btConstraintInfo1 info )
		{

			info.m_numConstraintRows = 6; // Fixed 2 linear + 2 angular + 1 limit (even if not used)
			info.nub = 0;
		}

		internal override void getInfo2(  btConstraintInfo2 info )
		{
			getInfo2NonVirtual( info, ref m_rbA.m_worldTransform, ref m_rbB.m_worldTransform
				, ref m_rbA.m_linearVelocity, ref m_rbB.m_linearVelocity
				, m_rbA.getInvMass(), m_rbB.getInvMass() );
		}







		internal void calculateTransforms( ref btTransform transA, ref btTransform transB )
		{
			if( m_useLinearReferenceFrameA || ( !m_useSolveConstraintObsolete ) )
			{
				transA.Apply( ref m_frameInA, out m_calculatedTransformA );
				transB.Apply( ref m_frameInB, out m_calculatedTransformB );
			}
			else
			{
				transA.Apply( ref m_frameInA, out m_calculatedTransformB );
				transB.Apply( ref m_frameInB, out m_calculatedTransformA );
			}
			m_realPivotAInW = m_calculatedTransformA.m_origin;
			m_realPivotBInW = m_calculatedTransformB.m_origin;
			m_sliderAxis = m_calculatedTransformA.m_basis.getColumn( 0 ); // along X
			if( m_useLinearReferenceFrameA || m_useSolveConstraintObsolete )
			{
				m_delta = m_realPivotBInW - m_realPivotAInW;
			}
			else
			{
				m_delta = m_realPivotAInW - m_realPivotBInW;
			}
			m_realPivotAInW.AddScale( m_sliderAxis, m_sliderAxis.dot( ref m_delta ), out m_projPivotInW );
            //m_projPivotInW = m_realPivotAInW + m_sliderAxis.dot( ref m_delta ) * m_sliderAxis;
			btVector3 normalWorld;
			int i;
			//linear part
			for( i = 0; i < 3; i++ )
			{
				normalWorld = m_calculatedTransformA.m_basis.getColumn( i );
				m_depth[i] = m_delta.dot( normalWorld );
			}
		}



		internal void testLinLimits()
		{
			m_solveLinLim = false;
			m_linPos = m_depth[0];
			if( m_lowerLinLimit <= m_upperLinLimit )
			{
				if( m_depth[0] > m_upperLinLimit )
				{
					m_depth[0] -= m_upperLinLimit;
					m_solveLinLim = true;
				}
				else if( m_depth[0] < m_lowerLinLimit )
				{
					m_depth[0] -= m_lowerLinLimit;
					m_solveLinLim = true;
				}
				else
				{
					m_depth[0] = btScalar.BT_ZERO;
				}
			}
			else
			{
				m_depth[0] = btScalar.BT_ZERO;
			}
		}



		internal void testAngLimits()
		{
			m_angDepth = btScalar.BT_ZERO;
			m_solveAngLim = false;
			if( m_lowerAngLimit <= m_upperAngLimit )
			{
				btVector3 axisA0 = m_calculatedTransformA.m_basis.getColumn( 1 );
				btVector3 axisA1 = m_calculatedTransformA.m_basis.getColumn( 2 );
				btVector3 axisB0 = m_calculatedTransformB.m_basis.getColumn( 1 );
				//		double rot = btAtan2Fast(axisB0.dot(axisA1), axisB0.dot(axisA0));  
				double rot = btScalar.btAtan2( axisB0.dot( axisA1 ), axisB0.dot( axisA0 ) );
				rot = btAdjustAngleToLimits( rot, m_lowerAngLimit, m_upperAngLimit );
				m_angPos = rot;
				if( rot < m_lowerAngLimit )
				{
					m_angDepth = rot - m_lowerAngLimit;
					m_solveAngLim = true;
				}
				else if( rot > m_upperAngLimit )
				{
					m_angDepth = rot - m_upperAngLimit;
					m_solveAngLim = true;
				}
			}
		}

		internal btVector3 getAncorInA()
		{
			btVector3 ancorInA;
			m_realPivotAInW.AddScale( ref m_sliderAxis, ( m_lowerLinLimit + m_upperLinLimit ) * (double)( 0.5 ), out ancorInA );
			//ancorInA = m_realPivotAInW + ( m_lowerLinLimit + m_upperLinLimit ) * (double)( 0.5 )
			//		* m_sliderAxis;
			btTransform inv;
			m_rbA.m_worldTransform.inverse( out inv );
			btVector3 tmp;
			inv.Apply( ref ancorInA, out tmp );
			//ancorInA = m_rbA.m_worldTransform.inverse() * ancorInA;
			ancorInA = tmp;
			return ancorInA;
		}



		internal btVector3 getAncorInB()
		{
			btVector3 ancorInB;
			ancorInB = m_frameInB.m_origin;
			return ancorInB;
		}


		internal void getInfo2NonVirtual( btConstraintInfo2 info, ref btTransform transA, ref btTransform transB, ref btVector3 linVelA, ref btVector3 linVelB, double rbAinvMass, double rbBinvMass )
		{
			//btITransform m_calculatedTransformB = getCalculatedTransformB();

			Debug.Assert( !m_useSolveConstraintObsolete );
			//int i;//, s = info.rowskip;

			double signFact = m_useLinearReferenceFrameA ? (double)( 1.0f ) : (double)( -1.0f );

			// difference between frames in WCS
			btVector3 ofs; m_calculatedTransformB.m_origin.Sub( m_calculatedTransformA.m_origin, out ofs );
			// now get weight factors depending on masses
			double miA = rbAinvMass;
			double miB = rbBinvMass;
			bool hasStaticBody = ( miA < btScalar.SIMD_EPSILON ) || ( miB < btScalar.SIMD_EPSILON );
			double miS = miA + miB;
			double factA, factB;
			if( miS > (double)( 0 ) )
			{
				factA = miB / miS;
			}
			else
			{
				factA = (double)( 0.5f );
			}
			factB = (double)( 1.0f ) - factA;
			btVector3 ax1, p, q;
			btVector3 ax1A = m_calculatedTransformA.m_basis.getColumn( 0 );
			btVector3 ax1B = m_calculatedTransformB.m_basis.getColumn( 0 );
			if( m_useOffsetForConstraintFrame )
			{
				// get the desired direction of slider axis
				// as weighted sum of X-orthos of frameA and frameB in WCS
				ax1 = ax1A * factA + ax1B * factB;
				ax1.normalize();
				// construct two orthos to slider axis
				btVector3.btPlaneSpace1( ref ax1, out p, out q );
			}
			else
			{ // old way - use frameA
				ax1 = m_calculatedTransformA.m_basis.getColumn( 0 );
				// get 2 orthos to slider axis (Y, Z)
				p = m_calculatedTransformA.m_basis.getColumn( 1 );
				q = m_calculatedTransformA.m_basis.getColumn( 2 );
			}
			// make rotations around these orthos equal
			// the slider axis should be the only unconstrained
			// rotational axis, the angular velocity of the two bodies perpendicular to
			// the slider axis should be equal. thus the constraint equations are
			//    p*w1 - p*w2 = 0
			//    q*w1 - q*w2 = 0
			// where p and q are unit vectors normal to the slider axis, and w1 and w2
			// are the angular velocity vectors of the two bodies.
			info.m_solverConstraints[0].m_relpos1CrossNormal = p;// m_J1angularAxis[0] = p[0];
			info.m_solverConstraints[1].m_relpos1CrossNormal = q;// m_J1angularAxis[0] = p[0];

			p.Invert( out info.m_solverConstraints[0].m_relpos2CrossNormal );
			q.Invert( out info.m_solverConstraints[1].m_relpos2CrossNormal );
			// compute the right hand side of the constraint equation. set relative
			// body velocities along p and q to bring the slider back into alignment.
			// if ax1A,ax1B are the unit length slider axes as computed from bodyA and
			// bodyB, we need to rotate both bodies along the axis u = (ax1 x ax2).
			// if "theta" is the angle between ax1 and ax2, we need an angular velocity
			// along u to cover angle erp*theta in one step :
			//   |angular_velocity| = angle/time = erp*theta / stepsize
			//                      = (erp*fps) * theta
			//    angular_velocity  = |angular_velocity| * (ax1 x ax2) / |ax1 x ax2|
			//                      = (erp*fps) * theta * (ax1 x ax2) / sin(theta)
			// ...as ax1 and ax2 are unit length. if theta is smallish,
			// theta ~= sin(theta), so
			//    angular_velocity  = (erp*fps) * (ax1 x ax2)
			// ax1 x ax2 is in the plane space of ax1, so we project the angular
			// velocity to p and q to find the right hand side.
			//	double k = info.fps * info.erp * getSoftnessOrthoAng();
			double currERP = (( m_flags & btSliderFlags.BT_SLIDER_FLAGS_ERP_ORTANG) != 0 )? m_softnessOrthoAng: m_softnessOrthoAng* info.erp;
			double k = info.fps * currERP;

			btVector3 u = ax1A.cross( ax1B );
			info.m_solverConstraints[0].m_rhs = k * u.dot( p );
			info.m_solverConstraints[1].m_rhs = k * u.dot( q );
			if( ( m_flags & btSliderFlags.BT_SLIDER_FLAGS_CFM_ORTANG) != 0 )
	{
				info.m_solverConstraints[0].m_cfm = m_cfmOrthoAng;
				info.m_solverConstraints[1].m_cfm = m_cfmOrthoAng;
			}

			int nrow = 1; // last filled row
			//int srow;
			double limit_err;
			int limit;
			bool powered;

			// next two rows. 
			// we want: velA + wA x relA == velB + wB x relB ... but this would
			// result in three equations, so we project along two orthos to the slider axis

			btTransform bodyA_trans = transA;
			btTransform bodyB_trans = transB;
			nrow++;
			//int s2 = nrow * s;
			nrow++;
			//int s3 = nrow * s;

			btVector3 tmpA = btVector3.Zero, tmpB = btVector3.Zero
				, relA = btVector3.Zero, relB = btVector3.Zero, c = btVector3.Zero;
			if( m_useOffsetForConstraintFrame )
			{
				// get vector from bodyB to frameB in WCS
				relB = m_calculatedTransformB.m_origin - bodyB_trans.m_origin;
				// get its projection to slider axis
				btVector3 projB = ax1 * relB.dot( ax1 );
				// get vector directed from bodyB to slider axis (and orthogonal to it)
				btVector3 orthoB = relB - projB;
				// same for bodyA
				relA = m_calculatedTransformA.m_origin - bodyA_trans.m_origin;
				btVector3 projA = ax1 * relA.dot( ax1 );
				btVector3 orthoA = relA - projA;
				// get desired offset between frames A and B along slider axis
				double sliderOffs = m_linPos - m_depth[0];
				// desired vector from projection of center of bodyA to projection of center of bodyB to slider axis
				btVector3 totalDist = projA + ax1 * sliderOffs - projB;
				// get offset vectors relA and relB
				relA = orthoA + totalDist * factA;
				relB = orthoB - totalDist * factB;
				// now choose average ortho to slider axis
				p = orthoB * factA + orthoA * factB;
				double len2 = p.length2();
				if( len2 > btScalar.SIMD_EPSILON )
				{
					p /= btScalar.btSqrt( len2 );
				}
				else
				{
					p = m_calculatedTransformA.m_basis.getColumn( 1 );
				}
				// make one more ortho
				q = ax1.cross( p );
				// fill two rows
				tmpA = relA.cross( p );
				tmpB = relB.cross( p );
				info.m_solverConstraints[2].m_relpos1CrossNormal = tmpA;
				tmpB.Invert( out info.m_solverConstraints[2].m_relpos2CrossNormal );
				//for( i = 0; i < 3; i++ ) info.m_J1angularAxis[s2 + i] = tmpA[i];
				//for( i = 0; i < 3; i++ ) info.m_J2angularAxis[s2 + i] = -tmpB[i];
				tmpA = relA.cross( q );
				tmpB = relB.cross( q );
				if( hasStaticBody && getSolveAngLimit() )
				{ // to make constraint between static and dynamic objects more rigid
				  // remove wA (or wB) from equation if angular limit is hit
					tmpB *= factB;
					tmpA *= factA;
				}
				info.m_solverConstraints[3].m_relpos1CrossNormal = tmpA;
				tmpB.Invert( out info.m_solverConstraints[3].m_relpos2CrossNormal );
				//for( i = 0; i < 3; i++ ) info.m_J1angularAxis[s3 + i] = tmpA[i];
				//for( i = 0; i < 3; i++ ) info.m_J2angularAxis[s3 + i] = -tmpB[i];
				info.m_solverConstraints[2].m_contactNormal1 = p;
				info.m_solverConstraints[3].m_contactNormal1 = q;
				p.Invert( out info.m_solverConstraints[2].m_contactNormal2 );
				q.Invert( out info.m_solverConstraints[3].m_contactNormal2 );
				//for( i = 0; i < 3; i++ ) info.m_J1linearAxis[s2 + i] = p[i];
				//for( i = 0; i < 3; i++ ) info.m_J1linearAxis[s3 + i] = q[i];
				//for( i = 0; i < 3; i++ ) info.m_J2linearAxis[s2 + i] = -p[i];
				//for( i = 0; i < 3; i++ ) info.m_J2linearAxis[s3 + i] = -q[i];
			}
			else
			{   // old way - maybe incorrect if bodies are not on the slider axis
				// see discussion "Bug in slider constraint" http://bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=4024&start=0
				c = bodyB_trans.m_origin - bodyA_trans.m_origin;
				btVector3 tmp = c.cross( p );
				tmp.Mult( factA, out info.m_solverConstraints[2].m_relpos1CrossNormal );
				//for( i = 0; i < 3; i++ ) info.m_J1angularAxis[s2 + i] = factA * tmp[i];
				tmp.Mult( factB, out info.m_solverConstraints[2].m_relpos2CrossNormal );
				//for( i = 0; i < 3; i++ ) info.m_J2angularAxis[s2 + i] = factB * tmp[i];
				tmp = c.cross( q );
				tmp.Mult( factA, out info.m_solverConstraints[3].m_relpos1CrossNormal );
				//for( i = 0; i < 3; i++ ) info.m_J1angularAxis[s3 + i] = factA * tmp[i];
				tmp.Mult( factB, out info.m_solverConstraints[3].m_relpos2CrossNormal );
				//for( i = 0; i < 3; i++ ) info.m_J2angularAxis[s3 + i] = factB * tmp[i];

				info.m_solverConstraints[2].m_contactNormal1 = p;
				info.m_solverConstraints[3].m_contactNormal1 = q;
				p.Invert( out info.m_solverConstraints[2].m_contactNormal2 );
				q.Invert( out info.m_solverConstraints[3].m_contactNormal2 );
				//for( i = 0; i < 3; i++ ) info.m_J1linearAxis[s2 + i] = p[i];
				//for( i = 0; i < 3; i++ ) info.m_J1linearAxis[s3 + i] = q[i];
				//for( i = 0; i < 3; i++ ) info.m_J2linearAxis[s2 + i] = -p[i];
				//for( i = 0; i < 3; i++ ) info.m_J2linearAxis[s3 + i] = -q[i];
			}
			// compute two elements of right hand side

			//	k = info.fps * info.erp * getSoftnessOrthoLin();
			currERP = ( ( m_flags & btSliderFlags.BT_SLIDER_FLAGS_ERP_ORTLIN )!=0) ? m_softnessOrthoLin : m_softnessOrthoLin * info.erp;
			k = info.fps * currERP;

			double rhs = k * p.dot( ofs );
			info.m_solverConstraints[2].m_rhs = rhs;
			//info.m_constraintError[s2] = rhs;
			rhs = k * q.dot( ofs );
			info.m_solverConstraints[3].m_rhs = rhs;
			//info.m_constraintError[s3] = rhs;
			if(( m_flags & btSliderFlags.BT_SLIDER_FLAGS_CFM_ORTLIN )!= 0)
			{
				info.m_solverConstraints[2].m_cfm = m_cfmOrthoLin;
				info.m_solverConstraints[3].m_cfm = m_cfmOrthoLin;
				//info.cfm[s2] = m_cfmOrthoLin;
				//info.cfm[s3] = m_cfmOrthoLin;
			}


			// check linear limits
			limit_err = (double)( 0.0 );
			limit = 0;
			if( getSolveLinLimit() )
			{
				limit_err = getLinDepth() * signFact;
				limit = ( limit_err > (double)( 0.0 ) ) ? 2 : 1;
			}
			powered = false;
			if( getPoweredLinMotor() )
			{
				powered = true;
			}
			// if the slider has joint limits or motor, add in the extra row
			if( limit != 0 || powered )
			{
				nrow++;
				//srow = nrow * info.rowskip;
				info.m_solverConstraints[4].m_contactNormal1 = ax1;
				//info.m_J1linearAxis[srow + 0] = ax1[0];
				//info.m_J1linearAxis[srow + 1] = ax1[1];
				//info.m_J1linearAxis[srow + 2] = ax1[2];
				ax1.Invert( out info.m_solverConstraints[4].m_contactNormal2 );
				//info.m_J2linearAxis[srow + 0] = -ax1[0];
				//info.m_J2linearAxis[srow + 1] = -ax1[1];
				//info.m_J2linearAxis[srow + 2] = -ax1[2];
				// linear torque decoupling step:
				//
				// we have to be careful that the linear constraint forces (+/- ax1) applied to the two bodies
				// do not create a torque couple. in other words, the points that the
				// constraint force is applied at must lie along the same ax1 axis.
				// a torque couple will result in limited slider-jointed free
				// bodies from gaining angular momentum.
				if( m_useOffsetForConstraintFrame )
				{
					// this is needed only when bodyA and bodyB are both dynamic.
					if( !hasStaticBody )
					{
						tmpA = relA.cross( ax1 );
						tmpB = relB.cross( ax1 );
						info.m_solverConstraints[4].m_relpos1CrossNormal = tmpA;
						//info.m_J1angularAxis[srow + 0] = tmpA[0];
						//info.m_J1angularAxis[srow + 1] = tmpA[1];
						//info.m_J1angularAxis[srow + 2] = tmpA[2];
						tmpB.Invert( out info.m_solverConstraints[4].m_relpos2CrossNormal );
						//info.m_J2angularAxis[srow + 0] = -tmpB[0];
						//info.m_J2angularAxis[srow + 1] = -tmpB[1];
						//info.m_J2angularAxis[srow + 2] = -tmpB[2];
					}
				}
				else
				{ // The old way. May be incorrect if bodies are not on the slider axis
					btVector3 ltd;  // Linear Torque Decoupling vector (a torque)
					ltd = c.cross( ax1 );
					ltd.Mult( factA, out info.m_solverConstraints[4].m_relpos1CrossNormal );
					//info.m_J1angularAxis[srow + 0] = factA * ltd[0];
					//info.m_J1angularAxis[srow + 1] = factA * ltd[1];
					//info.m_J1angularAxis[srow + 2] = factA * ltd[2];
					ltd.Mult( factB, out info.m_solverConstraints[4].m_relpos2CrossNormal );
					//info.m_J2angularAxis[srow + 0] = factB * ltd[0];
					//info.m_J2angularAxis[srow + 1] = factB * ltd[1];
					//info.m_J2angularAxis[srow + 2] = factB * ltd[2];
				}
				// right-hand part
				double lostop = getLowerLinLimit();
				double histop = getUpperLinLimit();
				if( ( limit!= 0) && ( lostop == histop ) )
				{  // the joint motor is ineffective
					powered = false;
				}
				info.m_solverConstraints[4].m_rhs = 0;
				info.m_solverConstraints[4].m_lowerLimit = 0;
				info.m_solverConstraints[4].m_upperLimit = 0;
				currERP = (( m_flags & btSliderFlags.BT_SLIDER_FLAGS_ERP_LIMLIN ) != 0 )? m_softnessLimLin : info.erp;
				if( powered )
				{
					if(( m_flags & btSliderFlags.BT_SLIDER_FLAGS_CFM_DIRLIN ) != 0 )
					{
						info.m_solverConstraints[4].m_cfm = m_cfmDirLin;
					}
					double tag_vel = getTargetLinMotorVelocity();
					double mot_fact = getMotorFactor( m_linPos, m_lowerLinLimit, m_upperLinLimit, tag_vel, info.fps * currERP );
					info.m_solverConstraints[4].m_rhs -= signFact * mot_fact * getTargetLinMotorVelocity();
					info.m_solverConstraints[4].m_lowerLimit += -getMaxLinMotorForce() / info.fps;
					info.m_solverConstraints[4].m_upperLimit += getMaxLinMotorForce() / info.fps;
				}
				if( limit != 0 )
				{
					k = info.fps * currERP;
					info.m_solverConstraints[4].m_rhs += k * limit_err;
					if( ( m_flags & btSliderFlags.BT_SLIDER_FLAGS_CFM_LIMLIN ) != 0 )
					{
						info.m_solverConstraints[4].m_cfm = m_cfmLimLin;
					}
					if( lostop == histop )
					{   // limited low and high simultaneously
						info.m_solverConstraints[4].m_lowerLimit = btScalar.BT_MIN_FLOAT;
						info.m_solverConstraints[4].m_upperLimit = btScalar.BT_MAX_FLOAT;
					}
					else if( limit == 1 )
					{ // low limit
						info.m_solverConstraints[4].m_lowerLimit = btScalar.BT_MIN_FLOAT;
						info.m_solverConstraints[4].m_upperLimit = 0;
					}
					else
					{ // high limit
						info.m_solverConstraints[4].m_lowerLimit = 0;
						info.m_solverConstraints[4].m_upperLimit = btScalar.BT_MAX_FLOAT;
					}
					// bounce (we'll use slider parameter abs(1.0 - m_dampingLimLin) for that)
					double bounce = btScalar.btFabs( (double)( 1.0 ) - getDampingLimLin() );
					if( bounce > (double)( 0.0 ) )
					{
						double vel = linVelA.dot( ax1 );
						vel -= linVelB.dot( ax1 );
						vel *= signFact;
						// only apply bounce if the velocity is incoming, and if the
						// resulting c[] exceeds what we already have.
						if( limit == 1 )
						{   // low limit
							if( vel < 0 )
							{
								double newc = -bounce * vel;
								if( newc > info.m_solverConstraints[nrow].m_rhs )
								{
									info.m_solverConstraints[nrow].m_rhs = newc;
								}
							}
						}
						else
						{ // high limit - all those computations are reversed
							if( vel > 0 )
							{
								double newc = -bounce * vel;
								if( newc < info.m_solverConstraints[nrow].m_rhs )
								{
									info.m_solverConstraints[nrow].m_rhs = newc;
								}
							}
						}
					}
					info.m_solverConstraints[4].m_rhs *= getSoftnessLimLin();
				} // if(limit)
			} // if linear limit
			  // check angular limits
			limit_err = (double)( 0.0 );
			limit = 0;
			if( getSolveAngLimit() )
			{
				limit_err = getAngDepth();
				limit = ( limit_err > (double)( 0.0 ) ) ? 1 : 2;
			}
			// if the slider has joint limits, add in the extra row
			powered = false;
			if( getPoweredAngMotor() )
			{
				powered = true;
			}
			if( limit!=0 || powered )
			{
				nrow++;
				//srow = nrow * info.rowskip;
				info.m_solverConstraints[nrow].m_relpos1CrossNormal = ax1;
				//info.m_J1angularAxis[srow + 0] = ax1[0];
				//info.m_J1angularAxis[srow + 1] = ax1[1];
				//info.m_J1angularAxis[srow + 2] = ax1[2];

				ax1.Invert( out info.m_solverConstraints[nrow].m_relpos2CrossNormal );
				//info.m_J2angularAxis[srow + 0] = -ax1[0];
				//info.m_J2angularAxis[srow + 1] = -ax1[1];
				//info.m_J2angularAxis[srow + 2] = -ax1[2];

				double lostop = getLowerAngLimit();
				double histop = getUpperAngLimit();
				if( limit != 0 && ( lostop == histop ) )
				{  // the joint motor is ineffective
					powered = false;
				}
				currERP = ( m_flags & btSliderFlags.BT_SLIDER_FLAGS_ERP_LIMANG ) != 0 ? m_softnessLimAng : info.erp;
				if( powered )
				{
					if(( m_flags & btSliderFlags.BT_SLIDER_FLAGS_CFM_DIRANG )!= 0)
					{
						info.m_solverConstraints[nrow].m_cfm = m_cfmDirAng;
					}
					double mot_fact = getMotorFactor( m_angPos, m_lowerAngLimit, m_upperAngLimit, getTargetAngMotorVelocity(), info.fps * currERP );
					info.m_solverConstraints[nrow].m_rhs = mot_fact * getTargetAngMotorVelocity();
					info.m_solverConstraints[nrow].m_lowerLimit = -getMaxAngMotorForce() / info.fps;
					info.m_solverConstraints[nrow].m_upperLimit = getMaxAngMotorForce() / info.fps;
				}
				if( limit != 0 )
				{
					k = info.fps * currERP;
					info.m_solverConstraints[nrow].m_rhs += k * limit_err;
					if( ( m_flags & btSliderFlags.BT_SLIDER_FLAGS_CFM_LIMANG ) != 0 )
					{
						info.m_solverConstraints[nrow].m_cfm = m_cfmLimAng;
					}
					if( lostop == histop )
					{
						// limited low and high simultaneously
						info.m_solverConstraints[nrow].m_lowerLimit = btScalar.BT_MIN_FLOAT;
						info.m_solverConstraints[nrow].m_upperLimit = btScalar.BT_MAX_FLOAT;
					}
					else if( limit == 1 )
					{ // low limit
						info.m_solverConstraints[nrow].m_lowerLimit = 0;
						info.m_solverConstraints[nrow].m_upperLimit = btScalar.BT_MAX_FLOAT;
					}
					else
					{ // high limit
						info.m_solverConstraints[nrow].m_lowerLimit = btScalar.BT_MIN_FLOAT;
						info.m_solverConstraints[nrow].m_upperLimit = 0;
					}
					// bounce (we'll use slider parameter abs(1.0 - m_dampingLimAng) for that)
					double bounce = btScalar.btFabs( (double)( 1.0 ) - getDampingLimAng() );
					if( bounce > (double)( 0.0 ) )
					{
						double vel = m_rbA.getAngularVelocity().dot( ax1 );
						vel -= m_rbB.getAngularVelocity().dot( ax1 );
						// only apply bounce if the velocity is incoming, and if the
						// resulting c[] exceeds what we already have.
						if( limit == 1 )
						{   // low limit
							if( vel < 0 )
							{
								double newc = -bounce * vel;
								if( newc > info.m_solverConstraints[nrow].m_rhs )
								{
									info.m_solverConstraints[nrow].m_rhs = newc;
								}
							}
						}
						else
						{   // high limit - all those computations are reversed
							if( vel > 0 )
							{
								double newc = -bounce * vel;
								if( newc < info.m_solverConstraints[nrow].m_rhs )
								{
									info.m_solverConstraints[nrow].m_rhs = newc;
								}
							}
						}
					}
					info.m_solverConstraints[nrow].m_rhs *= getSoftnessLimAng();
				} // if(limit)
			} // if angular limit or powered
		}


		///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
		///If no axis is provided, it uses the default axis for this constraint.
		internal override void setParam( btConstraintParams num, double value, int axis )
		{
			switch( num )
			{
				case btConstraintParams.BT_CONSTRAINT_STOP_ERP:
					if( axis < 1 )
					{
						m_softnessLimLin = value;
						m_flags |= btSliderFlags.BT_SLIDER_FLAGS_ERP_LIMLIN;
					}
					else if( axis < 3 )
					{
						m_softnessOrthoLin = value;
						m_flags |= btSliderFlags.BT_SLIDER_FLAGS_ERP_ORTLIN;
					}
					else if( axis == 3 )
					{
						m_softnessLimAng = value;
						m_flags |= btSliderFlags.BT_SLIDER_FLAGS_ERP_LIMANG;
					}
					else if( axis < 6 )
					{
						m_softnessOrthoAng = value;
						m_flags |= btSliderFlags.BT_SLIDER_FLAGS_ERP_ORTANG;
					}
					else
					{
						btAssertConstrParams( false );
					}
					break;
				case btConstraintParams.BT_CONSTRAINT_CFM:
					if( axis < 1 )
					{
						m_cfmDirLin = value;
						m_flags |= btSliderFlags.BT_SLIDER_FLAGS_CFM_DIRLIN;
					}
					else if( axis == 3 )
					{
						m_cfmDirAng = value;
						m_flags |= btSliderFlags.BT_SLIDER_FLAGS_CFM_DIRANG;
					}
					else
					{
						btAssertConstrParams( false );
					}
					break;
				case btConstraintParams.BT_CONSTRAINT_STOP_CFM:
					if( axis < 1 )
					{
						m_cfmLimLin = value;
						m_flags |= btSliderFlags.BT_SLIDER_FLAGS_CFM_LIMLIN;
					}
					else if( axis < 3 )
					{
						m_cfmOrthoLin = value;
						m_flags |= btSliderFlags.BT_SLIDER_FLAGS_CFM_ORTLIN;
					}
					else if( axis == 3 )
					{
						m_cfmLimAng = value;
						m_flags |= btSliderFlags.BT_SLIDER_FLAGS_CFM_LIMANG;
					}
					else if( axis < 6 )
					{
						m_cfmOrthoAng = value;
						m_flags |= btSliderFlags.BT_SLIDER_FLAGS_CFM_ORTANG;
					}
					else
					{
						btAssertConstrParams( false );
					}
					break;
			}
		}

		///return the local value of parameter
		internal override double getParam( btConstraintParams num, int axis )
		{
			double retVal = ( btScalar.BT_LARGE_FLOAT );
			switch( num )
			{
				case btConstraintParams.BT_CONSTRAINT_STOP_ERP:
					if( axis < 1 )
					{
						btAssertConstrParams( ( m_flags & btSliderFlags.BT_SLIDER_FLAGS_ERP_LIMLIN ) != 0);
						retVal = m_softnessLimLin;
					}
					else if( axis < 3 )
					{
						btAssertConstrParams( ( m_flags & btSliderFlags.BT_SLIDER_FLAGS_ERP_ORTLIN ) != 0);
						retVal = m_softnessOrthoLin;
					}
					else if( axis == 3 )
					{
						btAssertConstrParams( ( m_flags & btSliderFlags.BT_SLIDER_FLAGS_ERP_LIMANG ) != 0);
						retVal = m_softnessLimAng;
					}
					else if( axis < 6 )
					{
						btAssertConstrParams( ( m_flags & btSliderFlags.BT_SLIDER_FLAGS_ERP_ORTANG ) != 0);
						retVal = m_softnessOrthoAng;
					}
					else
					{
						btAssertConstrParams( false );
					}
					break;
				case btConstraintParams.BT_CONSTRAINT_CFM:
					if( axis < 1 )
					{
						btAssertConstrParams( ( m_flags & btSliderFlags.BT_SLIDER_FLAGS_CFM_DIRLIN ) != 0);
						retVal = m_cfmDirLin;
					}
					else if( axis == 3 )
					{
						btAssertConstrParams( ( m_flags & btSliderFlags.BT_SLIDER_FLAGS_CFM_DIRANG ) != 0);
						retVal = m_cfmDirAng;
					}
					else
					{
						btAssertConstrParams( false );
					}
					break;
				case btConstraintParams.BT_CONSTRAINT_STOP_CFM:
					if( axis < 1 )
					{
						btAssertConstrParams( (m_flags & btSliderFlags.BT_SLIDER_FLAGS_CFM_LIMLIN ) != 0 );
						retVal = m_cfmLimLin;
					}
					else if( axis < 3 )
					{
						btAssertConstrParams( (m_flags & btSliderFlags.BT_SLIDER_FLAGS_CFM_ORTLIN ) != 0 );
						retVal = m_cfmOrthoLin;
					}
					else if( axis == 3 )
					{
						btAssertConstrParams( (m_flags & btSliderFlags.BT_SLIDER_FLAGS_CFM_LIMANG ) != 0 );
						retVal = m_cfmLimAng;
					}
					else if( axis < 6 )
					{
						btAssertConstrParams( (m_flags & btSliderFlags.BT_SLIDER_FLAGS_CFM_ORTANG ) != 0 );
						retVal = m_cfmOrthoAng;
					}
					else
					{
						btAssertConstrParams( false );
					}
					break;
			}
			return retVal;
		}





#if SERIALIZE_DONE
	virtual	int	calculateSerializeBufferSize();

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	string	serialize(object dataBuffer, btSerializer* serializer);
#endif

	};


#if SERIALIZE_DONE
	///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64


	struct btSliderConstraintData
{
	btTypedConstraintData	m_typeConstraintData;
	btTransformFloatData m_rbAFrame; // constraint axii. Assumes z is hinge axis.
	btTransformFloatData m_rbBFrame;
	
	float	m_linearUpperLimit;
	float	m_linearLowerLimit;

	float	m_angularUpperLimit;
	float	m_angularLowerLimit;

	int	m_useLinearReferenceFrameA;
	int m_useOffsetForConstraintFrame;

};


struct btSliderConstraintDoubleData
{
	btTypedConstraintDoubleData	m_typeConstraintData;
	btTransformDoubleData m_rbAFrame; // constraint axii. Assumes z is hinge axis.
	btTransformDoubleData m_rbBFrame;
	
	double	m_linearUpperLimit;
	double	m_linearLowerLimit;

	double	m_angularUpperLimit;
	double	m_angularLowerLimit;

	int	m_useLinearReferenceFrameA;
	int m_useOffsetForConstraintFrame;

};

public		int	calculateSerializeBufferSize()
{
	return sizeof(btSliderConstraintData2);
}

	///fills the dataBuffer and returns the struct name (and 0 on failure)
public	string	serialize(object dataBuffer, btSerializer* serializer)
{

	btSliderConstraintData2* sliderData = (btSliderConstraintData2*) dataBuffer;
	btTypedConstraint::serialize(&sliderData.m_typeConstraintData,serializer);

	m_frameInA.serialize(sliderData.m_rbAFrame);
	m_frameInB.serialize(sliderData.m_rbBFrame);

	sliderData.m_linearUpperLimit = m_upperLinLimit;
	sliderData.m_linearLowerLimit = m_lowerLinLimit;

	sliderData.m_angularUpperLimit = m_upperAngLimit;
	sliderData.m_angularLowerLimit = m_lowerAngLimit;

	sliderData.m_useLinearReferenceFrameA = m_useLinearReferenceFrameA;
	sliderData.m_useOffsetForConstraintFrame = m_useOffsetForConstraintFrame;

	return btSliderConstraintDataName;
}

#endif

}
