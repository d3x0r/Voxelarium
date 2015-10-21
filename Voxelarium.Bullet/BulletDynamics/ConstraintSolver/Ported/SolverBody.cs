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

namespace Bullet.Dynamics.ConstraintSolver
{
	///Until we get other contributions, only use SIMD on Windows, when using Visual Studio 2008 or later, and not double precision
	//#if BT_USE_SSE
	//#define USE_SIMD 1
	//#endif //



	///The btSolverBody is an internal datastructure for the constraint solver. Only necessary data is packed to increase cache coherence/performance.
	public class btSolverBody
	{

		public btITransform m_worldTransform;
		public btVector3 m_deltaLinearVelocity;
		public btVector3 m_deltaAngularVelocity;
		public btVector3 m_angularFactor;
		public btVector3 m_linearFactor;
		public btVector3 m_invMass;
		public btVector3 m_pushVelocity;
		public btVector3 m_turnVelocity;
		public btVector3 m_linearVelocity;
		public btVector3 m_angularVelocity;
		public btVector3 m_externalForceImpulse;
		public btVector3 m_externalTorqueImpulse;

		public btRigidBody m_originalBody;

		void setWorldTransform( ref btTransform worldTransform )
		{
			m_worldTransform = worldTransform;
		}

		btITransform getWorldTransform( )
		{
			return m_worldTransform;
		}

		/// <summary>
		/// Reset all vectors
		/// </summary>
		public void Clear()
		{
			m_deltaLinearVelocity = btVector3.Zero;
			m_deltaAngularVelocity = btVector3.Zero;
			m_pushVelocity = btVector3.Zero;
			m_turnVelocity = btVector3.Zero;
			m_deltaLinearVelocity = btVector3.Zero;
			m_deltaAngularVelocity = btVector3.Zero;
			m_pushVelocity = btVector3.Zero;
			m_turnVelocity = btVector3.Zero;
		}

		public void getVelocityInLocalPointNoDelta( ref btVector3 rel_pos, out btVector3 velocity )
		{
			if( m_originalBody != null )
			{
				btVector3 tmp;
				btVector3 tmp2;
				m_angularVelocity.Add( ref m_externalTorqueImpulse, out tmp );
				tmp.cross( ref rel_pos, out tmp2 );
				m_linearVelocity.Add( ref m_externalForceImpulse, out tmp );
				tmp2.Add( ref tmp, out velocity );
			}
			else
				btVector3.setValue( out velocity, 0, 0, 0 );
		}


		public void getVelocityInLocalPointObsolete( ref btVector3 rel_pos, ref btVector3 velocity )
		{
			if( m_originalBody != null )
			{
				btVector3 tmp;
				btVector3 tmp2;
				m_angularVelocity.Add( ref m_externalTorqueImpulse, out tmp );
				tmp.cross( ref rel_pos, out tmp2 );
				m_linearVelocity.Add( ref m_externalForceImpulse, out tmp );
				tmp2.Add( ref tmp, out velocity );
				//velocity = m_linearVelocity + m_deltaLinearVelocity 
				//	+ ( m_angularVelocity + m_deltaAngularVelocity ).cross( rel_pos );
			}
			else
				btVector3.setValue( out velocity, 0, 0, 0 );
		}

		public void getAngularVelocity( out btVector3 angVel )
		{
			if( m_originalBody != null )
				m_angularVelocity.Add( ref m_externalTorqueImpulse, out angVel );
			//angVel = m_angularVelocity + m_deltaAngularVelocity;
			else
				btVector3.setValue( out angVel, 0, 0, 0 );
		}


		//Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position
		public void applyImpulse( ref btVector3 linearComponent
								, ref btVector3 angularComponent
								, double impulseMagnitude )
		{
			if( m_originalBody != null )
			{
				btVector3 tmp;
				linearComponent.Mult( ref m_linearFactor, out tmp );
				tmp.Mult( impulseMagnitude, out m_deltaLinearVelocity );
				tmp.Add( ref m_deltaLinearVelocity, out m_deltaLinearVelocity );
				//m_deltaLinearVelocity += linearComponent * impulseMagnitude * m_linearFactor;
				angularComponent.Mult( ref m_angularFactor, out tmp );
				tmp.Mult( impulseMagnitude, out m_deltaAngularVelocity );
				tmp.Add( ref m_deltaAngularVelocity, out m_deltaAngularVelocity );
				//m_deltaAngularVelocity += angularComponent * ( impulseMagnitude * m_angularFactor );
			}
		}

		public void internalApplyPushImpulse( ref btVector3 linearComponent, ref btVector3 angularComponent, double impulseMagnitude )
		{
			if( m_originalBody != null )
			{
				btVector3 tmp;
				linearComponent.Mult( ref m_linearFactor, out tmp );
				tmp.Mult( impulseMagnitude, out tmp );
				tmp.Add( ref m_pushVelocity, out m_pushVelocity );
				//m_pushVelocity += linearComponent * impulseMagnitude * m_linearFactor;
				angularComponent.Mult( ref m_angularFactor, out tmp );
				tmp.Mult( impulseMagnitude, out tmp );
				tmp.Add( ref m_turnVelocity, out m_turnVelocity );
				//m_turnVelocity += angularComponent * ( impulseMagnitude * m_angularFactor );
			}
		}



		public void getDeltaLinearVelocity( out btVector3 result )
		{
			result = m_deltaLinearVelocity;
		}

		public void getDeltaAngularVelocity( out btVector3 result )
		{
			result = m_deltaAngularVelocity;
		}

		public void getPushVelocity( out btVector3 result )
		{
			result = m_pushVelocity;
		}

		public void getTurnVelocity( out btVector3 result )
		{
			result = m_turnVelocity;
		}


		////////////////////////////////////////////////
		///some internal methods, don't use them

		public void internalSetDeltaLinearVelocity( ref btVector3 result )
		{
			m_deltaLinearVelocity = result;
		}

		public void internalSetDeltaAngularVelocity( ref btVector3 result )
		{
			m_deltaAngularVelocity = result ;
		}

		public void internalSetAngularFactor( ref btVector3 result )
		{
			m_angularFactor = result ;
		}

		public void internalSetInvMass( ref btVector3 invMass )
		{
			m_invMass = invMass;
		}

		public void internalSetPushVelocity( ref btVector3 result )
		{
			m_pushVelocity = result;
		}

		public void internalSetTurnVelocity( ref btVector3 result )
		{
			m_turnVelocity = result;
		}
		public void internalGetDeltaLinearVelocity( out btVector3 result )
		{
			result = m_deltaLinearVelocity;
		}

		public void internalGetDeltaAngularVelocity( out btVector3 result )
		{
			result = m_deltaAngularVelocity;
		}

		public void internalGetAngularFactor( out btVector3 result )
		{
			result = m_angularFactor;
		}

		public void internalGetInvMass( out btVector3 result )
		{
			result = m_invMass;
		}
#if !DISABLE_OPERATORS
		public btVector3 internalGetInvMass(  )
		{
			return m_invMass;
		}
#endif

		public void internalGetPushVelocity( out btVector3 result )
		{
			result = m_pushVelocity;
		}

		public void internalGetTurnVelocity( out btVector3 result )
		{
			result = m_turnVelocity;
		}
		/*
		public void internalGetVelocityInLocalPointObsolete( ref btVector3 rel_pos, ref btVector3 velocity )
		{
			velocity = m_linearVelocity + m_deltaLinearVelocity 
				+ ( m_angularVelocity + m_deltaAngularVelocity ).cross( rel_pos );
		}
		*/
		public void internalGetAngularVelocity( out btVector3 angVel )
		{
			m_angularVelocity.Add( ref m_deltaAngularVelocity, out angVel );
			//angVel = m_angularVelocity + m_deltaAngularVelocity;
		}


		//Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position
		public void internalApplyImpulse( ref btVector3 linearComponent, ref btVector3 angularComponent, double impulseMagnitude )
		{
			if( m_originalBody != null )
			{
				btVector3 tmp;
				linearComponent.Mult( ref m_linearFactor, out tmp );
				tmp.Mult( impulseMagnitude, out tmp );
				tmp.Add( ref m_deltaLinearVelocity, out m_deltaLinearVelocity );
				//m_deltaLinearVelocity += linearComponent * impulseMagnitude * m_linearFactor;
				angularComponent.Mult( ref m_angularFactor, out tmp );
				tmp.Mult( impulseMagnitude, out tmp );
				tmp.Add( ref m_deltaAngularVelocity, out m_deltaAngularVelocity );
				//m_deltaAngularVelocity += angularComponent * ( impulseMagnitude * m_angularFactor );
			}
		}



		public void writebackVelocity()
		{
			if( m_originalBody != null )
			{
				m_linearVelocity.Add( ref m_deltaLinearVelocity, out m_linearVelocity );
				//m_linearVelocity += m_deltaLinearVelocity;
				m_angularVelocity.Add( ref m_deltaAngularVelocity, out m_angularVelocity );
				//m_angularVelocity += m_deltaAngularVelocity;

				//m_originalBody.setCompanionId(-1);
			}
		}


		public void writebackVelocityAndTransform( double timeStep, double splitImpulseTurnErp )
		{
			//(void)timeStep;
			if( m_originalBody != null )
			{
				m_linearVelocity.Add( ref m_deltaLinearVelocity, out m_linearVelocity );
				m_angularVelocity.Add( ref m_deltaAngularVelocity, out m_angularVelocity );
				//m_linearVelocity += m_deltaLinearVelocity;
				//m_angularVelocity += m_deltaAngularVelocity;

				//correct the position/orientation based on push/turn recovery
				btTransform newTransform;
				if( m_pushVelocity[0] != 0 || m_pushVelocity[1] != 0 || m_pushVelocity[2] != 0 || m_turnVelocity[0] != 0 || m_turnVelocity[1] != 0 || m_turnVelocity[2] != 0 )
				{
					btVector3 tmp;
					m_turnVelocity.Mult( splitImpulseTurnErp, out tmp );
					//	btQuaternion orn = m_worldTransform.getRotation();
					btTransformUtil.integrateTransform( m_worldTransform, ref m_pushVelocity
						, ref tmp
						, timeStep, out newTransform );
					m_worldTransform = newTransform;
				}
				//m_worldTransform.setRotation(orn);
				//m_originalBody.setCompanionId(-1);
			}
		}



	};

}

