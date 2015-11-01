//	#define QUATERNION_DERIVATIVE
/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

namespace Bullet.LinearMath
{

	/// Utils related to temporal transforms
	class btTransformUtil
	{
		static internal double ANGULAR_MOTION_THRESHOLD = 0.5 * btScalar.SIMD_HALF_PI;



		public static void integrateTransform( btITransform curTrans, btIVector3 linvel, btIVector3 angvel
			, double timeStep, out btTransform predictedTransform )
		{
			btVector3 tmp;
			btVector3 tmp2;
			linvel.Mult( timeStep, out tmp );
			curTrans.getOrigin( out tmp2 );
            tmp2.Add( ref tmp, out predictedTransform.m_origin );
#if QUATERNION_DERIVATIVE
		btQuaternion predictedOrn = curTrans.getRotation();
		predictedOrn += (angvel predictedOrn) * (timeStep 0.5);
		predictedOrn.normalize();
#else
			//Exponential map
			//google for "Practical Parameterization of Rotations Using the Exponential Map", F. Sebastian Grassia

			btVector3 axis;
			double fAngle = angvel.length();
			//limit the angular motion
			if( fAngle * timeStep > ANGULAR_MOTION_THRESHOLD )
			{
				fAngle = ANGULAR_MOTION_THRESHOLD / timeStep;
			}

			if( fAngle < 0.001 )
			{
				// use Taylor's expansions of sync function
				angvel.Mult( ( 0.5 * timeStep - ( timeStep * timeStep * timeStep ) * ( 0.020833333333 ) * fAngle * fAngle ), out axis );
			}
			else
			{
				// sync(fAngle) = sin(cfAngle)/t
				angvel.Mult( ( btScalar.btSin( 0.5 * fAngle * timeStep ) / fAngle ), out axis );
			}
			btQuaternion dorn = new btQuaternion( axis.x, axis.y, axis.z, btScalar.btCos( fAngle * timeStep * 0.5 ) );
			btQuaternion orn0;
			curTrans.getRotation( out orn0 );

			btQuaternion predictedOrn;
			dorn.Mult( ref orn0, out predictedOrn );
			predictedOrn.normalize();
#endif
			btMatrix3x3.setRotation( out predictedTransform.m_basis, ref predictedOrn);
			//predictedTransform.setRotation( ref predictedOrn );
		}

		internal static void calculateVelocityQuaternion( ref btVector3 pos0, ref btVector3 pos1, ref btQuaternion orn0, ref btQuaternion orn1, double timeStep, out btVector3 linVel, out btVector3 angVel )
		{
			btVector3 tmp;
			pos1.Sub( ref pos0, out tmp );
			tmp.Div( timeStep, out linVel );
			btVector3 axis;
			double angle;
			if( !orn0.Equals(ref orn1 ) )
			{
				calculateDiffAxisAngleQuaternion( ref orn0, ref orn1, out axis, out angle );
				axis.Mult( angle, out tmp );
				tmp.Div( timeStep, out angVel );
			}
			else
			{
				angVel = btVector3.Zero;
			}
		}

		internal static void calculateDiffAxisAngleQuaternion( ref btQuaternion orn0, ref btQuaternion orn1a, out btVector3 axis, out double angle )
		{
			btQuaternion orn1;
			orn0.nearest( ref orn1a, out orn1 );
			btQuaternion dorn;
			btQuaternion tmp;
			orn0.inverse( out tmp );
			orn1.Mult( ref tmp, out dorn );
			angle = dorn.getAngle();
			axis.x = dorn.x; axis.y = dorn.y; axis.z = dorn.z; axis.w = 0;
			//check for axis length
			double len = axis.length2();
			if( len < btScalar.SIMD_EPSILON * btScalar.SIMD_EPSILON )
				axis = btVector3.xAxis;
			else
				axis.Div( btScalar.btSqrt( len ), out axis );
		}

		internal static void calculateVelocity( ref btTransform transform0, ref btTransform transform1, double timeStep, out btVector3 linVel, out btVector3 angVel )
		{
			transform1.m_origin.Sub( ref transform0.m_origin, out linVel );
			linVel.Div( timeStep, out linVel );
			btVector3 axis;
			double angle;
			calculateDiffAxisAngle( ref transform0, ref transform1, out axis, out angle );
			axis.Mult( angle, out angVel );
			angVel.Div( timeStep, out angVel );
		}
		internal static void calculateVelocity( btITransform transform0, btITransform transform1, double timeStep, out btVector3 linVel, out btVector3 angVel )
		{
			transform1.getOrigin().Sub( transform0.getOrigin(), out linVel );
			linVel.Div( timeStep, out linVel );
			btVector3 axis;
			double angle;
			calculateDiffAxisAngle( transform0, transform1, out axis, out angle );
			axis.Mult( angle, out angVel );
			angVel.Div( timeStep, out angVel );
		}

		internal static void calculateDiffAxisAngle( ref btTransform transform0, ref btTransform transform1, out btVector3 axis, out double angle )
		{
			btMatrix3x3 tmp_m;
			transform0.m_basis.inverse( out tmp_m );
			btMatrix3x3 dmat;
			transform1.m_basis.Mult( ref tmp_m, out dmat );
			btQuaternion dorn;
			dmat.getRotation( out dorn );

			///floating point inaccuracy can lead to w component > 1..., which breaks 
			dorn.normalize();

			angle = dorn.getAngle();
			axis.x = dorn.x; axis.y = dorn.y; axis.z = dorn.z; axis.w = 0;
			//check for axis length
			double len = axis.length2();
			if( len < btScalar.SIMD_EPSILON * btScalar.SIMD_EPSILON )
				axis = btVector3.xAxis;
			else
				axis.Div( btScalar.btSqrt( len ), out axis );
		}
		internal static void calculateDiffAxisAngle( btITransform transform0, btITransform transform1, out btVector3 axis, out double angle )
		{
			btMatrix3x3 tmp_m;
			transform0.getBasis().inverse( out tmp_m );
			btMatrix3x3 dmat;
			transform1.getBasis().Mult( ref tmp_m, out dmat );
			btQuaternion dorn;
			dmat.getRotation( out dorn );

			///floating point inaccuracy can lead to w component > 1..., which breaks 
			dorn.normalize();

			angle = dorn.getAngle();
			axis.x = dorn.x; axis.y = dorn.y; axis.z = dorn.z; axis.w = 0;
			//check for axis length
			double len = axis.length2();
			if( len < btScalar.SIMD_EPSILON * btScalar.SIMD_EPSILON )
				axis = btVector3.xAxis;
			else
				axis.Div( btScalar.btSqrt( len ), out axis );
		}

	};


	///The btConvexSeparatingDistanceUtil can help speed up convex collision detection 
	///by conservatively updating a cached separating distance/vector instead of re-calculating the closest distance
	public class btConvexSeparatingDistanceUtil
	{
		btQuaternion m_ornA;
		btQuaternion m_ornB;
		btVector3 m_posA;
		btVector3 m_posB;

		btVector3 m_separatingNormal;

		double m_boundingRadiusA;
		double m_boundingRadiusB;
		double m_separatingDistance;



		public btConvexSeparatingDistanceUtil( double boundingRadiusA, double boundingRadiusB )
		{
			m_boundingRadiusA = boundingRadiusA;
			m_boundingRadiusB = boundingRadiusB;
			m_separatingDistance = 0;
		}

		public double getConservativeSeparatingDistance()
		{
			return m_separatingDistance;
		}

		public void updateSeparatingDistance( ref btTransform transA, ref btTransform transB )
		{
			btVector3 toPosA; transA.getOrigin( out toPosA );
			btVector3 toPosB; transB.getOrigin( out toPosB );
			btQuaternion toOrnA;
			transA.getRotation( out toOrnA );
			btQuaternion toOrnB;
			transB.getRotation( out toOrnB );

			if( m_separatingDistance > 0 )
			{


				btVector3 linVelA, angVelA, linVelB, angVelB;
				btTransformUtil.calculateVelocityQuaternion( ref m_posA, ref toPosA, ref m_ornA, ref toOrnA, 1, out linVelA, out angVelA );
				btTransformUtil.calculateVelocityQuaternion( ref m_posB, ref toPosB, ref m_ornB, ref toOrnB, 1, out linVelB, out angVelB );
				double maxAngularProjectedVelocity = angVelA.length() * m_boundingRadiusA + angVelB.length() * m_boundingRadiusB;
				btVector3 relLinVel;
					linVelB.Sub( ref linVelA, out relLinVel );
				double relLinVelocLength = relLinVel.dot( ref m_separatingNormal );
				if( relLinVelocLength < 0 )
				{
					relLinVelocLength = 0;
				}

				double projectedMotion = maxAngularProjectedVelocity + relLinVelocLength;
				m_separatingDistance -= projectedMotion;
			}

			m_posA = toPosA;
			m_posB = toPosB;
			m_ornA = toOrnA;
			m_ornB = toOrnB;
		}

		public void initSeparatingDistance( ref btVector3 separatingVector, double separatingDistance, ref btTransform transA, ref btTransform transB )
		{
			m_separatingDistance = separatingDistance;

			if( m_separatingDistance > 0 )
			{
				m_separatingNormal = separatingVector;

				m_posA = transA.m_origin;
				m_posB = transB.m_origin;
				transA.getRotation( out m_ornA );
				transB.getRotation( out m_ornB );
			}
		}

	}

}
