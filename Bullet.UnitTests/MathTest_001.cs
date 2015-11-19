using Bullet.Collision.BroadPhase;
using Bullet.Collision.Shapes;
using Bullet.Dynamics;
using Bullet.LinearMath;
using System;
using System.Collections.Generic;
using System.Text;
//using BulletSharp;
namespace Bullet.UnitTests
{

    public class MathTest_001
    {
		static void PrintTransform( ref btTransform t )
		{
			int n;
			for( n = 0; n < 3; n++ )
				Console.WriteLine( "   < {0} {1} {2} {3} >", t.m_basis[n][0], t.m_basis[n][1], t.m_basis[n][2], t.m_basis[n][3] );
			Console.WriteLine( "O = <{0},{1},{2}>", t.m_basis[0], t.m_basis[1], t.m_basis[1] );
		}

		public static void Run()
		{

			{
				btTransform planeObjWorld = new btTransform( new btQuaternion( 0.2, 0.1, 0.3, 1 ), new btVector3( 3, 4, 5 ) );
				btTransform convexWorldTransform = new btTransform( new btQuaternion( 0.2, 0.3, 0, 1 ), new btVector3( 3, 4, 5 ) );
				btTransform convexInPlaneTrans;
				btTransform tmp;
				planeObjWorld.inverse( out tmp );
				tmp.Apply( ref convexWorldTransform, out convexInPlaneTrans );
				PrintTransform( ref convexInPlaneTrans );

				planeObjWorld.inverseTimes( ref convexWorldTransform, out convexInPlaneTrans );
				PrintTransform( ref convexInPlaneTrans );
				//convexInPlaneTrans = planeObjWorld( convexWorldTransform;
				//PrintTransform( &convexInPlaneTrans );
			}
			{
				btTransform planeObjWorld = new btTransform( new btQuaternion( 0.2, 0.1, 0.3, 1 ), new btVector3( 5, 2, 1 ) );
				btTransform convexWorldTransform = new btTransform( new btQuaternion( 0.2, 0.3, 0, 1 ), new btVector3( 3, 4, 5 ) );
				btTransform convexInPlaneTrans;
				btTransform tmp;
				planeObjWorld.inverse( out tmp );
				tmp.Apply( ref convexWorldTransform, out convexInPlaneTrans );
				PrintTransform( ref convexInPlaneTrans );

				planeObjWorld.inverseTimes( ref convexWorldTransform, out convexInPlaneTrans );
				PrintTransform( ref convexInPlaneTrans );
				//convexInPlaneTrans = planeObjWorld( convexWorldTransform;
				//PrintTransform( &convexInPlaneTrans );

				btQuaternion perturbeRot = new btQuaternion( 0.1, 0.5, 0.25, 0.8 );
				perturbeRot.normalize();

				btTransform planeObjWrapTrans = new btTransform( new btQuaternion( 0.2, 0.1, 0.3, 1 ), new btVector3( 4, 7, 2 ) );
				planeObjWrapTrans.inverseTimes( ref convexWorldTransform, out convexInPlaneTrans );

				//now perturbe the convex-world transform
				btMatrix3x3 perturbeMat = new btMatrix3x3( ref perturbeRot );
				btMatrix3x3 tmpPerturbe; convexWorldTransform.m_basis.Apply( ref perturbeMat, out tmpPerturbe );
				convexWorldTransform.m_basis = tmpPerturbe;

				btTransform planeInConvex;
				convexWorldTransform.inverseTimes( ref planeObjWrapTrans, out planeInConvex );
				PrintTransform( ref planeInConvex );
			}
			Console.Read();

		}



	}
}
