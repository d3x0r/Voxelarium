using Bullet.Collision.BroadPhase;
using Bullet.Collision.Shapes;
using Bullet.Debug.OpenGL2;
using Bullet.Dynamics;
using Bullet.Dynamics.ConstraintSolver;
using Bullet.LinearMath;
using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;
//using BulletSharp;
namespace Bullet.UnitTests
{
	public class Test_005 : ITest
	{
		int step;
		bool sloped;
		bool timing_only;

		btDiscreteDynamicsWorld world;
		btRigidBody fallingRigidBody;
		btRigidBody fallingRigidBody2;
		btRigidBody[] staticRigidBody = new btRigidBody[10];
		void Setup()
		{
			world = new btDiscreteDynamicsWorld();
			world.setDebugDrawer( Program.Drawer );

			btVector3 tmp; btVector3.yAxis.Add( ref btVector3.xAxis, out tmp );
			tmp.normalized( out tmp );
			btCollisionShape groundShape;
			if( sloped )
				groundShape = new btStaticPlaneShape( ref btVector3.Zero, ref tmp );
			else
				groundShape = new btStaticPlaneShape( ref btVector3.Zero, ref btVector3.yAxis );

			btDefaultMotionState groundMotionState = new btDefaultMotionState();

			btRigidBody.btRigidBodyConstructionInfo
				groundRigidBodyCI = new btRigidBody.btRigidBodyConstructionInfo( 0, groundMotionState
							, groundShape, ref btVector3.Zero );
			btRigidBody groundRigidBody = new btRigidBody( groundRigidBodyCI );
			world.addRigidBody( groundRigidBody );

			//-------------------------------------------------------

			btCollisionShape staticShape = new btBoxShape( ref btVector3.One );

			btVector3 origin = new btVector3( -3, 4, 0 );
			btTransform init = new btTransform( ref btQuaternion.Identity, ref origin );
			btDefaultMotionState fallMotionState = new btDefaultMotionState( ref init );

			btScalar mass = 0;
			btVector3 fallInertia;
			staticShape.calculateLocalInertia( mass, out fallInertia );

			btRigidBody.btRigidBodyConstructionInfo
				fallingRigidBodyCI = new btRigidBody.btRigidBodyConstructionInfo( mass, fallMotionState
							, staticShape, ref fallInertia );

			staticRigidBody[0] = new btRigidBody( fallingRigidBodyCI );

			world.addRigidBody( staticRigidBody[0] );

			//-------------------------------------------------------

			btCollisionShape fallShape = new btBoxShape( ref btVector3.One );

			origin = new btVector3( -3, 10, 0 );
			init = new btTransform( ref btQuaternion.Identity, ref origin );
			fallMotionState = new btDefaultMotionState( ref init );

			mass = 1;
			fallShape.calculateLocalInertia( mass, out fallInertia );

			fallingRigidBodyCI = new btRigidBody.btRigidBodyConstructionInfo( mass, fallMotionState
							, fallShape, ref fallInertia );

			fallingRigidBody = new btRigidBody( fallingRigidBodyCI );

			world.addRigidBody( fallingRigidBody );

			//-------------------------------------------------------

			btCollisionShape fallShape2 = new btBoxShape( ref btVector3.One );

			origin = new btVector3( 3, 10, 0 );
			init = new btTransform( ref btQuaternion.Identity, ref origin );
			fallMotionState = new btDefaultMotionState( ref init );

			mass = 1;
			fallShape2.calculateLocalInertia( mass, out fallInertia );

			fallingRigidBodyCI = new btRigidBody.btRigidBodyConstructionInfo( mass, fallMotionState
							, fallShape2, ref fallInertia );

			fallingRigidBody2 = new btRigidBody( fallingRigidBodyCI );

			world.addRigidBody( fallingRigidBody2 );

			//---------------------------------------------------
			// Hinge them together
			btVector3 pivotInA; btVector3.xAxis.Mult( 3, out pivotInA );
			btVector3 axisInA = btVector3.yAxis;
			btVector3 pivotInB; btVector3.xAxis.Mult( -3, out pivotInB );
			btVector3 axisInB = btVector3.yAxis;

			btHingeConstraint constraint = new btHingeConstraint( fallingRigidBody, fallingRigidBody2
				, ref pivotInA, ref pivotInB, ref axisInA, ref axisInB );

			//constraint.enableMotor( true );
			//constraint.setMotorTargetVelocity( 1 );
			//constraint.setLimit( -btScalar.SIMD_2_PI, btScalar.SIMD_2_PI );

			constraint.enableAngularMotor( true, -1, -0.1 );

			world.addConstraint( constraint );
		}

		//-------------------------------------------------------

		public void Reset()
		{
			btVector3 origin = new btVector3( -3, 10, 0 );
			btTransform init = new btTransform( ref btQuaternion.Identity, ref origin );
			fallingRigidBody.setWorldTransform( ref init );
			origin.x = 3;
			init.setOrigin( ref origin );
			fallingRigidBody2.setWorldTransform( ref init );
			step = 0;

		}

		//-------------------------------------------------------

		internal void Tick()
		{

			{
				if( step == 72 )
				{
					int a = 3;
				}
				world.stepSimulation( 1 / 60.0f, 10 );
				if( !timing_only )
				{
					if( Program.Display != null )
						world.debugDrawWorld();

					btTransform trans;
					fallingRigidBody.getMotionState().getWorldTransform( out trans );
					btTransform trans2;
					fallingRigidBody2.getMotionState().getWorldTransform( out trans2 );

					Console.WriteLine( "Iteration {0}", step );

					Console.WriteLine( "{0}", trans.ToString( "ball orient\t", "\t\t", "ball origin\t" ) );
					btVector3 v = fallingRigidBody.getAngularVelocity();
					Console.WriteLine( "ball Ang Vel : {0}", v );
					v = fallingRigidBody.getLinearVelocity();
					Console.WriteLine( "ball Lin Vel : {0}", v );

					Console.WriteLine( "{0}", trans2.ToString( "ball2 orient\t", "\t\t", "ball2 origin\t" ) );
					v = fallingRigidBody2.getAngularVelocity();
					Console.WriteLine( "ball2 Ang Vel : {0}", v );
					v = fallingRigidBody2.getLinearVelocity();
					Console.WriteLine( "ball2 Lin Vel : {0}", v );
				}
			}
			step++;
			//  world = new DiscreteDynamicsWorld( null, null, null, null );
		}


		static public void Run( bool timing_only, bool sloped )
		{
			Test_005 test = new Test_005();
			test.timing_only = timing_only;
			test.sloped = sloped;
			test.Setup();
			if( Program.Display != null )
			{
				Program.Display.Tick += test.Tick;
				Program.Display.Test = test;
				while( test.step < 1300 )
				{
					Thread.Sleep( 50 );
				}
				Program.Display.Tick -= test.Tick;
			}
			else
			{
				while( test.step < 300 )
					test.Tick();
			}
		}
	}
}
