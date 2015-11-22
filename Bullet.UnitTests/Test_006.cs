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
	public class Test_006 : ITest
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
			{
				btCollisionShape fallShape = new btBoxShape( ref btVector3.One );

				btVector3 origin = new btVector3( -1, 10, 0 );
				btTransform init = new btTransform( ref btQuaternion.Identity, ref origin );
				btDefaultMotionState fallMotionState = new btDefaultMotionState( ref init );

				btScalar mass = 1;
				btVector3 fallInertia;
				fallShape.calculateLocalInertia( mass, out fallInertia );

				btRigidBody.btRigidBodyConstructionInfo
					fallingRigidBodyCI = new btRigidBody.btRigidBodyConstructionInfo( mass, fallMotionState
								, fallShape, ref fallInertia );

				fallingRigidBody = new btRigidBody( fallingRigidBodyCI );

				world.addRigidBody( fallingRigidBody );
			}

			//-------------------------------------------------------
			{
				btCollisionShape staticShape = new btBoxShape( ref btVector3.One );

				btVector3 origin = new btVector3( -1, 4, 0 );
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
			}
			//-------------------------------------------------------

			{
				btCompoundShape fallShape2 = new btCompoundShape();

				btVector3 origin = new btVector3( 3, 40, 0 );
				btTransform init = new btTransform( btQuaternion.Identity, origin );
				btDefaultMotionState fallMotionState = new btDefaultMotionState( ref init );

				btCollisionShape part = new btBoxShape( ref btVector3.One );
				{
					int x, y, z;
					for( x = -2; x <= 2; x++ )
						for( y = -2; y <= 2; y++ )
							for( z = -2; z <= 2; z++ )
							{
								tmp = btVector3.Zero;
								tmp.AddScale( ref btVector3.xAxis, x * 2, out tmp );
								tmp.AddScale( ref btVector3.yAxis, y * 2, out tmp );
								tmp.AddScale( ref btVector3.zAxis, z * 2, out tmp );
								init.setOrigin( ref tmp );
								fallShape2.addChildShape( ref init, part );
							}
				}

				double mass = 125;// .001;
				btVector3 fallInertia;
				fallShape2.calculateLocalInertia( mass, out fallInertia ); // fills fallInertia

				btRigidBody.btRigidBodyConstructionInfo
					fallingRigidBodyCI = new btRigidBody.btRigidBodyConstructionInfo( mass, fallMotionState
						, fallShape2, ref fallInertia );



				fallingRigidBody2 = new btRigidBody( fallingRigidBodyCI );

				world.addRigidBody( fallingRigidBody2 );
			}

			if( false )
			{
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
		}

		//-------------------------------------------------------

		public void Reset()
		{
			btVector3 origin = new btVector3( -1, 10, 0 );
			btTransform init = new btTransform( ref btQuaternion.Identity, ref origin );
			fallingRigidBody.setWorldTransform( ref init );
			fallingRigidBody.setAngularVelocity( ref btVector3.Zero );
			fallingRigidBody.setLinearVelocity( ref btVector3.Zero );
			origin.x = 3;
			origin.y = 50;
			init.setOrigin( ref origin );
			fallingRigidBody2.setWorldTransform( ref init );
			fallingRigidBody2.setAngularVelocity( ref btVector3.Zero );
			fallingRigidBody2.setLinearVelocity( ref btVector3.Zero );
			step = 0;
		}

		//-------------------------------------------------------

		internal void Tick()
		{

			{
				if( step == 142 )
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

		public static Test_006 runningInstance;

		static public void Run( bool timing_only, bool sloped )
		{
			Test_006 test = new Test_006();
			test.timing_only = timing_only;
			runningInstance = test;
			btScalar.LoggingFlags = DbgFlag.PredictedTransform;
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
