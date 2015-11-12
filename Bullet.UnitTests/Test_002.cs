using Bullet.Collision.BroadPhase;
using Bullet.Collision.Shapes;
using Bullet.Dynamics;
using Bullet.LinearMath;
using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;
//using BulletSharp;
namespace Bullet.UnitTests
{
	public class Test_002
	{
		int step;
		bool sloped;

		btDiscreteDynamicsWorld world;
		btRigidBody fallingRigidBody;

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


			btCollisionShape fallShape = new btSphereShape( btScalar.BT_ONE );

			btVector3 origin = new btVector3( 0, 50, 0 );
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

		internal void Tick()
		{
			if( step == 188 )
			{
				int a = 3;
			}
			{
				world.stepSimulation( 1 / 60.0f, 10 );
				if( Program.Display != null )
					world.debugDrawWorld();

				btTransform trans;
				fallingRigidBody.getMotionState().getWorldTransform( out trans );

				Console.WriteLine( "Iteration {0}", step );
				Console.WriteLine( "{0}", trans.ToString( "ball orient\t", "\t\t", "ball origin\t" ) );
				btVector3 v = fallingRigidBody.getAngularVelocity();
				Console.WriteLine( "ball Ang Vel : {0}", v );
				v = fallingRigidBody.getLinearVelocity();
				Console.WriteLine( "ball Lin Vel : {0}", v );
			}
			step++;
			//  world = new DiscreteDynamicsWorld( null, null, null, null );
		}

		static public void Run( bool sloped )
		{
			Test_002 test = new Test_002();
			test.sloped = sloped;
			test.Setup();
			if( Program.Display != null )
			{
				Program.Display.Tick += test.Tick;
				while( test.step < 300 )
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