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
	public class Test_004
	{
		int step;
		bool sloped;
		bool box;
		bool timing_only;

		btDiscreteDynamicsWorld world;
		btRigidBody fallingRigidBody;
		btRigidBody fallingRigidBody2;

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

			//-------------------------------------------------------

			btCollisionShape fallShape2;
			if( !box )
				fallShape2 = new btSphereShape( btScalar.BT_ONE );
			else
				fallShape2 = new btBoxShape( ref btVector3.One );

			origin = new btVector3( sloped ? -34 : 0.25, 1, 0.25 );
			init = new btTransform( ref btQuaternion.Identity, ref origin );
			fallMotionState = new btDefaultMotionState( ref init );

			mass = 1;
			fallShape2.calculateLocalInertia( mass, out fallInertia );

			fallingRigidBodyCI = new btRigidBody.btRigidBodyConstructionInfo( mass, fallMotionState
							, fallShape2, ref fallInertia );

			fallingRigidBody2 = new btRigidBody( fallingRigidBodyCI );

			world.addRigidBody( fallingRigidBody2 );
		}


		//-------------------------------------------------------
		internal void Tick()
		{

			{
				if( step == 198 )
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


		static public void Run( bool timing_only, bool sloped, bool box = false )
		{
			Test_004 test = new Test_004();
			test.timing_only = timing_only;
			test.sloped = sloped;
			test.box = box;
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
