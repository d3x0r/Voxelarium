﻿using Bullet.Collision.BroadPhase;
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
	public class Test_003
	{
		int step;
		bool sloped;

		btDiscreteDynamicsWorld world;
		btRigidBody fallingRigidBody;
		btRigidBody fallingRigidBody2;

		public void Setup()
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

			btCollisionShape fallShape = new btBoxShape( ref btVector3.One );

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

			btCollisionShape fallShape2 = new btSphereShape( btScalar.BT_ONE );

			origin = new btVector3( sloped?-34:0.25, 1, 0.25 );
			init = new btTransform( ref btQuaternion.Identity, ref origin );
			fallMotionState = new btDefaultMotionState( ref init );

			mass = 1;
			fallShape2.calculateLocalInertia( mass, out fallInertia );

			fallingRigidBodyCI = new btRigidBody.btRigidBodyConstructionInfo( mass, fallMotionState
							, fallShape2, ref fallInertia );

			fallingRigidBody2 = new btRigidBody( fallingRigidBodyCI );

			world.addRigidBody( fallingRigidBody2 );

			//-------------------------------------------------------


		}

		internal void Tick()
		{
			//for( int i = 0; i < 300; i++ )
			{
				if( step == 204 )
				{
					int a = 3;
				}

				world.stepSimulation( 1 / 60.0f, 10 );
				if( Program.Display != null )
					world.debugDrawWorld();

				btTransform trans;
				fallingRigidBody.getMotionState().getWorldTransform( out trans );
				btTransform trans2;
				fallingRigidBody2.getMotionState().getWorldTransform( out trans2 );

				Console.WriteLine( "Iteration {0}", step );
				Console.WriteLine( "{0}", trans.ToString( "cube orient\t", "\t\t", "cube origin\t" ) );
				btIVector3 v = fallingRigidBody.getAngularVelocity();
				Console.WriteLine( "cube Ang Vel : {0}", v );
				v = fallingRigidBody.getLinearVelocity();
				Console.WriteLine( "cube Lin Vel : {0}", v );

				Console.WriteLine( "{0}", trans2.ToString( "ball orient\t", "\t\t", "ball origin\t" ) );
				v = fallingRigidBody2.getAngularVelocity();
				Console.WriteLine( "ball Ang Vel : {0}", v );
				v = fallingRigidBody2.getLinearVelocity();
				Console.WriteLine( "ball Lin Vel : {0}", v );
				step++;
			}
			//  world = new DiscreteDynamicsWorld( null, null, null, null );
		}

		static public void Run( bool sloped )
		{
			Test_003 test = new Test_003();
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