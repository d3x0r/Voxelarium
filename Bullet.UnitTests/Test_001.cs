using Bullet.Collision.BroadPhase;
using Bullet.Collision.Shapes;
using Bullet.Dynamics;
using Bullet.LinearMath;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using System.Threading;
//using BulletSharp;
namespace Bullet.UnitTests
{
	public class Test_001
	{
		int step;
		bool sloped;
		bool timing_only;
		bool large_count;
        btDiscreteDynamicsWorld world;
		btRigidBody fallingRigidBody;

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


			btCollisionShape fallShape = new btBoxShape( ref btVector3.One );
			if( large_count )
			{
				int n;
				int cubes = 5;
				for( n = 0; n < cubes * cubes * cubes; n++ )
				{
					btVector3 origin = new btVector3( ( n %cubes - cubes/2) * 3, 50 + ( n/(cubes*cubes))*3, (( n / cubes)%cubes - cubes/2 ) * 3 );
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
					if( n % (cubes* cubes ) == 0 )
						Console.Write( "." );
				}
			}
			else
			{
				btVector3 origin = new btVector3( 1, 50, 1 );
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
		}

		internal void Tick()
		{
			{
				world.stepSimulation( 1 / 60.0f, 10 );
				if( Program.Display != null )
					world.debugDrawWorld();
				if( !timing_only )
				{
					btTransform trans;

					fallingRigidBody.getMotionState().getWorldTransform( out trans );
					Console.WriteLine( "Iteration {0}", step );
					Console.WriteLine( trans.ToString( "cube orient\t", "\t\t", "cube origin\t" ) );
					btVector3 v = fallingRigidBody.getAngularVelocity();
					Console.WriteLine( "cube Ang Vel : {0}", v );
					v = fallingRigidBody.getLinearVelocity();
					Console.WriteLine( "cube Lin Vel : {0}", v );
				}
			}
			//  world = new DiscreteDynamicsWorld( null, null, null, null );
			step++;
		}

		static public void Run( bool timing_only, Stopwatch sw, bool large_count, bool sloped )
		{
			Test_001 test = new Test_001();
			test.sloped = sloped;
			test.large_count = large_count;
			test.timing_only = timing_only;
			test.Setup();
			if( sw != null )
			{
				sw.Reset();
				sw.Start();
			}
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
