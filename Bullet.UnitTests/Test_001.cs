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
	public class Test_001
	{

		static public void Run( bool sloped )
		{
			btDiscreteDynamicsWorld world;
			world = new btDiscreteDynamicsWorld();
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

			btVector3 origin = new btVector3( 1, 50, 1 );
			btTransform init = new btTransform( ref btQuaternion.Identity, ref origin );
			btDefaultMotionState fallMotionState = new btDefaultMotionState( ref init );

			btScalar mass = 1;
			btVector3 fallInertia;
			fallShape.calculateLocalInertia( mass, out fallInertia );

			btRigidBody.btRigidBodyConstructionInfo
				fallingRigidBodyCI = new btRigidBody.btRigidBodyConstructionInfo( mass, fallMotionState
							, fallShape, ref fallInertia );

			btRigidBody fallingRigidBody = new btRigidBody( fallingRigidBodyCI );

			world.addRigidBody( fallingRigidBody );

			for( int i = 0; i < 300; i++ )
			{
				if( i == 195 )
				{
					int a = 3;
				}
				world.stepSimulation( 1 / 60.0f, 10 );

				btTransform trans;
				fallingRigidBody.getMotionState().getWorldTransform( out trans );
				Console.WriteLine( "Iteration {0}", i );
				Console.WriteLine( "cube height: {0}", trans.getOrigin() );
				Console.WriteLine( "cube orient:\t{0}", trans.m_basis.ToString( "\t\t" ) );
				btIVector3 v = fallingRigidBody.getAngularVelocity();
				Console.WriteLine( "cube Ang Vel : {0}", v );
				v = fallingRigidBody.getLinearVelocity();
				Console.WriteLine( "cube Lin Vel : {0}", v );
			}
			//  world = new DiscreteDynamicsWorld( null, null, null, null );
		}

	}
}
