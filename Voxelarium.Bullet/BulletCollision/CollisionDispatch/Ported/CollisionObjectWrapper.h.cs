
using Bullet.Collision.Shapes;
using Bullet.LinearMath;
using System;

namespace Bullet.Collision.Dispatch
{

	///btCollisionObjectWrapperis an internal data structure. 
	///Most users can ignore this and use btCollisionObject and btCollisionShape instead


	public class btCollisionObjectWrapper : IDisposable
	{

		//public btCollisionObjectWrapper operator=(btCollisionObjectWrapper);

		public btCollisionObjectWrapper m_parent;
		public btCollisionShape m_shape;
		public btCollisionObject m_collisionObject;
		public int m_partId;
		public int m_index;

		public void Dispose()
		{
			BulletGlobals.CollisionObjectWrapperPool.Free( this );
		}

		public void Initialize( btCollisionObjectWrapper parent, btCollisionShape shape, btCollisionObject collisionObject
			//, ref btTransform worldTransform
			, int partId, int index )
		{
			m_parent = ( parent );
			m_shape = ( shape );
			m_collisionObject = ( collisionObject );
			//m_worldTransform = ( worldTransform );
			m_partId = ( partId );
			m_index = ( index );
		}

		//public btTransform getWorldTransform() {  return m_worldTransform; }
		//public void getWorldTransform( out btTransform result ) { result = m_worldTransform; }
		//public btCollisionObject getCollisionObject() { return m_collisionObject; }
		public btCollisionShape getCollisionShape() { return m_shape; }
	};

}
