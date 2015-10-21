
using Bullet.Collision.Shapes;
using Bullet.LinearMath;

namespace Bullet.Collision.Dispatch
{

	///btCollisionObjectWrapperis an internal data structure. 
	///Most users can ignore this and use btCollisionObject and btCollisionShape instead


	public class btCollisionObjectWrapper
	{

		//public btCollisionObjectWrapper operator=(btCollisionObjectWrapper);

		public btCollisionObjectWrapper m_parent;
		public btCollisionShape m_shape;
		public btCollisionObject m_collisionObject;
		public btITransform m_worldTransform;
		public int m_partId;
		public int m_index;

		public btCollisionObjectWrapper( btCollisionObjectWrapper parent, btCollisionShape shape, btCollisionObject collisionObject, btITransform worldTransform, int partId, int index )
		{
			m_parent = ( parent );
			m_shape = ( shape );
			m_collisionObject = ( collisionObject );
			m_worldTransform = ( worldTransform );
			m_partId = ( partId );
			m_index = ( index );
		}

		public btITransform getWorldTransform() {  return m_worldTransform; }
		public btCollisionObject getCollisionObject() { return m_collisionObject; }
		public btCollisionShape getCollisionShape() { return m_shape; }
	};

}
