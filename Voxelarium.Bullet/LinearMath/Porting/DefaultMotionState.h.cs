
namespace Bullet.LinearMath
{

	///The btDefaultMotionState provides a common implementation to synchronize world transforms with offsets.
	public class btDefaultMotionState : btMotionState
	{
		btTransform m_graphicsWorldTrans;
		btTransform m_centerOfMassOffset;
		btTransform m_startWorldTrans;
		object m_userPointer;

		public btDefaultMotionState()
		{
			m_graphicsWorldTrans = btTransform.Identity;
			m_centerOfMassOffset = btTransform.Identity;
			m_startWorldTrans = btTransform.Identity;
			m_userPointer = ( 0 );
		}

		public btDefaultMotionState( ref btTransform startTrans )
		{
			m_graphicsWorldTrans = ( startTrans );
			m_centerOfMassOffset = btTransform.Identity;
			m_startWorldTrans = ( startTrans );
			m_userPointer = ( 0 );
		}

		public btDefaultMotionState( ref btTransform startTrans, ref btTransform centerOfMassOffset )
		{
			m_graphicsWorldTrans = ( startTrans );
			m_centerOfMassOffset = ( centerOfMassOffset );
			m_startWorldTrans = ( startTrans );
			m_userPointer = ( 0 );
		}

		///synchronizes world transform from user to physics
		public virtual void getWorldTransform( out btTransform centerOfMassWorldTrans )
		{
			btTransform tmp;
			m_centerOfMassOffset.inverse( out tmp );
			m_graphicsWorldTrans.Apply( ref tmp, out centerOfMassWorldTrans );
		}

		///synchronizes world transform from physics to user
		///Bullet only calls the update of worldtransform for active objects
		public virtual void setWorldTransform( ref btTransform centerOfMassWorldTrans )
		{
			centerOfMassWorldTrans.Apply( ref m_centerOfMassOffset, out m_graphicsWorldTrans );
		}
	};
}