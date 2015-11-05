using System;
using System.Collections.Generic;
using System.Text;

namespace Bullet.LinearMath
{
	public interface btMotionState
	{
		void getWorldTransform( out btTransform worldTrans );
		//Bullet only calls the update of worldtransform for active objects
		void setWorldTransform( ref btTransform worldTrans);
	}
}
