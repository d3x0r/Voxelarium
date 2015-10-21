using System;
using System.Collections.Generic;
using System.Text;

namespace Bullet.Types
{
	public enum btObjectTypes
	{
		/* btTypedConstraintType */
		POINT2POINT_CONSTRAINT_TYPE = 3,
		HINGE_CONSTRAINT_TYPE,
		CONETWIST_CONSTRAINT_TYPE,
		D6_CONSTRAINT_TYPE,
		SLIDER_CONSTRAINT_TYPE,
		CONTACT_CONSTRAINT_TYPE,
		D6_SPRING_CONSTRAINT_TYPE,
		GEAR_CONSTRAINT_TYPE,
		FIXED_CONSTRAINT_TYPE,
		D6_SPRING_2_CONSTRAINT_TYPE,
		MAX_CONSTRAINT_TYPE,

		/* btManifoldTYpe */
		MIN_CONTACT_MANIFOLD_TYPE = 1024,
		BT_PERSISTENT_MANIFOLD_TYPE
	};

	///rudimentary class to provide type info
	public class btTypedObject
	{
		internal btTypedObject( btObjectTypes objectType )
		{
			m_objectType = objectType;
		}
		public btObjectTypes m_objectType;
		public btObjectTypes getObjectType()
		{
			return m_objectType;
		}
	};

}
