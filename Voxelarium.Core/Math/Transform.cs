//#define COLUMN_MAJOR_EXPECTED
//#define DISABLE_OPERATORS
/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/
 
 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose, 
 including commercial applications, and to alter it and redistribute it freely, 
 subject to the following restrictions:
 
 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 
 This source version has been altered significantly from C++ to C#

 Now part of Voxelarium.
 */

namespace Voxelarium.LinearMath
{


	/*@brief The btTransform class supports rigid transforms with only translation and rotation and no scaling/shear.
	 It can be used in combination with btVector3, btQuaternion and btMatrix3x3 linear algebra classes. */
	public struct btTransform 
	{
		public static btTransform Identity = new btTransform( ref btMatrix3x3.Identity );
		public btTransform T { get { return this; } set { this = value; } }

		///Storage for the rotation
		internal btMatrix3x3 m_basis;
		///Storage for the translation
		internal btVector3 m_origin;

		/*@brief Constructor from btQuaternion (optional btVector3 )
		  @param q Rotation from quaternion 
		  @param c Translation from Vector (default 0,0,0) */
		public btTransform( ref btQuaternion q )
		{
			m_basis = new btMatrix3x3( ref q );
			m_origin = btVector3.Zero;
		}
		public btTransform( ref btVector3 q )
		{
			m_basis = btMatrix3x3.Identity;
			m_origin = q;
		}

		/*@brief Constructor from btMatrix3x3 (optional btVector3)
		  @param b Rotation from Matrix 
		  @param c Translation from Vector default (0,0,0)*/
		public btTransform( ref btMatrix3x3 b )
		{
			m_basis = b;
			m_origin = btVector3.Zero;
		}

		/*@brief Constructor from btQuaternion (optional btVector3 )
		  @param q Rotation from quaternion 
		  @param c Translation from Vector (default 0,0,0) */
		public btTransform( ref btQuaternion q, ref btVector3 c )
		{
			m_basis = new btMatrix3x3( ref q );
			m_origin = c;
		}
		/*@brief Constructor from btQuaternion (optional btVector3 )
		  @param q Rotation from quaternion 
		  @param c Translation from Vector (default 0,0,0) */
		public void setValue( ref btQuaternion q, ref btVector3 c )
		{
			m_basis = new btMatrix3x3( ref q );
			m_origin = c;
		}

		/*@brief Constructor from btMatrix3x3 (optional btVector3)
		  @param b Rotation from Matrix 
		  @param c Translation from Vector default (0,0,0)*/
		public btTransform( ref btMatrix3x3 b, ref btVector3 c )
		{
			m_basis = b;
			m_origin = c;
		}
		/*@brief Constructor from btMatrix3x3 (optional btVector3)
		  @param b Rotation from Matrix 
		  @param c Translation from Vector default (0,0,0)*/
		public void setValue( ref btMatrix3x3 b, ref btVector3 c )
		{
			m_basis = b;
			m_origin = c;
		}
		/*@brief Copy constructor */
		public btTransform( ref btTransform other )
		{
			m_basis = other.m_basis;
			m_origin = other.m_origin;
		}

		public float x { get { return m_origin.x; }  set { m_origin.x = value; } }
		public float y { get { return m_origin.y; } set { m_origin.y = value; } }
		public float z { get { return m_origin.z; } set { m_origin.z = value; } }

		public void GetForward( out btVector3 v ) { v.x = m_basis.m_el2.x; v.y = m_basis.m_el2.y; v.z = m_basis.m_el2.z; v.w = btScalar.BT_ZERO; }
		public void GetRight( out btVector3 v ) { v.x = m_basis.m_el0.x; v.y = m_basis.m_el0.y; v.z = m_basis.m_el0.z; v.w = btScalar.BT_ZERO; }
		public void GetUp( out btVector3 v ) { v.x = m_basis.m_el1.x; v.y = m_basis.m_el1.y; v.z = m_basis.m_el1.z; v.w = btScalar.BT_ZERO; }

		/*@brief Set the current transform as the value of the product of two transforms
		  @param t1 Transform 1
		  @param t2 Transform 2
		  This = Transform1  Transform2 */
		public void mult( ref btTransform t1, ref btTransform t2 )
		{
			t1.m_basis.Apply( ref t2.m_basis, out m_basis );
			t1.Apply( ref t2.m_origin, out m_origin );
		}
		public void Apply( ref btTransform t2, out btTransform result )
		{
			m_basis.Apply( ref t2.m_basis, out result.m_basis );
			Apply( ref t2.m_origin, out result.m_origin );
		}

		/*		void multInverseLeft(btTransform t1, btTransform t2) {
					btVector3 v = t2.m_origin - t1.m_origin;
					m_basis = btMultTransposeLeft(t1.m_basis, t2.m_basis);
					m_origin = v  t1.m_basis;
				}
				*/

		/*@brief Return the transform of the vector */
		public void Apply( ref btVector3 x, out btVector3 result )
		{
			btVector3 tmp;
			x.dot3( ref m_basis, out tmp );
			tmp.Add( ref m_origin, out result );
		}

		/*@brief Return the transform of the vector */
		public static void Apply( ref btTransform t, ref btVector3 x, out btVector3 result )
		{
			t.Apply( ref x, out result );
		}

		/*@brief Return the transform of the btQuaternion */
		public static void Apply( ref btTransform t, ref btQuaternion q, out btQuaternion result )
		{
			btQuaternion tmp;
			t.getRotation( out tmp );
			tmp.Mult( ref q, out result );
		}


		/*@brief Return the basis matrix for the rotation */

		/*@brief Return the origin vector translation */
		public void getOrigin(out btVector3 result ) { result = m_origin; }

		/*@brief Return a quaternion representing the rotation */
		public void getRotation( out btQuaternion result )
		{
			m_basis.getRotation( out result );
		}

		/*@brief Set the translational element
		  @param origin The vector to set the translation to */
		public void setOrigin( ref btVector3 origin )
		{
			m_origin = origin;
		}

		/*@brief Set the rotational element by btMatrix3x3 */
		public void setBasis( ref btMatrix3x3 basis )
		{
			m_basis = basis;
		}

		/*@brief Set the rotational element by btQuaternion */
		public void setRotation( ref btQuaternion q )
		{
			m_basis.setRotation( ref q );
		}


		/*@brief Set this transformation to the identity */
		public void setIdentity()
		{
			m_basis.setIdentity();
			m_origin = btVector3.Zero;
		}


		/*@brief Return the inverse of this transform */
		public void inverse( out btTransform result )
		{
			m_basis.transpose( out result.m_basis );
			m_origin.Invert( out result.m_origin );
		}


		public void invXform( ref btVector3 inVec, out btVector3 result )
		{
			btVector3 v;
			inVec.Sub( ref m_origin, out v );
			btMatrix3x3 tmp;
			m_basis.transpose( out tmp );
			tmp.Apply( ref v, out result );
		}

		/*@brief Return the inverse of this transform times the other transform
		  @param t The other transform 
		  return this.inverse()  the other */
		public void inverseTimes( ref btTransform t, out btTransform result )
		{
			btVector3 v;
			t.m_origin.Sub( ref m_origin, out v );
			m_basis.transposeTimes( ref t.m_basis, out result.m_basis );
			m_basis.ApplyInverse( ref v, out result.m_origin );
		}


		/*@brief Test if two transforms have all elements equal */
		public static bool Equals( ref btTransform t1, ref btTransform t2 )
		{
			return ( t1.m_basis.Equals( ref t2.m_basis ) &&
					 t1.m_origin.Equals( ref t2.m_origin ) );
		}
		public bool Equals( ref btTransform t2 )
		{
			return Equals( this, t2 );
		}

		public void GetGLMatrix( out btMatrix3x3 m )
		{
#if !COLUMN_MAJOR_EXPECTED
			m = m_basis;
			m.m_el3 = m_origin;
			m.m_el3.w = 1;
#else
			m.m_el0.x = m_basis.m_el0.x;
			m.m_el0.y = m_basis.m_el1.x;
			m.m_el0.z = m_basis.m_el2.x;
			m.m_el0.w = m_basis.m_el0.w;

			m.m_el1.x = m_basis.m_el0.y;
			m.m_el1.y = m_basis.m_el1.y;
			m.m_el1.z = m_basis.m_el2.y;
			m.m_el1.w = m_basis.m_el1.w;

			m.m_el2.x = m_basis.m_el0.z;
			m.m_el2.y = m_basis.m_el1.z;
			m.m_el2.z = m_basis.m_el2.z;
			m.m_el2.w = m_basis.m_el2.w;
			m.m_el3 = m_origin;
			m.m_el3.w = m_basis[3][3];
#endif
		}

		public void GetGLCameraMatrix( out btMatrix3x3 m )
		{
#if !COLUMN_MAJOR_EXPECTED
			m.m_el0.x = m_basis.m_el0.x;
			m.m_el0.y = m_basis.m_el1.x;
			m.m_el0.z = -m_basis.m_el2.x;
			m.m_el0.w = 0;

			m.m_el1.x = m_basis.m_el0.y;
			m.m_el1.y = m_basis.m_el1.y;
			m.m_el1.z = -m_basis.m_el2.y;
			m.m_el1.w = 0;// m_basis.m_el1.w;

			m.m_el2.x = m_basis.m_el0.z;
			m.m_el2.y = m_basis.m_el1.z;
			m.m_el2.z = -m_basis.m_el2.z;
			m.m_el2.w = 0;// m_basis.m_el2.w;
			//m.m_el3 = 
			btVector3 negOrigin;
			m_origin.Invert( out negOrigin );
			m_basis.m_el2.Invert( out m_basis.m_el2 );
			m_basis.ApplyInverseRotation( ref negOrigin, out m.m_el3 );
			m_basis.m_el2.Invert( out m_basis.m_el2 );
			m.m_el3.w = 1;
#else
			m = m_basis;
			btVector3 tmp;
			btVector3 negOrigin;
			m_origin.Invert( out negOrigin );
			m.m_el0.z = -m.m_el0.z;
			m.m_el1.z = -m.m_el1.z;
			m.m_el2.z = -m.m_el2.z;
			m_basis.m_el0.z = -m_basis.m_el0.z;
			m_basis.m_el1.z = -m_basis.m_el1.z;
			m_basis.m_el2.z = -m_basis.m_el2.z;
			m_basis.ApplyInverseRotation( ref negOrigin, out tmp );
			m_basis.m_el0.z = -m_basis.m_el0.z;
			m_basis.m_el1.z = -m_basis.m_el1.z;
			m_basis.m_el2.z = -m_basis.m_el2.z;
			m.m_el3.x = tmp.x;
			m.m_el3.y = tmp.y;
			m.m_el3.z = tmp.z;
			m.m_el3.w = m_basis[3][3];
#endif
		}


		public void Translate( ref btVector3 v )
		{
			m_origin = v;
		}
		public void Translate( float x, float y, float z )
		{
			m_origin.x = x;
			m_origin.y = y;
			m_origin.z = z;
		}
		public void Move( float x, float y, float z )
		{
			m_origin.x += x;
			m_origin.y += y;
			m_origin.z += z;
		}


		/*
		///for serialization
		struct btTransformFloatData
		{
			btMatrix3x3FloatData m_basis;
			btVector3FloatData m_origin;
		};

		struct btTransformDoubleData
		{
			btMatrix3x3DoubleData m_basis;
			btVector3DoubleData m_origin;
		};

		public void btTransform::serialize( btTransformData& dataOut)
		{
			m_basis.serialize( dataOut.m_basis );
			m_origin.serialize( dataOut.m_origin );
		}

		public void btTransform::serializeFloat( btTransformFloatData& dataOut)
		{
			m_basis.serializeFloat( dataOut.m_basis );
			m_origin.serializeFloat( dataOut.m_origin );
		}


		public void btTransform::deSerialize(btTransformData& dataIn)
		{
			m_basis.deSerialize( dataIn.m_basis );
			m_origin.deSerialize( dataIn.m_origin );
		}

		public void btTransform::deSerializeFloat(btTransformFloatData& dataIn)
		{
			m_basis.deSerializeFloat( dataIn.m_basis );
			m_origin.deSerializeFloat( dataIn.m_origin );
		}

		public void btTransform::deSerializeDouble(btTransformDoubleData& dataIn)
		{
			m_basis.deSerializeDouble( dataIn.m_basis );
			m_origin.deSerializeDouble( dataIn.m_origin );
		}


		*/

	}




}