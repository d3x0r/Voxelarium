//#define DISABLE_NON_REF_CONSTRUCTORS
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
*/

namespace Bullet.LinearMath
{

	/*
#if BT_USE_DOUBLE_PRECISION
#define btTransformData btTransformDoubleData
#else
#define btTransformData btTransformFloatData
#endif
	*/

		/* btITransform can be used to maintain a reference to another transform.  This will 
		avoid accidental copies. */
	public interface btITransform
	{
		/*@brief Return reference to the basis matrix for the rotation */
		btIMatrix3x3 Basis {get;}

		btTransform T { get; set; }
		void Get( out btTransform tmp );

		/*@brief Constructor from btQuaternion (optional btVector3 )
		  @param q Rotation from quaternion 
		  @param c Translation from Vector (default 0,0,0) */
		void setValue( ref btQuaternion q, ref btVector3 c );

		/*@brief Constructor from btMatrix3x3 (optional btVector3)
		  @param b Rotation from Matrix 
		  @param c Translation from Vector default (0,0,0)*/
		void setValue( ref btMatrix3x3 b, ref btVector3 c );

		/*@brief Set the current transform as the value of the product of two transforms
		  @param t1 Transform 1
		  @param t2 Transform 2
		  This = Transform1  Transform2 */
		void mult( ref btTransform t1, ref btTransform t2 );
		/*		void multInverseLeft(btTransform t1, btTransform t2) {
					btVector3 v = t2.m_origin - t1.m_origin;
					m_basis = btMultTransposeLeft(t1.m_basis, t2.m_basis);
					m_origin = v  t1.m_basis;
				} */

		/*@brief Return the transform of the vector */
		void Apply( ref btVector3 x, out btVector3 result );
		void Apply( ref btTransform t2, out btTransform result );
		void Apply( btITransform t2, out btTransform result );

		/*@brief Return the basis matrix for the rotation */
		btIMatrix3x3 getBasis();
		void getBasis( out btMatrix3x3 m );

		/*@brief Return the origin vector translation */
		void getOrigin( out btVector3 result );
		btIVector3 getOrigin();

		/*@brief Return a quaternion representing the rotation */
		void getRotation( out btQuaternion result );


		/*@brief Set the translational element
		  @param origin The vector to set the translation to */
		void setOrigin( ref btVector3 origin );

		/*@brief Set the rotational element by btMatrix3x3 */
		void setBasis( ref btMatrix3x3 basis );

		/*@brief Set the rotational element by btQuaternion */
		void setRotation( ref btQuaternion q );


		/*@brief Set this transformation to the identity */
		void setIdentity();


		/*@brief Return the inverse of this transform */
		void inverse( out btTransform result );


		void invXform( ref btVector3 inVec, out btVector3 result );

		/*@brief Return the inverse of this transform times the other transform
		  @param t The other transform 
		  return this.inverse()  the other */
		void inverseTimes( ref btTransform t, out btTransform result );
		void inverseTimes( btITransform t, out btTransform result );

		/*@brief Test if two transforms have all elements equal */
		bool Equals( ref btTransform t2 );
	}

	/*@brief The btTransform class supports rigid transforms with only translation and rotation and no scaling/shear.
	 It can be used in combination with btVector3, btQuaternion and btMatrix3x3 linear algebra classes. */
	public struct btTransform : btITransform
	{
		public static btTransform Identity = new btTransform( ref btMatrix3x3.Identity );
		public btTransform T { get { return this; } set { this = value; } }
		public void Get( out btTransform tmp ) { tmp = this; }

		///Storage for the rotation
		public btMatrix3x3 m_basis;
		///Storage for the translation
		public btVector3 m_origin;

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
#if !DISABLE_NON_REF_CONSTRUCTORS
		public btTransform( btQuaternion q, btVector3 c ) : this( ref q, ref c )
		{
		}
#endif
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
		public void Apply( btITransform t2, out btTransform result )
		{
			m_basis.Apply( t2.getBasis(), out result.m_basis );
			Apply( t2.getOrigin(), out result.m_origin );
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
		public void Apply( btIVector3 x, out btVector3 result )
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

#if !DISABLE_OPERATOR
		public static btVector3 operator *( btTransform t, btIVector3 x )
		{
			btVector3 result;
			t.Apply( x, out result );
			return result;
		}
#endif

		/*@brief Return the basis matrix for the rotation */
		public btIMatrix3x3 getBasis() { return m_basis; }
		public btIMatrix3x3 Basis { get { return m_basis; } }
		public void getBasis( out btMatrix3x3 m ) { m = m_basis; }

#if !DISABLE_OPERATORS
		public btIVector3 getOrigin() { return m_origin; }
#endif


		/*@brief Return the origin vector translation */
		public void getOrigin(out btVector3 result ) { result = m_origin; }

		/*@brief Return a quaternion representing the rotation */
		public void getRotation( out btQuaternion result )
		{
			m_basis.getRotation( out result );
		}

#if asdfasdf
		/*@brief Set from an array 
		  @param m A pointer to a 15 element array (12 rotation(row major padded on the right by 1), and 3 translation */
		void setFromOpenGLMatrix( double m )
		{
			m_basis.setFromOpenGLSubMatrix( m );
			m_origin.setValue( m[12], m[13], m[14] );
		}

		/*@brief Fill an array representation
		  @param m A pointer to a 15 element array (12 rotation(row major padded on the right by 1), and 3 translation */
		void getOpenGLMatrix( double m )
		{
			m_basis.getOpenGLSubMatrix( m );
			m[12] = m_origin.x;
			m[13] = m_origin.y;
			m[14] = m_origin.z;
			m[15] = 1.0;
		}
#endif

		/*@brief Set the translational element
		  @param origin The vector to set the translation to */
		public void setOrigin( ref btVector3 origin )
		{
			m_origin = origin;
		}

		public void setOrigin( btIVector3 origin )
		{
			origin.Copy( out m_origin );
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
			btVector3 tmp;
			m_origin.Invert( out tmp );
			result.m_basis.Apply( ref tmp, out result.m_origin );
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
		public void inverseTimes( btITransform t, out btTransform result )
		{
			btVector3 v;
			t.getOrigin().Sub( ref m_origin, out v );
			btMatrix3x3 m;
			t.getBasis( out m );
			m_basis.transposeTimes( ref m, out result.m_basis );
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
		public override string ToString()
		{
			return string.Format( "Orientation: {0}\n Origin: {1}", m_basis, m_origin );
		}

	}




}