//#define DISABLE_OPERATORS
/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans * http://continuousphysics.com/Bullet/

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
	public interface btIMatrix3x3
	{
		btIVector3 this[int n] { get; }
		/* @brief Get a column of the matrix as a vector 
		 * @param i Column number 0 indexed */
		btVector3 getColumn( int i );
		void getColumn( int i, out btVector3 result );
		/* @brief Get a row of the matrix as a vector 
		 * @param i Row number 0 indexed */
		btIVector3 getRow( int i );
		void getRow( int i, out btVector3 result );

		/* @brief Set the matrix from a quaternion
		 * @param q The Quaternion to match */
		void setRotation( ref btQuaternion q );

		void setValue( double xx, double xy, double xz,
			double yx, double yy, double yz,
			double zx, double zy, double zz );

		/* @brief Set the matrix from euler angles using YPR around YXZ respectively
		 * @param yaw Yaw about Y axis
		 * @param pitch Pitch about X axis
		 * @param roll Roll about Z axis 
*/
		void setEulerYPR( double yaw, double pitch, double roll );

		/* @brief Set the matrix from euler angles YPR around ZYX axes
		 @param eulerX Roll about X axis
		 @param eulerY Pitch around Y axis
		 @param eulerZ Yaw aboud Z axis
		 
		 These angles are used to produce a rotation matrix. The euler
		 angles are applied in ZYX order. I.e a vector is first rotated 
		 about X then Y and then Z
		*/
		void setEulerZYX( double eulerX, double eulerY, double eulerZ );

		/*@brief Get the matrix represented as a quaternion 
		 @param q The quaternion which will be set */
		void getRotation( out btQuaternion result );
		/*@brief Get the matrix represented as euler angles around YXZ, roundtrip with setEulerYPR
		 @param yaw Yaw around Y axis
		 @param pitch Pitch around X axis
		 @param roll around Z axis */
		void getEulerYPR( out double yaw, out double pitch, out double roll );

		/*@brief Get the matrix represented as euler angles around ZYX
		 @param yaw Yaw around X axis
		 @param pitch Pitch around Y axis
		 @param roll around X axis 
		 @param solution_number Which solution of two possible solutions ( 1 or 2) are possible values*/
		void getEulerZYX( out double yaw, out double pitch, out double roll, int solution_number = 1 );

		/*@brief Create a scaled copy of the matrix 
		 @param s Scaling vector The elements of the vector will scale each column */

		//btMatrix3x3 scaled( ref btVector3 s );
		void scaled( ref btVector3 s, out btMatrix3x3 result );

		/// Solve A * x = b, where b is a column vector. This is more efficient
		/// than computing the inverse in one-shot cases.
		///Solve33 is from Box2d, thanks to Erin Catto,
		void solve33( ref btVector3 b, out btVector3 result );

		void Mult( double k, out btMatrix3x3 result );

		void Mult( ref btVector3 v, out btVector3 result );

		void Mult( ref btMatrix3x3 m2, out btMatrix3x3 result );

		void Add( ref btMatrix3x3 m2, out btMatrix3x3 result );

		void Sub( ref btMatrix3x3 m2, out btMatrix3x3 result );

		double determinant();
		void absolute( out btMatrix3x3 m );

		// btMatrix3x3 absolute();
		// btMatrix3x3 transpose()
		void transpose( out btMatrix3x3 result );
		btMatrix3x3 adjoint();
		void adjoint( out btMatrix3x3 result );

		//btMatrix3x3 inverse();

		void inverse( out btMatrix3x3 result );

		void transposeTimes( ref btMatrix3x3 m, out btMatrix3x3 result );

		void timesTranspose( ref btMatrix3x3 m, out btMatrix3x3 result );

		void Apply( ref btVector3 v, out btVector3 result );
		void ApplyInverse( ref btVector3 v, out btVector3 result );

		/*
				public static btVector3 operator *( ref btVector3 v, ref btMatrix3x3 m )
				{
					return new btVector3( m.tdotx( ref v ), m.tdoty( ref v ), m.tdotz( ref v ) );
				}
				*/

		void Apply( ref btMatrix3x3 m2, out btMatrix3x3 result );

		double tdotx( ref btVector3 v );
        double tdoty( ref btVector3 v );
        double tdotz( ref btVector3 v );
		/*
		public btMatrix3x3 btMultTransposeLeft(btMatrix3x3 m1, btMatrix3x3 m2) {
		return btMatrix3x3(
		m1[0,0] * m2[0,0] + m1[1,0] * m2[1,0] + m1[2,0] * m2[2,0],
		m1[0,0] * m2[0,1] + m1[1,0] * m2[1,1] + m1[2,0] * m2[2,1],
		m1[0,0] * m2[0,2] + m1[1,0] * m2[1,2] + m1[2,0] * m2[2,2],
		m1[0,1] * m2[0,0] + m1[1,1] * m2[1,0] + m1[2,1] * m2[2,0],
		m1[0,1] * m2[0,1] + m1[1,1] * m2[1,1] + m1[2,1] * m2[2,1],
		m1[0,1] * m2[0,2] + m1[1,1] * m2[1,2] + m1[2,1] * m2[2,2],
		m1[0,2] * m2[0,0] + m1[1,2] * m2[1,0] + m1[2,2] * m2[2,0],
		m1[0,2] * m2[0,1] + m1[1,2] * m2[1,1] + m1[2,2] * m2[2,1],
		m1[0,2] * m2[0,2] + m1[1,2] * m2[1,2] + m1[2,2] * m2[2,2]);
		}
		*/

		/*@brief Equality operator between two matrices
		 It will test all elements are equal. * */
		bool Equals( ref btMatrix3x3 m1 );

	}

	public struct btMatrix3x3 : btIMatrix3x3
	{
		/*
		#if BT_USE_DOUBLE_PRECISION
		#define btMatrix3x3Data	btMatrix3x3DoubleData 
		#else
		#define btMatrix3x3Data	btMatrix3x3FloatData
		#endif //BT_USE_DOUBLE_PRECISION
		*/

		/*@brief The btMatrix3x3 class implements a 3x3 rotation matrix, to perform linear algebra in combination with btQuaternion, btTransform and btVector3.
		 Make sure to only include a pure orthogonal matrix without scaling. */

		///Data storage for the matrix, each vector is a row of the matrix
		public btVector3 m_el0;
		public btVector3 m_el1;
		public btVector3 m_el2;
		public btVector3 m_el3;


		//		explicit btMatrix3x3(stringbtScalar m) { setFromOpenGLSubMatrix(m); }

		/*@brief Constructor from Quaternion */
		public btMatrix3x3( ref btQuaternion q )
		{
			//setRotation( q );
			double d = q.length2();
			//btFullAssert(d != (double)(0.0));
			double s = 2.0 / d;

			double xs = q.x * s, ys = q.y * s, zs = q.z * s;
			double wx = q.w * xs, wy = q.w * ys, wz = q.w * zs;
			double xx = q.x * xs, xy = q.x * ys, xz = q.x * zs;
			double yy = q.y * ys, yz = q.y * zs, zz = q.z * zs;
			m_el0.x = 1.0 - ( yy + zz ); m_el0.y = xy - wz; m_el0.z = xz + wy; m_el0.w = 0;
			m_el1.x = xy + wz; m_el1.y = 1.0 - ( xx + zz ); m_el1.z = yz - wx; m_el1.w = 0;
			m_el2.x = xz - wy; m_el2.y = yz + wx; m_el2.z = 1.0 - ( xx + yy ); m_el2.w = 0;
			m_el3.x = 0; m_el3.y = 0; m_el3.z = 0; m_el3.w = 1;

		}
		/*
		template <typename double>
		Matrix3x3(double yaw, double pitch, double roll)
		{ 
		setEulerYPR(yaw, pitch, roll);
		}
		*/
		/* @brief Constructor with row major formatting */
		public btMatrix3x3( double xx, double xy, double xz,
			double yx, double yy, double yz,
			double zx, double zy, double zz )
		{
			m_el0.x = xx; m_el1.x = xy; m_el2.x = xz; m_el3.x = 0;
			m_el0.y = yx; m_el1.y = yy; m_el2.y = yz; m_el3.y = 0;
			m_el0.z = zx; m_el1.z = zy; m_el2.z = zz; m_el3.z = 0;
			m_el0.w = 0; m_el1.w = 0; m_el2.w = 0; m_el3.w = 1;
		}

		public btMatrix3x3( ref btVector3 v0, ref btVector3 v1, ref btVector3 v2 )
		{
			m_el0.x = v0.x; m_el0.y = v0.y; m_el0.z = v0.z; m_el0.w = 0;
			m_el1.x = v1.x; m_el1.y = v1.y; m_el1.z = v1.z; m_el1.w = 0;
			m_el2.x = v2.x; m_el2.y = v2.y; m_el2.z = v2.z; m_el2.w = 0;
			m_el3.x = 0; m_el3.y = 0; m_el3.z = 0; m_el3.w = 1;
		}

		// Copy constructor
		public btMatrix3x3( ref btMatrix3x3 rhs )
		{
			m_el0.x = rhs.m_el0.x; m_el0.y = rhs.m_el0.y; m_el0.z = rhs.m_el0.z; m_el0.w = rhs.m_el0.w;
			m_el1.x = rhs.m_el1.x; m_el1.y = rhs.m_el1.y; m_el1.z = rhs.m_el1.z; m_el1.w = rhs.m_el1.w;
			m_el2.x = rhs.m_el2.x; m_el2.y = rhs.m_el2.y; m_el2.z = rhs.m_el2.z; m_el2.w = rhs.m_el2.w;
			m_el3.x = rhs.m_el3.x; m_el3.y = rhs.m_el3.y; m_el3.z = rhs.m_el3.z; m_el3.w = rhs.m_el3.w;
		}



		/* @brief Get a column of the matrix as a vector 
		 * @param i Column number 0 indexed */
		public btVector3 getColumn( int i )
		{
			switch( i )
			{
				default:
#if PARANOID_ASSERTS
					Debug.Assert( i > 3 || i < 0 );
#endif
				case 0: return new btVector3( m_el0.x, m_el1.x, m_el2.x );
				case 1: return new btVector3( m_el0.y, m_el1.y, m_el2.y );
				case 2: return new btVector3( m_el0.z, m_el1.z, m_el2.z );
				case 3: return new btVector3( m_el0.w, m_el1.w, m_el2.w );
			}
		}
		public void getColumn( int i, out btVector3 result )
		{
			switch( i )
			{
				default:
#if PARANOID_ASSERTS
					Debug.Assert( i > 3 || i < 0 );
#endif
				case 0:
					result = new btVector3( m_el0.x, m_el1.x, m_el2.x );
					break;
				case 1:
					result = new btVector3( m_el0.y, m_el1.y, m_el2.y );
					break;
				case 2:
					result = new btVector3( m_el0.z, m_el1.z, m_el2.z );
					break;
				case 3:
					result = new btVector3( m_el0.w, m_el1.w, m_el2.w );
					break;
			}
			return;
		}

		/* @brief Get a row of the matrix as a vector 
		 * @param i Row number 0 indexed */
		public btIVector3 getRow( int i )
		{
			switch( i )
			{
				default:
#if PARANOID_ASSERTS
					Debug.Assert( i > 3 || i < 0 );
#endif
				case 0: return m_el0;
				case 1: return m_el1;
				case 2: return m_el2;
				case 3: return m_el3;
			}
		}

		public void getRow( int i, out btVector3 result )
		{
			switch( i )
			{
				default:
#if PARANOID_ASSERTS
					Debug.Assert( i > 3 || i < 0 );
#endif
				case 0: result = m_el0; return;
				case 1: result = m_el1; return;
				case 2: result = m_el2; return;
				case 3: result = m_el3; return;
			}
		}

		/* @brief Get a reference to a row of the matrix as a vector 
		 * @param i Row number 0 indexed */
		public btIVector3 this[int i]
		{
			get
			{
				return getRow( i );
			}
		}
		public double this[int i, int j]
		{
			set
			{
				switch( i )
				{
					default:
#if PARANOID_ASSERTS
						Debug.Assert( i > 3 || i < 0 );
#endif
					case 0:
						switch( j )
						{
							default:
#if PARANOID_ASSERTS
								Debug.Assert( j > 3 || j < 0 );
#endif
							case 0: m_el0.x = value; break;
							case 1: m_el0.y = value; break;
							case 2: m_el0.z = value; break;
							case 3: m_el0.w = value; break;
						}
						break;
					case 1:
						switch( j )
						{
							default:
#if PARANOID_ASSERTS
								Debug.Assert( j > 3 || j < 0 );
#endif
							case 0: m_el1.x = value; break;
							case 1: m_el1.y = value; break;
							case 2: m_el1.z = value; break;
							case 3: m_el1.w = value; break;
						}
						break;
					case 2:
						switch( j )
						{
							default:
#if PARANOID_ASSERTS
								Debug.Assert( j > 3 || j < 0 );
#endif
							case 0: m_el2.x = value; break;
							case 1: m_el2.y = value; break;
							case 2: m_el2.z = value; break;
							case 3: m_el2.w = value; break;
						}
						break;
					case 3:
						switch( j )
						{
							default:
#if PARANOID_ASSERTS
								Debug.Assert( j > 3 || j < 0 );
#endif
							case 0: m_el3.x = value; break;
							case 1: m_el3.y = value; break;
							case 2: m_el3.z = value; break;
							case 3: m_el3.w = value; break;
						}
						break;
				}
			}
			get
			{
				switch( i )
				{
					default:
#if PARANOID_ASSERTS
						Debug.Assert( i > 3 || i < 0 );
#endif
					case 0:
						switch( j )
						{
							default:
#if PARANOID_ASSERTS
								Debug.Assert( j > 3 || j < 0 );
#endif
							case 0: return m_el0.x;
							case 1: return m_el0.y;
							case 2: return m_el0.z;
							case 3: return m_el0.w;
						}
					case 1:
						switch( j )
						{
							default:
#if PARANOID_ASSERTS
								Debug.Assert( j > 3 || j < 0 );
#endif
							case 0: return m_el1.x;
							case 1: return m_el1.y;
							case 2: return m_el1.z;
							case 3: return m_el1.w;
						}
					case 2:
						switch( j )
						{
							default:
#if PARANOID_ASSERTS
								Debug.Assert( j > 3 || j < 0 );
#endif
							case 0: return m_el2.x;
							case 1: return m_el2.y;
							case 2: return m_el2.z;
							case 3: return m_el2.w;
						}
					case 3:
						switch( j )
						{
							default:
#if PARANOID_ASSERTS
								Debug.Assert( j > 3 || j < 0 );
#endif
							case 0: return m_el3.x;
							case 1: return m_el3.y;
							case 2: return m_el3.z;
							case 3: return m_el3.w;
						}
				}
			}
		}

#if asdfasdf
		/* @brief Set from the rotational part of a 4x4 OpenGL matrix
		 * @param m A pointer to the beginning of the array of scalars*/
		public void setFromOpenGLSubMatrix(stringbtScalar m )
		{
			m_el[0].setValue( m[0], m[4], m[8] );
			m_el[1].setValue( m[1], m[5], m[9] );
			m_el[2].setValue( m[2], m[6], m[10] );
		}
#endif

		/* @brief Set the values of the matrix explicitly (row major)
		 * @param xx Top left
		 * @param xy Top Middle
		 * @param xz Top Right
		 * @param yx Middle Left
		 * @param yy Middle Middle
		 * @param yz Middle Right
		 * @param zx Bottom Left
		 * @param zy Bottom Middle
		 * @param zz Bottom Right*/
		public void setValue( double xx, double xy, double xz,
			double yx, double yy, double yz,
			double zx, double zy, double zz )
		{
			m_el0.x = xx; m_el1.x = xy; m_el2.x = xz;
			m_el0.y = yx; m_el1.y = yy; m_el2.y = yz;
			m_el0.z = zx; m_el1.z = zy; m_el2.z = zz;
		}

		public static void setValue( out btMatrix3x3 m, double xx, double xy, double xz,
			double yx, double yy, double yz,
			double zx, double zy, double zz )
		{
			m.m_el0.x = xx; m.m_el1.x = xy; m.m_el2.x = xz; m.m_el3.x = 0;
			m.m_el0.y = yx; m.m_el1.y = yy; m.m_el2.y = yz; m.m_el3.y = 0;
			m.m_el0.z = zx; m.m_el1.z = zy; m.m_el2.z = zz; m.m_el3.z = 0;
			m.m_el0.w = 0; m.m_el1.w = 0; m.m_el2.w = 0; m.m_el3.w = 1;
		}

		/* @brief Set the matrix from a quaternion
		 * @param q The Quaternion to match */
		public void setRotation( ref btQuaternion q )
		{
			double d = q.length2();
#if PARANOID_ASSERTS
			Debug.Assert( d != 0 );
#endif
			double s = 2.0 / d;

			double xs = q.x * s, ys = q.y * s, zs = q.z * s;
			double wx = q.w * xs, wy = q.w * ys, wz = q.w * zs;
			double xx = q.x * xs, xy = q.x * ys, xz = q.x * zs;
			double yy = q.y * ys, yz = q.y * zs, zz = q.z * zs;
			setValue(
				1.0 - ( yy + zz ), xy - wz, xz + wy,
				xy + wz, 1.0 - ( xx + zz ), yz - wx,
				xz - wy, yz + wx, 1.0 - ( xx + yy ) );
		}

		public static void setRotation( out btMatrix3x3 result, ref btQuaternion q )
		{
			double d = q.length2();
#if PARANOID_ASSERTS
			Debug.Assert( d != 0 );
#endif
			double s = 2.0 / d;

			double xs = q.x * s, ys = q.y * s, zs = q.z * s;
			double wx = q.w * xs, wy = q.w * ys, wz = q.w * zs;
			double xx = q.x * xs, xy = q.x * ys, xz = q.x * zs;
			double yy = q.y * ys, yz = q.y * zs, zz = q.z * zs;
			btMatrix3x3.setValue( out result,
				1.0 - ( yy + zz ), xy - wz, xz + wy,
				xy + wz, 1.0 - ( xx + zz ), yz - wx,
				xz - wy, yz + wx, 1.0 - ( xx + yy ) );
		}


		/* @brief Set the matrix from euler angles using YPR around YXZ respectively
		 * @param yaw Yaw about Y axis
		 * @param pitch Pitch about X axis
		 * @param roll Roll about Z axis 
*/
		public void setEulerYPR( double yaw, double pitch, double roll )
		{
			setEulerZYX( roll, pitch, yaw );
		}

		/* @brief Set the matrix from euler angles YPR around ZYX axes
		 @param eulerX Roll about X axis
		 @param eulerY Pitch around Y axis
		 @param eulerZ Yaw aboud Z axis
		 
		 These angles are used to produce a rotation matrix. The euler
		 angles are applied in ZYX order. I.e a vector is first rotated 
		 about X then Y and then Z
		*/
		public void setEulerZYX( double eulerX, double eulerY, double eulerZ )
		{
			///@todo proposed to reverse this since it's labeled zyx but takes arguments xyz and it will match all other parts of the code
			double ci = btScalar.btCos( eulerX );
			double cj = btScalar.btCos( eulerY );
			double ch = btScalar.btCos( eulerZ );
			double si = btScalar.btSin( eulerX );
			double sj = btScalar.btSin( eulerY );
			double sh = btScalar.btSin( eulerZ );
			double cc = ci * ch;
			double cs = ci * sh;
			double sc = si * ch;
			double ss = si * sh;

			setValue( cj * ch, sj * sc - cs, sj * cc + ss,
				cj * sh, sj * ss + cc, sj * cs - sc,
				-sj, cj * si, cj * ci );
		}

		/*@brief Set the matrix to the identity */
		public void setIdentity()
		{
			//this = Identity;
			setValue( 1.0, 0, 0,
				0, 1.0, 0,
				0, 0, 1.0 );
		}

		public static btMatrix3x3 Identity = new btMatrix3x3(
			1.0, 0, 0,
			0, 1.0, 0,
			0, 0, 1.0 );

#if asdfsdf
		/*@brief Fill the rotational part of an OpenGL matrix and clear the shear/perspective
		 @param m The array to be filled */
		void getOpenGLSubMatrix( double m )
		{
			//OpenTK.Matrix4 mat; mat.
			m[0] = (double)( m_el0.x );
			m[1] = (double)( m_el1.x );
			m[2] = (double)( m_el2.x );
			m[3] = 0;
			m[4] = (double)( m_el0.y );
			m[5] = (double)( m_el1.y );
			m[6] = (double)( m_el2.y );
			m[7] = 0;
			m[8] = (double)( m_el0.z );
			m[9] = (double)( m_el1.z );
			m[10] = (double)( m_el2.z );
			m[11] = 0;
		}
#endif

		/*@brief Get the matrix represented as a quaternion 
		 @param q The quaternion which will be set */
		public void getRotation( out btQuaternion result )
		{
			double trace = m_el0.x + m_el1.y + m_el2.z;

			double[] temp = new double[4];

			if( trace > 0 )
			{
				double s = btScalar.btSqrt( trace + 1.0 );
				temp[3] = ( s * 0.5 );
				s = 0.5 / s;

				temp[0] = ( ( m_el2.y - m_el1.z ) * s );
				temp[1] = ( ( m_el0.z - m_el2.x ) * s );
				temp[2] = ( ( m_el1.x - m_el0.y ) * s );
			}
			else
			{
				int i = m_el0.x < m_el1.y ?
					( m_el1.y < m_el2.z ? 2 : 1 ) :
					( m_el0.x < m_el2.z ? 2 : 0 );
				int j = ( i + 1 ) % 3;
				int k = ( i + 2 ) % 3;

				double s = btScalar.btSqrt( this[i, i] - this[j, j] - this[k, k] + 1.0 );
				temp[i] = s * 0.5;
				s = 0.5 / s;

				temp[3] = ( this[k, j] - this[j, k] ) * s;
				temp[j] = ( this[j, i] + this[i, j] ) * s;
				temp[k] = ( this[k, i] + this[i, k] ) * s;
			}
			result.x = temp[0];
			result.y = temp[1];
			result.z = temp[2];
			result.w = temp[3];
		}

		/*@brief Get the matrix represented as euler angles around YXZ, roundtrip with setEulerYPR
		 @param yaw Yaw around Y axis
		 @param pitch Pitch around X axis
		 @param roll around Z axis */
		public void getEulerYPR( out double yaw, out double pitch, out double roll )
		{

			// first use the normal calculus
			yaw = btScalar.btAtan2( m_el1.x, m_el0.x );
			pitch = btScalar.btAsin( -m_el2.x );
			roll = btScalar.btAtan2( m_el2.y, m_el2.z );

			// on pitch = +/-HalfPI
			if( btScalar.btFabs( pitch ) == btScalar.SIMD_HALF_PI )
			{
				if( yaw > 0 )
					yaw -= btScalar.SIMD_PI;
				else
					yaw += btScalar.SIMD_PI;

				if( roll > 0 )
					roll -= btScalar.SIMD_PI;
				else
					roll += btScalar.SIMD_PI;
			}
		}

#if false

	static double btGetMatrixElem( ref btMatrix3x3 mat, int index )
	{
		int i = index % 3;
		int j = index / 3;
		return mat[i][j];
	}


		// method 2 - found in 6dof constraint
		///MatrixToEulerXYZ from http://www.geometrictools.com/LibFoundation/Mathematics/Wm4Matrix3.inl.html
		static bool matrixToEulerXYZ( btMatrix3x3& mat, ref btVector3 xyz )
		{
			//	// rot =  cy*cz          -cy*sz           sy
			//	//        cz*sx*sy+cx*sz  cx*cz-sx*sy*sz -cy*sx
			//	//       -cx*cz*sy+sx*sz  cz*sx+cx*sy*sz  cx*cy
			//

			double fi = btGetMatrixElem( mat, 2 );
			if( fi < (double)( 1.0f ) )
			{
				if( fi > (double)( -1.0f ) )
				{
					xyz[0] = btAtan2( -btGetMatrixElem( mat, 5 ), btGetMatrixElem( mat, 8 ) );
					xyz[1] = btAsin( btGetMatrixElem( mat, 2 ) );
					xyz[2] = btAtan2( -btGetMatrixElem( mat, 1 ), btGetMatrixElem( mat, 0 ) );
					return true;
				}
				else
				{
					// WARNING.  Not unique.  XA - ZA = -atan2(r10,r11)
					xyz[0] = -btAtan2( btGetMatrixElem( mat, 3 ), btGetMatrixElem( mat, 4 ) );
					xyz[1] = -SIMD_HALF_PI;
					xyz[2] = (double)( 0.0 );
					return false;
				}
			}
			else
			{
				// WARNING.  Not unique.  XAngle + ZAngle = atan2(r10,r11)
				xyz[0] = btAtan2( btGetMatrixElem( mat, 3 ), btGetMatrixElem( mat, 4 ) );
				xyz[1] = SIMD_HALF_PI;
				xyz[2] = 0.0;
			}
			return false;
		}
#endif

		struct Euler
		{
			internal double yaw;
			internal double pitch;
			internal double roll;
		};

		public void getEulerXYZ( out btVector3 result )
		{
			result.w = 0;
			getEulerYPR( out result.y, out result.x, out result.z );
		}
		/*@brief Get the matrix represented as euler angles around ZYX
		 @param yaw Yaw around Y axis
		 @param pitch Pitch around X axis
		 @param roll around Z axis 
		 @param solution_number Which solution of two possible solutions ( 1 or 2) are possible values*/
		public void getEulerZYX( out double yaw, out double pitch, out double roll, int solution_number = 1 )
		{

			Euler euler_out = new Euler();
			Euler euler_out2 = new Euler(); //second solution
											//get the pointer to the raw data

			// Check that pitch is not at a singularity
			if( btScalar.btFabs( m_el2.x ) >= 1 )
			{
				euler_out.yaw = 0;
				euler_out2.yaw = 0;

				// From difference of angles formula
				double delta = btScalar.btAtan2( m_el0.x, m_el0.z );
				if( m_el2.x > 0 )  //gimbal locked up
				{
					euler_out.pitch = btScalar.SIMD_PI / 2.0;
					euler_out2.pitch = btScalar.SIMD_PI / 2.0;
					euler_out.roll = euler_out.pitch + delta;
					euler_out2.roll = euler_out.pitch + delta;
				}
				else // gimbal locked down
				{
					euler_out.pitch = -btScalar.SIMD_PI / 2.0;
					euler_out2.pitch = -btScalar.SIMD_PI / 2.0;
					euler_out.roll = -euler_out.pitch + delta;
					euler_out2.roll = -euler_out.pitch + delta;
				}
			}
			else
			{
				euler_out.pitch = -btScalar.btAsin( m_el2.x );
				euler_out2.pitch = btScalar.SIMD_PI - euler_out.pitch;

				euler_out.roll = btScalar.btAtan2( m_el2.y / btScalar.btCos( euler_out.pitch ),
					m_el2.z / btScalar.btCos( euler_out.pitch ) );
				euler_out2.roll = btScalar.btAtan2( m_el2.y / btScalar.btCos( euler_out2.pitch ),
					m_el2.z / btScalar.btCos( euler_out2.pitch ) );

				euler_out.yaw = btScalar.btAtan2( m_el1.x / btScalar.btCos( euler_out.pitch ),
					m_el0.x / btScalar.btCos( euler_out.pitch ) );
				euler_out2.yaw = btScalar.btAtan2( m_el1.x / btScalar.btCos( euler_out2.pitch ),
					m_el0.x / btScalar.btCos( euler_out2.pitch ) );
			}

			if( solution_number == 1 )
			{
				yaw = euler_out.yaw;
				pitch = euler_out.pitch;
				roll = euler_out.roll;
			}
			else
			{
				yaw = euler_out2.yaw;
				pitch = euler_out2.pitch;
				roll = euler_out2.roll;
			}
		}

		/*@brief Create a scaled copy of the matrix 
		 @param s Scaling vector The elements of the vector will scale each column */

		public void scaled( ref btVector3 s, out btMatrix3x3 result )
		{
			btMatrix3x3.setValue( out result,
				m_el0.x * s.x, m_el0.y * s.y, m_el0.z * s.z,
				m_el1.x * s.x, m_el1.y * s.y, m_el1.z * s.z,
				m_el2.x * s.x, m_el2.y * s.y, m_el2.z * s.z );
		}

		/// Solve A * x = b, where b is a column vector. This is more efficient
		/// than computing the inverse in one-shot cases.
		///Solve33 is from Box2d, thanks to Erin Catto,
		public void solve33( ref btVector3 b, out btVector3 result )
		{
			btVector3 col1 = getColumn( 0 );
			btVector3 col2 = getColumn( 1 );
			btVector3 col3 = getColumn( 2 );
			btVector3 tmp;
			col2.cross( ref col3, out tmp );
			double det = col1.dot( ref tmp );
			if( btScalar.btFabs( det ) > btScalar.SIMD_EPSILON )
			{
				det = 1.0f / det;
			}
			col2.cross( ref col3, out tmp );
			result.x = det * b.dot( ref tmp );
			b.cross( ref col3, out tmp );
			result.y = det * col1.dot( ref tmp );
			col2.cross( ref b, out tmp );
			result.z = det * col1.dot( ref tmp );
			result.w = 0;
		}

		public double tdotx( ref btVector3 v )
		{
			return m_el0.x * v.x + m_el1.x * v.y + m_el2.x * v.z;
		}
		public double tdoty( ref btVector3 v )
		{
			return m_el0.y * v.x + m_el1.y * v.y + m_el2.y * v.z;
		}
		public double tdotz( ref btVector3 v )
		{
			return m_el0.z * v.x + m_el1.z * v.y + m_el2.z * v.z;
		}


		/*@brief diagonalizes this matrix by the Jacobi method.
		 @param rot stores the rotation from the coordinate system in which the matrix is diagonal to the original
		 coordinate system, i.e., old_this = rot * new_this * rot^T. 
		 @param threshold See iteration
		 @param iteration The iteration stops when all off-diagonal elements are less than the threshold multiplied 
		 by the sum of the absolute values of the diagonal, or when maxSteps have been executed. 
		 
		 Note that this matrix is assumed to be symmetric. 
*/
		public void diagonalize( out btMatrix3x3 rot, double threshold, int maxSteps )
		{
			rot = Identity;
			for( int step = maxSteps; step > 0; step-- )
			{
				// find off-diagonal element [p,q] with largest magnitude
				int p = 0;
				int q = 1;
				int r = 2;
				double max = btScalar.btFabs( m_el0.y );
				double v = btScalar.btFabs( m_el0.z );
				if( v > max )
				{
					q = 2;
					r = 1;
					max = v;
				}
				v = btScalar.btFabs( m_el1.z );
				if( v > max )
				{
					p = 1;
					q = 2;
					r = 0;
					max = v;
				}

				double t = threshold * ( btScalar.btFabs( m_el0.x ) + btScalar.btFabs( m_el1.y ) + btScalar.btFabs( m_el2.z ) );
				if( max <= t )
				{
					if( max <= btScalar.SIMD_EPSILON * t )
					{
						return;
					}
					step = 1;
				}

				// compute Jacobi rotation J which leads to a zero for element [p,q] 
				double mpq = this[p, q];
				double theta = ( this[q, q] - this[p, p] ) / ( 2 * mpq );
				double theta2 = theta * theta;
				double cos;
				double sin;
				if( theta2 * theta2 < 10 / btScalar.SIMD_EPSILON )
				{
					t = ( theta >= 0 ) ? 1 / ( theta + btScalar.btSqrt( 1 + theta2 ) )
						: 1 / ( theta - btScalar.btSqrt( 1 + theta2 ) );
					cos = 1 / btScalar.btSqrt( 1 + t * t );
					sin = cos * t;
				}
				else
				{
					// approximation for large theta-value, i.e., a nearly diagonal matrix
					t = 1 / ( theta * ( 2 + 0.5 / theta2 ) );
					cos = 1 - 0.5 * t * t;
					sin = cos * t;
				}

				// apply rotation to matrix (this = J^T * this * J)
				this[p, q] = this[q, p] = 0;
				this[p, p] -= t * mpq;
				this[q, q] += t * mpq;
				double mrp = this[r, p];
				double mrq = this[r, q];
				this[r, p] = this[p, r] = cos * mrp - sin * mrq;
				this[r, q] = this[q, r] = cos * mrq + sin * mrp;

				// apply rotation to rot (rot = rot * J)
				for( int i = 0; i < 3; i++ )
				{
					btIVector3 row = rot[i];
					mrp = row[p];
					mrq = row[q];
					row[p] = cos * mrp - sin * mrq;
					row[q] = cos * mrq + sin * mrp;
				}
			}
		}




		/*@brief Calculate the matrix cofactor 
		 @param r1 The first row to use for calculating the cofactor
		 @param c1 The first column to use for calculating the cofactor
		 @param r1 The second row to use for calculating the cofactor
		 @param c1 The second column to use for calculating the cofactor
		 See http://en.wikipedia.org/wiki/Cofactor_(linear_algebra) for more details
*/
		double cofac( int r1, int c1, int r2, int c2 )
		{
			return this[r1, c1] * this[r2, c2] - this[r1, c2] * this[r2, c1];
		}


		public static void Mult( ref btMatrix3x3 m, double k, out btMatrix3x3 result )
		{
			m.m_el0.Mult( k, out result.m_el0 );
			m.m_el1.Mult( k, out result.m_el1 );
			m.m_el2.Mult( k, out result.m_el2 );
			m.m_el3.Mult( k, out result.m_el3 );
		}
		public void Mult( double k, out btMatrix3x3 result )
		{
			m_el0.Mult( k, out result.m_el0 );
			m_el1.Mult( k, out result.m_el1 );
			m_el2.Mult( k, out result.m_el2 );
			m_el3.Mult( k, out result.m_el3 );
		}
		public static void Mult( ref btMatrix3x3 m, ref btVector3 v, out btVector3 result )
		{
			result.x = m.m_el0.dot( ref v );
			result.y = m.m_el1.dot( ref v );
			result.z = m.m_el2.dot( ref v );
			result.w = m.m_el3.dot( ref v );
		}

		public void Mult( ref btVector3 v, out btVector3 result )
		{
			result.x = m_el0.dot( ref v );
			result.y = m_el1.dot( ref v );
			result.z = m_el2.dot( ref v );
			result.w = m_el3.dot( ref v );
		}

		public static void Mult( ref btMatrix3x3 m1, ref btMatrix3x3 m2, out btMatrix3x3 result )
		{
			result.m_el0.x = m2.tdotx( ref m1.m_el0 );
			result.m_el0.y = m2.tdoty( ref m1.m_el0 );
			result.m_el0.z = m2.tdotz( ref m1.m_el0 );
			result.m_el0.w = 0;
			result.m_el1.x = m2.tdotx( ref m1.m_el1 );
			result.m_el1.y = m2.tdoty( ref m1.m_el1 );
			result.m_el1.z = m2.tdotz( ref m1.m_el1 );
			result.m_el1.w = 0;
			result.m_el2.x = m2.tdotx( ref m1.m_el2 );
			result.m_el2.y = m2.tdoty( ref m1.m_el2 );
			result.m_el2.z = m2.tdotz( ref m1.m_el2 );
			result.m_el2.w = 0;
			result.m_el3.x = 0;
			result.m_el3.y = 0;
			result.m_el3.z = 0;
			result.m_el3.w = 1;
		}

		public void Mult( ref btMatrix3x3 m2, out btMatrix3x3 result )
		{
			result.m_el0.x = m2.tdotx( ref m_el0 );
			result.m_el0.y = m2.tdoty( ref m_el0 );
			result.m_el0.z = m2.tdotz( ref m_el0 );
			result.m_el0.w = 0;
			result.m_el1.x = m2.tdotx( ref m_el1 );
			result.m_el1.y = m2.tdoty( ref m_el1 );
			result.m_el1.z = m2.tdotz( ref m_el1 );
			result.m_el1.w = 0;
			result.m_el2.x = m2.tdotx( ref m_el2 );
			result.m_el2.y = m2.tdoty( ref m_el2 );
			result.m_el2.z = m2.tdotz( ref m_el2 );
			result.m_el2.w = 0;
			result.m_el3.x = 0;
			result.m_el3.y = 0;
			result.m_el3.z = 0;
			result.m_el3.w = 1;
		}

		public static void Add( ref btMatrix3x3 m1, ref btMatrix3x3 m2, out btMatrix3x3 result )
		{
			m1.m_el0.Add( ref m2.m_el0, out result.m_el0 );
			m1.m_el1.Add( ref m2.m_el1, out result.m_el1 );
			m1.m_el2.Add( ref m2.m_el2, out result.m_el2 );
			m1.m_el3.Add( ref m2.m_el3, out result.m_el3 );
		}
		public void Add( ref btMatrix3x3 m2, out btMatrix3x3 result )
		{
			m_el0.Add( ref m2.m_el0, out result.m_el0 );
			m_el1.Add( ref m2.m_el1, out result.m_el1 );
			m_el2.Add( ref m2.m_el2, out result.m_el2 );
			m_el3.Add( ref m2.m_el3, out result.m_el3 );
		}

		public static void Sub( ref btMatrix3x3 m1, ref btMatrix3x3 m2, out btMatrix3x3 result )
		{
			m1.m_el0.Sub( ref m2.m_el0, out result.m_el0 );
			m1.m_el1.Sub( ref m2.m_el1, out result.m_el1 );
			m1.m_el2.Sub( ref m2.m_el2, out result.m_el2 );
			m1.m_el3.Sub( ref m2.m_el3, out result.m_el3 );
		}


		public void Sub( ref btMatrix3x3 m2, out btMatrix3x3 result )
		{
			m_el0.Sub( ref m2.m_el0, out result.m_el0 );
			m_el1.Sub( ref m2.m_el1, out result.m_el1 );
			m_el2.Sub( ref m2.m_el2, out result.m_el2 );
			m_el3.Sub( ref m2.m_el3, out result.m_el3 );
		}



		public double determinant()
		{
			return btVector3.btTriple( ref m_el0, ref m_el1, ref m_el2 );
		}

		public void absolute( out btMatrix3x3 result )
		{
			btMatrix3x3.setValue( out result,
					btScalar.btFabs( m_el0.x ), btScalar.btFabs( m_el0.y ), btScalar.btFabs( m_el0.z ),
					btScalar.btFabs( m_el1.x ), btScalar.btFabs( m_el1.y ), btScalar.btFabs( m_el1.z ),
					btScalar.btFabs( m_el2.x ), btScalar.btFabs( m_el2.y ), btScalar.btFabs( m_el2.z ) );
		}

		public void transpose( out btMatrix3x3 result )
		{
			result.m_el0.x = m_el0.x;
			result.m_el0.y = m_el1.x;
			result.m_el0.z = m_el2.x;
			result.m_el0.w = m_el3.x;

			result.m_el1.x = m_el0.y;
			result.m_el1.y = m_el1.y;
			result.m_el1.z = m_el2.y;
			result.m_el1.w = m_el3.y;

			result.m_el2.x = m_el0.z;
			result.m_el2.y = m_el1.z;
			result.m_el2.z = m_el2.z;
			result.m_el2.w = m_el3.z;

			result.m_el3.x = m_el0.w;
			result.m_el3.y = m_el1.w;
			result.m_el3.z = m_el2.w;
			result.m_el3.w = m_el3.w;
		}

		public btMatrix3x3 adjoint()
		{
			return new btMatrix3x3( cofac( 1, 1, 2, 2 ), cofac( 0, 2, 2, 1 ), cofac( 0, 1, 1, 2 ),
				cofac( 1, 2, 2, 0 ), cofac( 0, 0, 2, 2 ), cofac( 0, 2, 1, 0 ),
				cofac( 1, 0, 2, 1 ), cofac( 0, 1, 2, 0 ), cofac( 0, 0, 1, 1 ) );
		}

		public void adjoint( out btMatrix3x3 result )
		{
			btMatrix3x3.setValue( out result, cofac( 1, 1, 2, 2 ), cofac( 0, 2, 2, 1 ), cofac( 0, 1, 1, 2 ),
				cofac( 1, 2, 2, 0 ), cofac( 0, 0, 2, 2 ), cofac( 0, 2, 1, 0 ),
				cofac( 1, 0, 2, 1 ), cofac( 0, 1, 2, 0 ), cofac( 0, 0, 1, 1 ) );
		}
		/*
				public btMatrix3x3 inverse()
				{
					btVector3 co = new btVector3( cofac( 1, 1, 2, 2 ), cofac( 1, 2, 2, 0 ), cofac( 1, 0, 2, 1 ) );
					double det = m_el0.dot( ref co );
#if PARANOID_ASSERTS
					Debug.Assert( det != 0 );
#endif
					double s = 1.0 / det;
					return new btMatrix3x3( co.x * s, cofac( 0, 2, 2, 1 ) * s, cofac( 0, 1, 1, 2 ) * s,
							co.y * s, cofac( 0, 0, 2, 2 ) * s, cofac( 0, 2, 1, 0 ) * s,
							co.z * s, cofac( 0, 1, 2, 0 ) * s, cofac( 0, 0, 1, 1 ) * s );
				}
				*/
		public void inverse( out btMatrix3x3 result )
		{
			btVector3 co = new btVector3( cofac( 1, 1, 2, 2 ), cofac( 1, 2, 2, 0 ), cofac( 1, 0, 2, 1 ) );
			double det = m_el0.dot( ref co );
#if PARANOID_ASSERTS
			Debug.Assert( det != 0 );
#endif
			double s = 1.0 / det;
			result.m_el0.x = co.x * s;
			result.m_el0.y = cofac( 0, 2, 2, 1 ) * s;
			result.m_el0.z = cofac( 0, 1, 1, 2 ) * s;
			result.m_el0.w = 0;
			result.m_el1.x = co.y * s;
			result.m_el1.y = cofac( 0, 0, 2, 2 ) * s;
			result.m_el1.z = cofac( 0, 2, 1, 0 ) * s;
			result.m_el1.w = 0;
			result.m_el2.x = co.z * s;
			result.m_el2.y = cofac( 0, 1, 2, 0 ) * s;
			result.m_el2.z = cofac( 0, 0, 1, 1 ) * s;
			result.m_el2.w = 0;

			result.m_el3.x = 0;
			result.m_el3.y = 0;
			result.m_el3.z = 0;
			result.m_el3.w = 0;
		}

		public void transposeTimes( ref btMatrix3x3 m, out btMatrix3x3 result )
		{
			result.m_el0.x = m_el0.x * m.m_el0.x + m_el1.x * m.m_el1.x + m_el2.x * m.m_el2.x;
			result.m_el0.y = m_el0.x * m.m_el0.y + m_el1.x * m.m_el1.y + m_el2.x * m.m_el2.y;
			result.m_el0.z = m_el0.x * m.m_el0.z + m_el1.x * m.m_el1.z + m_el2.x * m.m_el2.z;
			result.m_el0.w = 0;
			result.m_el1.x = m_el0.y * m.m_el0.x + m_el1.y * m.m_el1.x + m_el2.y * m.m_el2.x;
			result.m_el1.y = m_el0.y * m.m_el0.y + m_el1.y * m.m_el1.y + m_el2.y * m.m_el2.y;
			result.m_el1.z = m_el0.y * m.m_el0.z + m_el1.y * m.m_el1.z + m_el2.y * m.m_el2.z;
			result.m_el1.w = 0;
			result.m_el2.x = m_el0.z * m.m_el0.x + m_el1.z * m.m_el1.x + m_el2.z * m.m_el2.x;
			result.m_el2.y = m_el0.z * m.m_el0.y + m_el1.z * m.m_el1.y + m_el2.z * m.m_el2.y;
			result.m_el2.z = m_el0.z * m.m_el0.z + m_el1.z * m.m_el1.z + m_el2.z * m.m_el2.z;
			result.m_el2.w = 0;

			result.m_el3.x = 0;
			result.m_el3.y = 0;
			result.m_el3.z = 0;
			result.m_el3.w = 1;
		}

		public void timesTranspose( ref btMatrix3x3 m, out btMatrix3x3 result )
		{
			result.m_el0.x = m_el0.dot( ref m.m_el0 );
			result.m_el0.y = m_el0.dot( ref m.m_el1 );
			result.m_el0.z = m_el0.dot( ref m.m_el2 );
			result.m_el0.w = 0;

			result.m_el1.x = m_el1.dot( ref m.m_el0 );
			result.m_el1.y = m_el1.dot( ref m.m_el1 );
			result.m_el1.z = m_el1.dot( ref m.m_el2 );
			result.m_el1.w = 0;

			result.m_el2.x = m_el2.dot( ref m.m_el0 );
			result.m_el2.y = m_el2.dot( ref m.m_el1 );
			result.m_el2.z = m_el2.dot( ref m.m_el2 );
			result.m_el2.w = 0;

			result.m_el3.x = 0;
			result.m_el3.y = 0;
			result.m_el3.z = 0;
			result.m_el3.w = 1;
		}

#if !DISABLE_OPERATORS
		public static btVector3 operator *( btMatrix3x3 m, btVector3 v )
		{
			return new btVector3( m.m_el0.dot( ref v ),
					m.m_el1.dot( ref v ),
					m.m_el2.dot( ref v ) );
		}
		public static btVector3 operator *( btVector3 v, btMatrix3x3 m )
		{
			return new btVector3( m.tdotx( ref v ),
				m.tdoty( ref v ),
				m.tdotz( ref v )
			 );
		}
#endif
		public static void Apply( ref btMatrix3x3 m, ref btVector3 v, out btVector3 result )
		{
			result.x = m.m_el0.dot( ref v );
			result.y = m.m_el1.dot( ref v );
			result.z = m.m_el2.dot( ref v );
			result.w = 0;
		}
		public void Apply( ref btVector3 v, out btVector3 result )
		{
			result.x = m_el0.dot( ref v );
			result.y = m_el1.dot( ref v );
			result.z = m_el2.dot( ref v );
			result.w = 0;
		}
		public void ApplyInverse( ref btVector3 v, out btVector3 result )
		{
			result.x = tdotx( ref v );
			result.y = tdoty( ref v );
			result.z = tdotz( ref v );
			result.w = 0;
		}

		/*
				public static btVector3 operator *( ref btVector3 v, ref btMatrix3x3 m )
				{
					return new btVector3( m.tdotx( ref v ), m.tdoty( ref v ), m.tdotz( ref v ) );
				}
				*/

		public void Apply( ref btMatrix3x3 m2, out btMatrix3x3 result )
		{
			result.m_el0.x = m2.tdotx( ref m_el0 );
			result.m_el0.y = m2.tdoty( ref m_el0 );
			result.m_el0.z = m2.tdotz( ref m_el0 );
			result.m_el0.w = 0;

			result.m_el1.x = m2.tdotx( ref m_el1 );
			result.m_el1.y = m2.tdoty( ref m_el1 );
			result.m_el1.z = m2.tdotz( ref m_el1 );
			result.m_el1.w = 0;

			result.m_el2.x = m2.tdotx( ref m_el2 );
			result.m_el2.y = m2.tdoty( ref m_el2 );
			result.m_el2.z = m2.tdotz( ref m_el2 );
			result.m_el2.w = 0;

			result.m_el3.x = 0;
			result.m_el3.y = 0;
			result.m_el3.z = 0;
			result.m_el3.w = 1;
		}

		public void Apply( btIMatrix3x3 m2, out btMatrix3x3 result )
		{
			result.m_el0.x = m2.tdotx( ref m_el0 );
			result.m_el0.y = m2.tdoty( ref m_el0 );
			result.m_el0.z = m2.tdotz( ref m_el0 );
			result.m_el0.w = 0;

			result.m_el1.x = m2.tdotx( ref m_el1 );
			result.m_el1.y = m2.tdoty( ref m_el1 );
			result.m_el1.z = m2.tdotz( ref m_el1 );
			result.m_el1.w = 0;

			result.m_el2.x = m2.tdotx( ref m_el2 );
			result.m_el2.y = m2.tdoty( ref m_el2 );
			result.m_el2.z = m2.tdotz( ref m_el2 );
			result.m_el2.w = 0;

			result.m_el3.x = 0;
			result.m_el3.y = 0;
			result.m_el3.z = 0;
			result.m_el3.w = 1;
		}

		/*
		public btMatrix3x3 btMultTransposeLeft(btMatrix3x3 m1, btMatrix3x3 m2) {
		return btMatrix3x3(
		m1[0,0] * m2[0,0] + m1[1,0] * m2[1,0] + m1[2,0] * m2[2,0],
		m1[0,0] * m2[0,1] + m1[1,0] * m2[1,1] + m1[2,0] * m2[2,1],
		m1[0,0] * m2[0,2] + m1[1,0] * m2[1,2] + m1[2,0] * m2[2,2],
		m1[0,1] * m2[0,0] + m1[1,1] * m2[1,0] + m1[2,1] * m2[2,0],
		m1[0,1] * m2[0,1] + m1[1,1] * m2[1,1] + m1[2,1] * m2[2,1],
		m1[0,1] * m2[0,2] + m1[1,1] * m2[1,2] + m1[2,1] * m2[2,2],
		m1[0,2] * m2[0,0] + m1[1,2] * m2[1,0] + m1[2,2] * m2[2,0],
		m1[0,2] * m2[0,1] + m1[1,2] * m2[1,1] + m1[2,2] * m2[2,1],
		m1[0,2] * m2[0,2] + m1[1,2] * m2[1,2] + m1[2,2] * m2[2,2]);
		}
		*/

		/*@brief Equality operator between two matrices
		 It will test all elements are equal. * */
		public bool Equals( ref btMatrix3x3 m1 )
		{
			return ( m1.m_el0.x == m_el0.x && m1.m_el1.x == m_el1.x && m1.m_el2.x == m_el2.x &&
					m1.m_el0.y == m_el0.y && m1.m_el1.y == m_el1.y && m1.m_el2.y == m_el2.y &&
					m1.m_el0.z == m_el0.z && m1.m_el1.z == m_el1.z && m1.m_el2.z == m_el2.z );
		}

		/*
			///for serialization
			struct btMatrix3x3FloatData
			{
				btVector3FloatData m_el[3];
			};

			///for serialization
			struct btMatrix3x3DoubleData
			{
				btVector3DoubleData m_el[3];
			};




			public void btMatrix3x3::serialize(struct btMatrix3x3Data& dataOut)
		{
			for (int i = 0; i<3;i++)
				m_el[i].serialize( dataOut.m_el[i]);
		}

		public void btMatrix3x3::serializeFloat(struct btMatrix3x3FloatData& dataOut)
		{
			for (int i = 0; i<3;i++)
				m_el[i].serializeFloat( dataOut.m_el[i]);
		}


		public void btMatrix3x3::deSerialize(stringstruct btMatrix3x3Data& dataIn)
		{
			for (int i = 0; i<3;i++)
				m_el[i].deSerialize( dataIn.m_el[i]);
		}

		public void btMatrix3x3::deSerializeFloat(stringstruct btMatrix3x3FloatData& dataIn)
		{
			for (int i = 0; i<3;i++)
				m_el[i].deSerializeFloat( dataIn.m_el[i]);
		}

		public void btMatrix3x3::deSerializeDouble(stringstruct btMatrix3x3DoubleData& dataIn)
		{
			for (int i = 0; i<3;i++)
				m_el[i].deSerializeDouble( dataIn.m_el[i]);
		}
		*/
	}
}
