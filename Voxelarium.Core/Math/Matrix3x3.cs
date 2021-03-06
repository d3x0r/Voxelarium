//#define COLUMN_MAJOR_EXPECTED
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

This source version has been altered significantly from C++ to C#

 Now part of Voxelarium.
*/

using System;
using Voxelarium.Core.Support;

namespace Voxelarium.LinearMath
{
	public struct btMatrix3x3 
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
			float d = q.length2();
			//btFullAssert(d != (float)(0.0));
			float s = btScalar.BT_TWO / d;

			float xs = q.x * s, ys = q.y * s, zs = q.z * s;
			float wx = q.w * xs, wy = q.w * ys, wz = q.w * zs;
			float xx = q.x * xs, xy = q.x * ys, xz = q.x * zs;
			float yy = q.y * ys, yz = q.y * zs, zz = q.z * zs;
			m_el0.x = btScalar.BT_ONE - ( yy + zz ); m_el0.y = xy - wz; m_el0.z = xz + wy; m_el0.w = 0;
			m_el1.x = xy + wz; m_el1.y = btScalar.BT_ONE - ( xx + zz ); m_el1.z = yz - wx; m_el1.w = 0;
			m_el2.x = xz - wy; m_el2.y = yz + wx; m_el2.z = btScalar.BT_ONE - ( xx + yy ); m_el2.w = 0;
			m_el3.x = 0; m_el3.y = 0; m_el3.z = 0; m_el3.w = 1;

		}
		/*
		template <typename float>
		Matrix3x3(float yaw, float pitch, float roll)
		{ 
		setEulerYPR(yaw, pitch, roll);
		}
		*/
		/* @brief Constructor with row major formatting */
		public btMatrix3x3( float xx, float xy, float xz,
			float yx, float yy, float yz,
			float zx, float zy, float zz )
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


		public void Rotate( int axis, float angle )
		{
			//Log.log( " Rotate {0} {1}", axis, angle );
#if COLUMN_MAJOR_EXPECTED
			switch( axis )
			{
				default:
				case 0:
					{
						float savex = m_el0.z, savey = m_el1.z, savez = m_el2.z;
						float dsin = btScalar.btSin( angle )
							  , dcos = btScalar.btCos( angle );
						float v1x, v1y, v1z;
						float v2x, v2y, v2z;
						v1x = dcos * m_el0.z; v1y = dcos * m_el1.z; v1z = dcos * m_el2.z;
						v2x = dsin * m_el0.y; v2y = dsin * m_el1.y; v2z = dsin * m_el2.y;
						m_el0.z = v1x - v2x; m_el1.z = v1y - v2y; m_el2.z = v1z - v2z;
						v1x = savex * dsin; v1y = savey * dsin; v1z = savez * dsin;
						v2x = dcos * m_el0.y; v2y = dcos * m_el1.y; v2z = dcos * m_el2.y;
						m_el0.y = v1x + v2x; m_el1.y = v1y + v2y; m_el2.y = v1z + v2z;
						break;
					}
				case 1:
					{
						float savex = m_el0.x, savey = m_el1.x, savez = m_el2.x;
						float dsin = btScalar.btSin( angle )
							  , dcos = btScalar.btCos( angle );
						float v1x, v1y, v1z;
						float v2x, v2y, v2z;
						v1x = dcos * m_el0.x; v1y = dcos * m_el1.x; v1z = dcos * m_el2.x;
						v2x = dsin * m_el0.z; v2y = dsin * m_el1.z; v2z = dsin * m_el2.z;
						m_el0.x = v1x - v2x; m_el1.x = v1y - v2y; m_el2.x = v1z - v2z;
						v1x = savex * dsin; v1y = savey * dsin; v1z = savez * dsin;
						v2x = dcos * m_el0.z; v2y = dcos * m_el1.z; v2z = dcos * m_el2.z;
						m_el0.z = v1x + v2x; m_el1.z = v1y + v2y; m_el2.z = v1z + v2z;
						break;
					}
				case 2:
					{
						float savex = m_el0.x, savey = m_el1.x, savez = m_el2.x;
						float dsin = btScalar.btSin( angle )
							  , dcos = btScalar.btCos( angle );
						float v1x, v1y, v1z;
						float v2x, v2y, v2z;
						v1x = dcos * m_el0.x; v1y = dcos * m_el1.x; v1z = dcos * m_el2.x;
						v2x = dsin * m_el0.y; v2y = dsin * m_el1.y; v2z = dsin * m_el2.y;
						m_el0.x = v1x - v2x; m_el1.x = v1y - v2y; m_el2.x = v1z - v2z;
						v1x = savex * dsin; v1y = savey * dsin; v1z = savez * dsin;
						v2x = dcos * m_el0.y; v2y = dcos * m_el1.y; v2z = dcos * m_el2.y;
						m_el0.y = v1x + v2x; m_el1.y = v1y + v2y; m_el2.y = v1z + v2z;
						break;
					}
			}
#else
			float dsin = btScalar.btSin( angle )
				  , dcos = btScalar.btCos( angle );
			switch( axis )
			{

				default:
				case 0:
					{
						btVector3 savex = m_el1;
						btVector3 v1, v2;
						m_el1.Mult( dcos, out v1 );
						m_el2.Mult( dsin, out v2 );
						v1.Sub( ref v2, out m_el1 );
						savex.Mult( dsin, out v2 );
						m_el2.Mult( dcos, out v1 );
						v1.Add( ref v2, out m_el2 );
						break;
					}
				case 1:
					{
						btVector3 savex = m_el0;
						btVector3 v1, v2;
						m_el0.Mult( dcos, out v1 );
						m_el2.Mult( dsin, out v2 );
						v1.Sub( ref v2, out m_el0 );
						savex.Mult( dsin, out v2 );
						m_el2.Mult( dcos, out v1 );
						v1.Add( ref v2, out m_el2 );
						break;
					}
				case 2:
					{
						btVector3 savex = m_el0;
						btVector3 v1, v2;
						m_el0.Mult( dcos, out v1 );
						m_el1.Mult( dsin, out v2 );
						v1.Sub( ref v2, out m_el0 );
						savex.Mult( dsin, out v2 );
						m_el1.Mult( dcos, out v1 );
						v1.Add( ref v2, out m_el1 );
						break;
					}
			}
#endif
		}

		public void Rotate( float x, float y, float z )
		{
			if( x != 0 )
				Rotate( 0, x );
			if( y != 0 )
				Rotate( 1, y );
			if( z != 0 )
				Rotate( 2, z );
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

		public float this[int i, int j]
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
		public void setValue( float xx, float xy, float xz,
			float yx, float yy, float yz,
			float zx, float zy, float zz )
		{
			m_el0.x = xx; m_el1.x = xy; m_el2.x = xz;
			m_el0.y = yx; m_el1.y = yy; m_el2.y = yz;
			m_el0.z = zx; m_el1.z = zy; m_el2.z = zz;
		}

		public static void setValue( out btMatrix3x3 m, float xx, float xy, float xz,
			float yx, float yy, float yz,
			float zx, float zy, float zz )
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
			float d = q.length2();
#if PARANOID_ASSERTS
			Debug.Assert( d != 0 );
#endif
			float s = btScalar.BT_TWO / d;

			float xs = q.x * s, ys = q.y * s, zs = q.z * s;
			float wx = q.w * xs, wy = q.w * ys, wz = q.w * zs;
			float xx = q.x * xs, xy = q.x * ys, xz = q.x * zs;
			float yy = q.y * ys, yz = q.y * zs, zz = q.z * zs;
			setValue(
				btScalar.BT_ONE - ( yy + zz ), xy - wz, xz + wy,
				xy + wz, btScalar.BT_ONE - ( xx + zz ), yz - wx,
				xz - wy, yz + wx, btScalar.BT_ONE - ( xx + yy ) );
		}

		public static void setRotation( out btMatrix3x3 result, ref btQuaternion q )
		{
			float d = q.length2();
#if PARANOID_ASSERTS
			Debug.Assert( d != 0 );
#endif
			float s = btScalar.BT_TWO / d;

			float xs = q.x * s, ys = q.y * s, zs = q.z * s;
			float wx = q.w * xs, wy = q.w * ys, wz = q.w * zs;
			float xx = q.x * xs, xy = q.x * ys, xz = q.x * zs;
			float yy = q.y * ys, yz = q.y * zs, zz = q.z * zs;
			btMatrix3x3.setValue( out result,
				btScalar.BT_ONE - ( yy + zz ), xy - wz, xz + wy,
				xy + wz, btScalar.BT_ONE - ( xx + zz ), yz - wx,
				xz - wy, yz + wx, btScalar.BT_ONE - ( xx + yy ) );
		}


		/* @brief Set the matrix from euler angles using YPR around YXZ respectively
		 * @param yaw Yaw about Y axis
		 * @param pitch Pitch about X axis
		 * @param roll Roll about Z axis 
*/
		public void setEulerYPR( float yaw, float pitch, float roll )
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
		public void setEulerZYX( float eulerX, float eulerY, float eulerZ )
		{
			///@todo proposed to reverse this since it's labeled zyx but takes arguments xyz and it will match all other parts of the code
			float ci = btScalar.btCos( eulerX );
			float cj = btScalar.btCos( eulerY );
			float ch = btScalar.btCos( eulerZ );
			float si = btScalar.btSin( eulerX );
			float sj = btScalar.btSin( eulerY );
			float sh = btScalar.btSin( eulerZ );
			float cc = ci * ch;
			float cs = ci * sh;
			float sc = si * ch;
			float ss = si * sh;

			setValue( cj * ch, sj * sc - cs, sj * cc + ss,
				cj * sh, sj * ss + cc, sj * cs - sc,
				-sj, cj * si, cj * ci );
		}

		/*@brief Set the matrix to the identity */
		public void setIdentity()
		{
			//this = Identity;
			setValue( btScalar.BT_ONE, 0, 0,
				0, btScalar.BT_ONE, 0,
				0, 0, btScalar.BT_ONE );
		}

		public static btMatrix3x3 Identity = new btMatrix3x3(
			btScalar.BT_ONE, 0, 0,
			0, btScalar.BT_ONE, 0,
			0, 0, btScalar.BT_ONE );

#if asdfsdf
		/*@brief Fill the rotational part of an OpenGL matrix and clear the shear/perspective
		 @param m The array to be filled */
		void getOpenGLSubMatrix( float m )
		{
			//OpenTK.Matrix4 mat; mat.
			m[0] = (float)( m_el0.x );
			m[1] = (float)( m_el1.x );
			m[2] = (float)( m_el2.x );
			m[3] = 0;
			m[4] = (float)( m_el0.y );
			m[5] = (float)( m_el1.y );
			m[6] = (float)( m_el2.y );
			m[7] = 0;
			m[8] = (float)( m_el0.z );
			m[9] = (float)( m_el1.z );
			m[10] = (float)( m_el2.z );
			m[11] = 0;
		}
#endif

		/*@brief Get the matrix represented as a quaternion 
		 @param q The quaternion which will be set */
		public void getRotation( out btQuaternion result )
		{
			float trace = m_el0.x + m_el1.y + m_el2.z;

			float[] temp = new float[4];

			if( trace > 0 )
			{
				float s = btScalar.btSqrt( trace + btScalar.BT_ONE );
				temp[3] = ( s * btScalar.BT_HALF );
				s = btScalar.BT_TWO / s;

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

				float s = btScalar.btSqrt( this[i, i] - this[j, j] - this[k, k] + btScalar.BT_ONE );
				temp[i] = s * btScalar.BT_HALF;
				s = btScalar.BT_HALF / s;

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
		public void getEulerYPR( out float yaw, out float pitch, out float roll )
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


		struct Euler
		{
			internal float yaw;
			internal float pitch;
			internal float roll;
		};
		/*@brief Get the matrix represented as euler angles around ZYX
		 @param yaw Yaw around X axis
		 @param pitch Pitch around Y axis
		 @param roll around X axis 
		 @param solution_number Which solution of two possible solutions ( 1 or 2) are possible values*/
		public void getEulerZYX( out float yaw, out float pitch, out float roll, int solution_number = 1 )
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
				float delta = btScalar.btAtan2( m_el0.x, m_el0.z );
				if( m_el2.x > 0 )  //gimbal locked up
				{
					euler_out.pitch = btScalar.SIMD_HALF_PI;
					euler_out2.pitch = btScalar.SIMD_HALF_PI;
					euler_out.roll = euler_out.pitch + delta;
					euler_out2.roll = euler_out.pitch + delta;
				}
				else // gimbal locked down
				{
					euler_out.pitch = -btScalar.SIMD_HALF_PI;
					euler_out2.pitch = -btScalar.SIMD_HALF_PI;
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
			float det = col1.dot( ref tmp );
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

		public float tdotx( ref btVector3 v )
		{
			return m_el0.x * v.x + m_el1.x * v.y + m_el2.x * v.z;
		}
		public float tdoty( ref btVector3 v )
		{
			return m_el0.y * v.x + m_el1.y * v.y + m_el2.y * v.z;
		}
		public float tdotz( ref btVector3 v )
		{
			return m_el0.z * v.x + m_el1.z * v.y + m_el2.z * v.z;
		}




		/*@brief Calculate the matrix cofactor 
		 @param r1 The first row to use for calculating the cofactor
		 @param c1 The first column to use for calculating the cofactor
		 @param r1 The second row to use for calculating the cofactor
		 @param c1 The second column to use for calculating the cofactor
		 See http://en.wikipedia.org/wiki/Cofactor_(linear_algebra) for more details
*/
		float cofac( int r1, int c1, int r2, int c2 )
		{
			return this[r1, c1] * this[r2, c2] - this[r1, c2] * this[r2, c1];
		}


		public static void Mult( ref btMatrix3x3 m, float k, out btMatrix3x3 result )
		{
			m.m_el0.Mult( k, out result.m_el0 );
			m.m_el1.Mult( k, out result.m_el1 );
			m.m_el2.Mult( k, out result.m_el2 );
			m.m_el3.Mult( k, out result.m_el3 );
		}
		public void Mult( float k, out btMatrix3x3 result )
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



		public float determinant()
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
					float det = m_el0.dot( ref co );
#if PARANOID_ASSERTS
					Debug.Assert( det != 0 );
#endif
					float s = 1.0 / det;
					return new btMatrix3x3( co.x * s, cofac( 0, 2, 2, 1 ) * s, cofac( 0, 1, 1, 2 ) * s,
							co.y * s, cofac( 0, 0, 2, 2 ) * s, cofac( 0, 2, 1, 0 ) * s,
							co.z * s, cofac( 0, 1, 2, 0 ) * s, cofac( 0, 0, 1, 1 ) * s );
				}
				*/
		public void inverse( out btMatrix3x3 result )
		{
			btVector3 co = new btVector3( cofac( 1, 1, 2, 2 ), cofac( 1, 2, 2, 0 ), cofac( 1, 0, 2, 1 ) );
			float det = m_el0.dot( ref co );
#if PARANOID_ASSERTS
			Debug.Assert( det != 0 );
#endif
			float s = btScalar.BT_ONE / det;
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
		public void ApplyRotation( ref btVector3 m, out btVector3 result )
		{
#if COLUMN_MAJOR_EXPECTED
			result.x = this.m_el0.x * m.x +
					   this.m_el0.y * m.y +
					   this.m_el0.z * m.z;
			result.y = this.m_el1.x * m.x +
					   this.m_el1.y * m.y +
					   this.m_el1.z * m.z;
			result.z = this.m_el2.x * m.x +
					   this.m_el2.y * m.y +
					   this.m_el2.z * m.z;
#else
			result.x = this.m_el0.x * m.x +
					   this.m_el1.x * m.y +
					   this.m_el2.x * m.z;
			result.y = this.m_el0.y * m.x +
					   this.m_el1.y * m.y +
					   this.m_el2.y * m.z;
			result.z = this.m_el0.z * m.x +
					   this.m_el1.z * m.y +
					   this.m_el2.z * m.z;
#endif
			result.w = 0;
		}

		public void ApplyInverseRotation( ref btVector3 m, out btVector3 result )
		{
#if COLUMN_MAJOR_EXPECTED
			result.x = this.m_el0.x * m.x +
					   this.m_el1.x * m.y +
					   this.m_el2.x * m.z;
			result.y = this.m_el0.y * m.x +
					   this.m_el1.y * m.y +
					   this.m_el2.y * m.z;
			result.z = this.m_el0.z * m.x +
					   this.m_el1.z * m.y +
					   this.m_el2.z * m.z;
#else
			result.x = this.m_el0.x * m.x +
					   this.m_el0.y * m.y +
					   this.m_el0.z * m.z;
			result.y = this.m_el1.x * m.x +
					   this.m_el1.y * m.y +
					   this.m_el1.z * m.z;
			result.z = this.m_el2.x * m.x +
					   this.m_el2.y * m.y +
					   this.m_el2.z * m.z;
#endif
			result.w = 0;
		}

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

#if false
		void GetGLMatrix( out btMatrix3x3 m )
		{
			m = this;
			btVector3 tmp;
			ApplyInverseRotation( ref m_el3, out tmp );
			m.m_el3.x = tmp.x;
			m.m_el3.y = tmp.y;
			m.m_el3.z = tmp.z;
			m.m_el3.w = this[3][3];
		}
#endif

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
		public override string ToString()
		{
			btVector3 v1, v2, v3, v4;
			getColumn( 0, out v1 );
			getColumn( 1, out v2 );
			getColumn( 2, out v3 );
			getColumn( 3, out v4 );
			return v1 + "  " + v2 + "  " + v3 + "  " + v4;
			//return m_el0 + "  " + m_el1 + "  " + m_el2 + "  " + m_el3;
		}
	}
}
