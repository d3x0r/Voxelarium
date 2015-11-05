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
	# ifdef BT_USE_DOUBLE_PRECISION
	#define btQuaternionData btQuaternionDoubleData
	#define btQuaternionDataName "btQuaternionDoubleData"
	#else
	#define btQuaternionData btQuaternionFloatData
	#define btQuaternionDataName "btQuaternionFloatData"
	#endif //BT_USE_DOUBLE_PRECISION
	*/

	/*@brief The btQuaternion implements quaternion to perform linear algebra rotations in combination with btMatrix3x3, btVector3 and btTransform. */
	public struct btQuaternion
	{
		public static btQuaternion Zero = new btQuaternion( 0, 0, 0, 1 );

		public double x, y, z, w;
		/*@brief No initialization constructor */
		//		template <typename double>
		//		explicit Quaternion(stringbtScalar v) : Tuple4<double>(v) {}
		/*@brief Constructor from scalars */
		public btQuaternion( double _x, double _y, double _z, double _w )
		{ x = _x; y = _y; z = _z; w = _w; }

		/*@brief Axis angle Constructor
		  @param axis The axis which the rotation is around
		  @param angle The magnitude of the rotation around the angle (Radians) */
		public btQuaternion( ref btVector3 _axis, double _angle )
		{
			//setRotation( _axis, _angle );
			double d = _axis.length();
			if( d != 0.0 )
			{
				double s = btScalar.btSin( _angle * 0.5 ) / d;
				x = _axis.x * s;
				y = _axis.y * s;
				z = _axis.z * s;
				w = btScalar.btCos( _angle * 0.5 );
			}
			else
				x = y = z = w = 0;
		}
		/*@brief Constructor from Euler angles
		  @param yaw Angle around Y unless BT_EULER_DEFAULT_ZYX defined then Z
		  @param pitch Angle around X unless BT_EULER_DEFAULT_ZYX defined then Y
		  @param roll Angle around Z unless BT_EULER_DEFAULT_ZYX defined then X */
		btQuaternion( double yaw, double pitch, double roll )
		{
			double halfYaw = yaw * 0.5;
			double halfPitch = pitch * 0.5;
			double halfRoll = roll * 0.5;
			double cosYaw = btScalar.btCos( halfYaw );
			double sinYaw = btScalar.btSin( halfYaw );
			double cosPitch = btScalar.btCos( halfPitch );
			double sinPitch = btScalar.btSin( halfPitch );
			double cosRoll = btScalar.btCos( halfRoll );
			double sinRoll = btScalar.btSin( halfRoll );
			x = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
			y = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
			z = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
			w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
		}
		/*@brief Set the rotation using axis angle notation 
		  @param axis The axis around which to rotate
		  @param angle The magnitude of the rotation in Radians */
		public void setRotation( ref btVector3 axis, double _angle )
		{
			double d = axis.length();
			if( d != 0.0 )
			{
				double s = btScalar.btSin( _angle * 0.5 ) / d;
				setValue( axis.x * s, axis.y * s, axis.z * s,
					btScalar.btCos( _angle * 0.5 ) );
			}
			else
				x = y = z = w = 0;
		}

		public void setValue( double _x, double _y, double _z, double _w )
		{
			x = _x; y = _y; z = _z; w = _w;
		}
		/*@brief Set the quaternion using Euler angles
		  @param yaw Angle around Y
		  @param pitch Angle around X
		  @param roll Angle around Z */
		void setEuler( double yaw, double pitch, double roll )
		{
			double halfYaw = yaw * 0.5;
			double halfPitch = pitch * 0.5;
			double halfRoll = roll * 0.5;
			double cosYaw = btScalar.btCos( halfYaw );
			double sinYaw = btScalar.btSin( halfYaw );
			double cosPitch = btScalar.btCos( halfPitch );
			double sinPitch = btScalar.btSin( halfPitch );
			double cosRoll = btScalar.btCos( halfRoll );
			double sinRoll = btScalar.btSin( halfRoll );
			setValue( cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,
				cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,
				sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,
				cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw );
		}
		/*@brief Set the quaternion using euler angles 
		  @param yaw Angle around Z
		  @param pitch Angle around Y
		  @param roll Angle around X */
		void setEulerZYX( double yaw, double pitch, double roll )
		{
			double halfYaw = yaw * 0.5;
			double halfPitch = pitch * 0.5;
			double halfRoll = roll * 0.5;
			double cosYaw = btScalar.btCos( halfYaw );
			double sinYaw = btScalar.btSin( halfYaw );
			double cosPitch = btScalar.btCos( halfPitch );
			double sinPitch = btScalar.btSin( halfPitch );
			double cosRoll = btScalar.btCos( halfRoll );
			double sinRoll = btScalar.btSin( halfRoll );
			setValue( sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw, //x
							 cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw, //y
							 cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw, //z
							 cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw ); //formerly yzx
		}
		/*@brief Add two quaternions
		  @param q The quaternion to add to this one */
		public static void Add( ref btQuaternion q1, ref btQuaternion q2, out btQuaternion result )
		{
			result.x = q1.x + q2.x;
			result.y = q1.y + q2.y;
			result.z = q1.z + q2.z;
			result.w = q1.w + q2.w;
		}
		/*@brief Add two quaternions
		  @param q The quaternion to add to this one */
		public void Add( ref btQuaternion q2, out btQuaternion result )
		{
			result.x = x + q2.x;
			result.y = y + q2.y;
			result.z = z + q2.z;
			result.w = w + q2.w;
		}

		/*@brief Subtract out a quaternion
		  @param q The quaternion to subtract from this one */
		public static void Sub( ref btQuaternion q1, ref btQuaternion q2, out btQuaternion result )
		{
			result.x = q1.x - q2.x;
			result.y = q1.y - q2.y;
			result.z = q1.z - q2.z;
			result.w = q1.w - q2.w;
		}

		/*@brief Subtract out a quaternion
		  @param q The quaternion to subtract from this one */
		public void Sub( ref btQuaternion q2, out btQuaternion result )
		{
			result.x = x - q2.x;
			result.y = y - q2.y;
			result.z = z - q2.z;
			result.w = w - q2.w;
		}

		/*@brief Scale this quaternion
		  @param s The scalar to scale by */
		public static void Mult( ref btQuaternion q, double s, out btQuaternion result )
		{
			result.x = q.x * s;
			result.y = q.y * s;
			result.z = q.z * s;
			result.w = q.w * s;
		}
		public static void Mult( ref btQuaternion q, double s, out btVector3 result )
		{
			result.x = q.x * s;
			result.y = q.y * s;
			result.z = q.z * s;
			result.w = q.w * s;
		}
		public void Mult( double s, out btVector3 result )
		{
			result.x = x * s;
			result.y = y * s;
			result.z = z * s;
			result.w = w * s;
		}

		/*@brief Multiply this quaternion by q on the right
		  @param q The other quaternion 
		  Equivilant to this = this * q */
		public static void Mult( ref btQuaternion q1, ref btQuaternion q2, out btQuaternion result )
		{
			result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
			result.y = q1.w * q2.y + q1.y * q2.w + q1.z * q2.x - q1.x * q2.z;
			result.z = q1.w * q2.z + q1.z * q2.w + q1.x * q2.y - q1.y * q2.x;
			result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
		}
		public void Mult( ref btQuaternion q2, out btVector3 result )
		{
			result.x = w * q2.x + x * q2.w + y * q2.z - z * q2.y;
			result.y = w * q2.y + y * q2.w + z * q2.x - x * q2.z;
			result.z = w * q2.z + z * q2.w + x * q2.y - y * q2.x;
			result.w = 0;// w * q2.w - x * q2.x - y * q2.y - z * q2.z;
		}

		public void Mult( ref btQuaternion q2, out btQuaternion result )
		{
			result.x = w * q2.x + x * q2.w + y * q2.z - z * q2.y;
			result.y = w * q2.y + y * q2.w + z * q2.x - x * q2.z;
			result.z = w * q2.z + z * q2.w + x * q2.y - y * q2.x;
			result.w = w * q2.w - x * q2.x - y * q2.y - z * q2.z;
		}

		public static void Mult( ref btQuaternion q, ref btVector3 w, out btQuaternion result )
		{
			result.x = q.w * w.x + q.y * w.z - q.z * w.y;
			result.y = q.w * w.y + q.z * w.x - q.x * w.z;
			result.z = q.w * w.z + q.x * w.y - q.y * w.x;
			result.w = -q.x * w.x - q.y * w.y - q.z * w.z;
		}
		public void Mult( ref btVector3 w, out btQuaternion result )
		{
			result.x = this.w * w.x + y * w.z - z * w.y;
			result.y = this.w * w.y + z * w.x - x * w.z;
			result.z = this.w * w.z + x * w.y - y * w.x;
			result.w = -x * w.x - y * w.y - z * w.z;
		}

		/*@brief Return the dot product between this quaternion and another
		  @param q The other quaternion */
		public double dot( ref btQuaternion q )
		{
			return x * q.x +
					y * q.y +
					z * q.z +
					w * q.w;
		}

		/*@brief Return the length squared of the quaternion */
		public double length2()
		{
			return dot( ref this );
		}

		/*@brief Return the length of the quaternion */
		public double length()
		{
			return btScalar.btSqrt( length2() );
		}

		/*@brief Normalize the quaternion 
		  Such that x^2 + y^2 + z^2 +w^2 = 1 */
		public void normalize()
		{
			Div( ref this, length(), out this );
		}

		/*@brief Return a scaled version of this quaternion
		  @param s The scale factor */
		public void Mult( double s, out btQuaternion result )
		{
			result.x = x * s;
			result.y = y * s;
			result.z = z * s;
			result.w = w * s;
		}

		/*@brief Return an inversely scaled versionof this quaternion
		  @param s The inverse scale factor */
		public static void Div( ref btQuaternion q, double s, out btQuaternion result )
		{
			//Debug.Assert( s != 0.0 );
			q.Mult( ( 1.0 / s ), out result );
		}

		/*@brief Return a normalized version of this quaternion */
		void normalized( out btQuaternion result )
		{
			Div( ref this, length(), out result );
		}
		/*@brief Return the []half[] angle between this quaternion and the other
		 @param q The other quaternion */
		double angle( ref btQuaternion q )
		{
			double s = btScalar.btSqrt( length2() * q.length2() );
			//Debug.Assert( s != 0.0 );
			return btScalar.btAcos( dot( ref q ) / s );
		}

		/*@brief Return the angle between this quaternion and the other along the shortest path
		 @param q The other quaternion */
		public double angleShortestPath( ref btQuaternion q )
		{
			double s = btScalar.btSqrt( length2() * q.length2() );
			//Debug.Assert( s != 0.0 );
			if( dot( ref q ) < 0 ) // Take care of long angle case see http://en.wikipedia.org/wiki/Slerp
			{
				btQuaternion b;
				q.inverse( out b );
				return btScalar.btAcos( dot( ref b ) / s ) * 2.0;
			}
			else
				return btScalar.btAcos( dot( ref q ) / s ) * 2.0;
		}

		/*@brief Return the angle of rotation represented by this quaternion */
		public double getAngle()
		{
			double s = 2 * btScalar.btAcos( w );
			return s;
		}

		/*@brief Return the angle of rotation represented by this quaternion along the shortest path*/
		public double getAngleShortestPath()
		{
			double s;
			if( dot( ref this ) < 0 )
				s = 2.0 * btScalar.btAcos( w );
			else
				s = 2.0 * btScalar.btAcos( -w );

			return s;
		}


		/*@brief Return the axis of the rotation represented by this quaternion */
		public void getAxis( out btVector3 result )
		{
			double s_squared = 1.0 - w * w;

			if( s_squared < 10 * btScalar.SIMD_EPSILON ) //Check for divide by zero
			{
				result.x = 1.0; result.y = 0; result.z = 0; result.w = 0;  // Arbitrary
				return;
			}
			double s = 1.0 / btScalar.btSqrt( s_squared );
			Mult( s, out result );
		}

		/*@brief Return the inverse of this quaternion */
		public void inverse( out btQuaternion result )
		{
			result.x = -x;
			result.y = -y;
			result.z = -z;
			result.w = w;
		}

		/*@brief Return the negative of this quaternion 
		 * This simply negates each element */
		public void Sub( btQuaternion q, out btQuaternion result )
		{
			result.x = x - q.x;
			result.y = y - q.y;
			result.z = z - q.z;
			result.w = w - q.w;
		}
		/*@todo document this and it's use */
		public void farthest( ref btQuaternion qd, out btQuaternion result )
		{
			btQuaternion diff, sum;
			this.Sub( ref qd, out diff );
			this.Add( ref qd, out sum );
			if( diff.dot( ref diff ) > sum.dot( ref sum ) )
				result = qd;
			qd.inverse( out result );
		}

		/*@todo document this and it's use */
		public void nearest( ref btQuaternion qd, out btQuaternion result )
		{
			btQuaternion diff, sum;
			this.Sub( ref qd, out diff );
			this.Add( ref qd, out sum );
			if( diff.dot( ref diff ) < sum.dot( ref sum ) )
				result = qd;
			qd.inverse( out result );
		}


		/*@brief Return the quaternion which is the result of Spherical Linear Interpolation between this and the other quaternion
		 * @param q The other quaternion to interpolate with 
		 * @param t The ratio between this and q to interpolate. * If t = 0 the result is this, if t=1 the result is q.
		 * Slerp interpolates assuming constant velocity.  */
		void slerp( ref btQuaternion q, double t, out btQuaternion result )
		{
			double magnitude = btScalar.btSqrt( length2() * q.length2() );
			//Debug.Assert( magnitude > (double)( 0 ) );

			double product = dot( ref q ) / magnitude;
			if( btScalar.btFabs( product ) < 1 )
			{
				// Take care of long angle case see http://en.wikipedia.org/wiki/Slerp
				double sign = ( product < 0 ) ? -1 : 1;

				double theta = btScalar.btAcos( sign * product );
				double s1 = btScalar.btSin( sign * t * theta );
				double d = 1.0 / btScalar.btSin( theta );
				double s0 = btScalar.btSin( ( 1.0 - t ) * theta );
				result.x = ( x * s0 + q.x * s1 ) * d;
				result.y = ( y * s0 + q.y * s1 ) * d;
				result.z = ( z * s0 + q.z * s1 ) * d;
				result.w = ( w * s0 + q.w * s1 ) * d;
			}
			else
			{
				result = this;
			}
		}

		public static btQuaternion Identity = new btQuaternion( 0, 0, 0, 1 );

		//public double getw { return w; }
		/*
		public void serialize(struct btQuaternionData& dataOut);

public void deSerialize(stringstruct btQuaternionData& dataIn);

	public void serializeFloat(struct btQuaternionFloatData& dataOut);

public void deSerializeFloat(stringstruct btQuaternionFloatData& dataIn);

	public void serializeDouble(struct btQuaternionDoubleData& dataOut);

public void deSerializeDouble(stringstruct btQuaternionDoubleData& dataIn);
*/

		public static void Apply( ref btQuaternion q, ref btVector3 w, out btQuaternion result )
		{
			result.x = q.w * w.x + q.y * w.z - q.z * w.y;
			result.y = q.w * w.y + q.z * w.x - q.x * w.z;
			result.z = q.w * w.z + q.x * w.y - q.y * w.x;
			result.w = -q.x * w.x - q.y * w.y - q.z * w.z;
		}

		public static void Apply( ref btVector3 w, ref btQuaternion q, out btQuaternion result )
		{
			result.x = +w.x * q.w + w.y * q.z - w.z * q.y;
			result.y = +w.y * q.w + w.z * q.x - w.x * q.z;
			result.z = +w.z * q.w + w.x * q.y - w.y * q.x;
			result.w = -w.x * q.x - w.y * q.y - w.z * q.z;
		}

		/*@brief Calculate the dot product between two quaternions */
		public static double dot( ref btQuaternion q1, ref btQuaternion q2 )
		{
			return q1.dot( ref q2 );
		}


		/*@brief Return the length of a quaternion */
		public double length( ref btQuaternion q )
		{
			return q.length();
		}

		/*@brief Return the angle between two quaternions*/
		public static double btAngle( ref btQuaternion q1, ref btQuaternion q2 )
		{
			return q1.angle( ref q2 );
		}

		/*@brief Return the inverse of a quaternion*/
		public static void inverse( ref btQuaternion q, out btQuaternion result )
		{
			q.inverse( out result );
		}


		/*@brief Return the result of spherical linear interpolation betwen two quaternions 
		  @param q1 The first quaternion
		  @param q2 The second quaternion 
		  @param t The ration between q1 and q2.  t = 0 return q1, t=1 returns q2 
		  Slerp assumes constant velocity between positions. */
		public void slerp( ref btQuaternion q1, ref btQuaternion q2, double t, out btQuaternion result )
		{
			q1.slerp( ref q2, t, out result );
		}

		public void Rotate( ref btVector3 v, out btVector3 result )
		{
			btQuaternion q;
			btQuaternion tmp;
			Mult( ref v, out q );
			inverse( out tmp );
			q.Mult( ref tmp, out result );
		}

		public static void quatRotate( ref btQuaternion rotation, ref btVector3 v, out btVector3 result )
		{
			btQuaternion q;
			btQuaternion tmp;
			rotation.Mult( ref v, out q );
			rotation.inverse( out tmp );
			q.Mult( ref tmp, out result );
		}

		public static void shortestArcQuat( ref btVector3 v0, ref btVector3 v1, out btQuaternion result ) // Game Programming Gems 2.10 make sure v0,v1 are normalized
		{
			btVector3 c;
			v0.cross( ref v1, out c );
			double d = v0.dot( ref v1 );

			if( d < -1.0 + btScalar.SIMD_EPSILON )
			{
				btVector3 n, unused;
				btVector3.btPlaneSpace1( ref v0, out n, out unused );
				result.x = n.x; result.y = n.y; result.z = n.z; result.w = 0.0f; // just pick any vector that is orthogonal to v0
				return;
			}

			double s = btScalar.btSqrt( ( 1.0f + d ) * 2.0f );
			double rs = 1.0f / s;
			result.x = c.x * rs;
			result.y = c.y * rs;
			result.z = c.z * rs;
			result.w = s * 0.5f;
		}

		public void shortestArcQuatNormalize2( ref btVector3 v0, ref btVector3 v1, out btQuaternion result )
		{
			btVector3 _v0;
			btVector3 _v1;
			v0.normalized( out _v0 );
			v1.normalized( out _v1 );
			shortestArcQuat( ref _v0, ref _v1, out result );
		}

		public bool Equals( ref btQuaternion q )
		{
			return ( q.x == x ) && ( q.y == y ) && ( q.z == z ) && ( q.w == w );
		}

		/*
				struct btQuaternionFloatData
				{
					float m_floats[4];
				};

				struct btQuaternionDoubleData
				{
					double m_floats[4];

				};
				*/
		/*
		public void btQuaternion::serializeFloat(struct btQuaternionFloatData& dataOut)
{
	///could also do a memcpy, check if it is worth it
	for (int i = 0; i<4;i++)
		dataOut.m_floats[i] = float(m_floats[i]);
}

	public void btQuaternion::deSerializeFloat(stringstruct btQuaternionFloatData& dataIn)
{
	for (int i = 0; i<4;i++)
		m_floats[i] = (double)( dataIn.m_floats[i]);
}


public void btQuaternion::serializeDouble(struct btQuaternionDoubleData& dataOut)
{
	///could also do a memcpy, check if it is worth it
	for (int i = 0; i<4;i++)
		dataOut.m_floats[i] = double(m_floats[i]);
}

public void btQuaternion::deSerializeDouble(stringstruct btQuaternionDoubleData& dataIn)
{
	for (int i = 0; i<4;i++)
		m_floats[i] = (double)( dataIn.m_floats[i]);
}


public void btQuaternion::serialize(struct btQuaternionData& dataOut)
{
	///could also do a memcpy, check if it is worth it
	for (int i = 0; i<4;i++)
		dataOut.m_floats[i] = m_floats[i];
}

public void btQuaternion::deSerialize(stringstruct btQuaternionData& dataIn)
{
	for (int i = 0; i<4;i++)
		m_floats[i] = dataIn.m_floats[i];
}

	*/
	}

}