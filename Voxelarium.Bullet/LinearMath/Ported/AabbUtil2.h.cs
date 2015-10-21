#define USE_BANCHLESS
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

using System.Diagnostics;

namespace Bullet.LinearMath
{

	public class btAabbUtil
	{

		public static void AabbExpand( ref btVector3 aabbMin,
										   ref btVector3 aabbMax,
										   ref btVector3 expansionMin,
										   ref btVector3 expansionMax )
		{
			aabbMin.Add( ref expansionMin, out aabbMin );
			aabbMin.Add( ref expansionMax, out aabbMin );
		}

		/// conservative test for overlap between two aabbs
		public static bool TestPointAgainstAabb2( ref btVector3 aabbMin1, ref btVector3 aabbMax1,
										ref btVector3 point )
		{
			bool overlap = true;
			overlap = ( aabbMin1.x > point.x || aabbMax1.x < point.x ) ? false : overlap;
			overlap = ( aabbMin1.z > point.z || aabbMax1.z < point.z ) ? false : overlap;
			overlap = ( aabbMin1.y > point.y || aabbMax1.y < point.y ) ? false : overlap;
			return overlap;
		}


		/// conservative test for overlap between two aabbs
		public static bool TestAabbAgainstAabb2( ref btVector3 aabbMin1, ref btVector3 aabbMax1,
										ref btVector3 aabbMin2, ref btVector3 aabbMax2 )
		{
			bool overlap = true;
			overlap = ( aabbMin1.x > aabbMax2.x || aabbMax1.x < aabbMin2.x ) ? false : overlap;
			overlap = ( aabbMin1.z > aabbMax2.z || aabbMax1.z < aabbMin2.z ) ? false : overlap;
			overlap = ( aabbMin1.y > aabbMax2.y || aabbMax1.y < aabbMin2.y ) ? false : overlap;
			return overlap;
		}

		/// conservative test for overlap between triangle and aabb
		public static bool TestTriangleAgainstAabb2( btVector3[] vertices,
											btIVector3 aabbMin, btIVector3 aabbMax )
		{
			btVector3 p1 = vertices[0];
			btVector3 p2 = vertices[1];
			btVector3 p3 = vertices[2];

			if( btScalar.btMin( btScalar.btMin( p1[0], p2[0] ), p3[0] ) > aabbMax[0] ) return false;
			if( btScalar.btMax( btScalar.btMax( p1[0], p2[0] ), p3[0] ) < aabbMin[0] ) return false;

			if( btScalar.btMin( btScalar.btMin( p1[2], p2[2] ), p3[2] ) > aabbMax[2] ) return false;
			if( btScalar.btMax( btScalar.btMax( p1[2], p2[2] ), p3[2] ) < aabbMin[2] ) return false;

			if( btScalar.btMin( btScalar.btMin( p1[1], p2[1] ), p3[1] ) > aabbMax[1] ) return false;
			if( btScalar.btMax( btScalar.btMax( p1[1], p2[1] ), p3[1] ) < aabbMin[1] ) return false;
			return true;
		}


		public static int btOutcode( ref btVector3 p, ref btVector3 halfExtent )
		{
			return ( p.x < -halfExtent.x ? 0x01 : 0x0 ) |
				   ( p.x > halfExtent.x ? 0x08 : 0x0 ) |
				   ( p.y < -halfExtent.y ? 0x02 : 0x0 ) |
				   ( p.y > halfExtent.y ? 0x10 : 0x0 ) |
				   ( p.z < -halfExtent.z ? 0x4 : 0x0 ) |
				   ( p.z > halfExtent.z ? 0x20 : 0x0 );
		}



		public static bool btRayAabb2( ref btVector3 rayFrom,
										  ref btVector3 rayInvDirection,
										  uint[] raySign,
										  btVector3[] bounds,
										  out double tmin,
										  double lambda_min,
										  double lambda_max )
		{
			double tmax, tymin, tymax, tzmin, tzmax;
			tmin = ( bounds[raySign[0]].x - rayFrom.x ) * rayInvDirection.x;
			tmax = ( bounds[1 - raySign[0]].x - rayFrom.x ) * rayInvDirection.x;
			tymin = ( bounds[raySign[1]].y - rayFrom.y ) * rayInvDirection.y;
			tymax = ( bounds[1 - raySign[1]].y - rayFrom.y ) * rayInvDirection.y;

			if( ( tmin > tymax ) || ( tymin > tmax ) )
				return false;

			if( tymin > tmin )
				tmin = tymin;

			if( tymax < tmax )
				tmax = tymax;

			tzmin = ( bounds[raySign[2]].z - rayFrom.z ) * rayInvDirection.z;
			tzmax = ( bounds[1 - raySign[2]].z - rayFrom.z ) * rayInvDirection.z;

			if( ( tmin > tzmax ) || ( tzmin > tmax ) )
				return false;
			if( tzmin > tmin )
				tmin = tzmin;
			if( tzmax < tmax )
				tmax = tzmax;
			return ( ( tmin < lambda_max ) && ( tmax > lambda_min ) );
		}

		public static bool btRayAabb( ref btVector3 rayFrom,
										 ref btVector3 rayTo,
										 ref btVector3 aabbMin,
										 ref btVector3 aabbMax,
							  double param, ref btVector3 normal )
		{
			btVector3 aabbHalfExtent; aabbMax.SubScale( ref aabbMin ,  (double)( 0.5 ), out aabbHalfExtent );
			btVector3 aabbCenter;  aabbMax.AddScale( ref aabbMin, (double)( 0.5 ), out aabbCenter );
			btVector3 source;  rayFrom.Sub( ref aabbCenter, out source );
			btVector3 target;  rayTo.Sub( ref aabbCenter, out target );
			int sourceOutcode = btOutcode( ref source, ref aabbHalfExtent );
			int targetOutcode = btOutcode( ref target, ref aabbHalfExtent );
			if( ( sourceOutcode & targetOutcode ) == 0x0 )
			{
				double lambda_enter = btScalar.BT_ZERO;
				double lambda_exit = param;
				btVector3 r; target.Sub( ref source, out r );
				int i;
				double normSign = 1;
				btVector3 hitNormal = btVector3.Zero;
				int bit = 1;

				for( int j = 0; j < 2; j++ )
				{
					for( i = 0; i != 3; ++i )
					{
						if( ( sourceOutcode & bit ) != 0 )
						{
							double lambda = ( -source[i] - aabbHalfExtent[i] * normSign ) / r[i];
							if( lambda_enter <= lambda )
							{
								lambda_enter = lambda;
								hitNormal.setValue( 0, 0, 0 );
								hitNormal[i] = normSign;
							}
						}
						else if( ( targetOutcode & bit ) != 0  )
						{
							double lambda = ( -source[i] - aabbHalfExtent[i] * normSign ) / r[i];
							btScalar.btSetMin( ref lambda_exit, lambda );
						}
						bit <<= 1;
					}
					normSign = (double)( -1.0);
				}
				if( lambda_enter <= lambda_exit )
				{
					param = lambda_enter;
					normal = hitNormal;
					return true;
				}
			}
			return false;
		}



		public static void btTransformAabb( ref btVector3 halfExtents, double margin, ref btTransform t
				, out btVector3 aabbMinOut, out btVector3 aabbMaxOut )
		{
			btVector3 halfExtentsWithMargin; halfExtents.AddScale( ref btVector3.One, margin, out halfExtentsWithMargin  );
			btMatrix3x3 abs_b; t.getBasis().absolute( out abs_b);
			btIVector3 center = t.getOrigin();
			btVector3 extent; halfExtentsWithMargin.dot3( ref abs_b.m_el0, ref abs_b.m_el1, ref abs_b.m_el2, out extent );
			center.Sub(ref extent, out aabbMinOut );
			center.Add( ref extent, out aabbMaxOut );
		}


		public static void btTransformAabb( ref btVector3 localAabbMin, ref btVector3 localAabbMax, double margin, ref btTransform trans, out btVector3 aabbMinOut, out btVector3 aabbMaxOut )
		{
			Debug.Assert( localAabbMin.x <= localAabbMax.x );
			Debug.Assert( localAabbMin.y <= localAabbMax.y );
			Debug.Assert( localAabbMin.z <= localAabbMax.z );
			btVector3 tmp;
			localAabbMax.Sub( ref localAabbMin, out tmp );
			btVector3 localHalfExtents; tmp.Mult( (double)( 0.5 ), out localHalfExtents );
			btVector3 localCenter = localHalfExtents;
			btVector3.Zero.Mult( margin, out tmp );
			localHalfExtents.Add( ref tmp, out localHalfExtents );
			//localHalfExtents += btVector3( margin, margin, margin );

			btMatrix3x3 abs_b; trans.m_basis.absolute( out abs_b );
			btVector3 center; trans.Apply( ref localCenter, out center );
			btVector3 extent; localHalfExtents.dot3( ref abs_b.m_el0, ref abs_b.m_el1, ref abs_b.m_el2, out extent );
			center.Sub( ref extent, out aabbMinOut );
			center.Add(ref  extent, out aabbMaxOut );
		}

#if USE_BANCHLESS
		//This block replaces the block below and uses no branches, and replaces the 8 bit return with a 32 bit return for improved performance (~3x on XBox 360)
		public static uint testQuantizedAabbAgainstQuantizedAabb( ushort[] aabbMin1, ushort[] aabbMax1
				, ushort[] aabbMin2, ushort[] aabbMax2 )
		{
			return ( ( aabbMin1[0] <= aabbMax2[0] ) & ( aabbMax1[0] >= aabbMin2[0] )
				& ( aabbMin1[2] <= aabbMax2[2] ) & ( aabbMax1[2] >= aabbMin2[2] )
				& ( aabbMin1[1] <= aabbMax2[1] ) & ( aabbMax1[1] >= aabbMin2[1] ) )?
				(uint)1: (uint)0 ;
			//return (uint)( btScalar.btSelect( (uint)( ( aabbMin1[0] <= aabbMax2[0] )  ( aabbMax1 >= aabbMin2[0] )
		//		& ( aabbMin1[2] <= aabbMax2[2] ) & ( aabbMax1[2] >= aabbMin2[2] )
		//		& ( aabbMin1[1] <= aabbMax2[1] ) & ( aabbMax1[1] >= aabbMin2[1] ) ),
			//	1, 0 ) );
		}
#else
	public static bool testQuantizedAabbAgainstQuantizedAabb(string nsigned short int* aabbMin1,string nsigned short int* aabbMax1,string nsigned short int* aabbMin2,string nsigned short int* aabbMax2)
	{
		bool overlap = true;
		overlap = (aabbMin1[0] > aabbMax2[0] || aabbMax1[0] < aabbMin2[0]) ? false : overlap;
		overlap = (aabbMin1[2] > aabbMax2[2] || aabbMax1[2] < aabbMin2[2]) ? false : overlap;
		overlap = (aabbMin1[1] > aabbMax2[1] || aabbMax1[1] < aabbMin2[1]) ? false : overlap;
		return overlap;
	}
#endif //USE_BANCHLESS



	}
}