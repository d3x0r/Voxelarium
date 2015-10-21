/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2011 Advanced Micro Devices, Inc.  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

using System;
using System.Collections.Generic;
using Bullet.Types;

namespace Bullet.LinearMath
{

	public class GrahamVector3
	{
		btVector3 v;
		public double m_angle;
		public short m_orgIndex;

		public GrahamVector3( ref btVector3 org, int orgIndex )
		{
			v = org;
			m_orgIndex = orgIndex;
		}

		public struct btAngleCompareFunc : IComparer<GrahamVector3>
		{
			public btVector3 m_anchor;
			public btAngleCompareFunc( ref btVector3 anchor )
			{
				m_anchor = ( anchor );
			}

			int IComparer<GrahamVector3>.Compare( GrahamVector3 a, GrahamVector3 b )
			{
				if( a.m_angle != b.m_angle )
					return ( a.m_angle < b.m_angle ) ? -1 : ( a.m_angle > b.m_angle ) ? 1 : 0;
				else
				{
					btVector3 tmp;
					a.v.Sub( ref m_anchor, out tmp );
					double al = tmp.length2();
					b.v.Sub( ref m_anchor, out tmp );
					double bl = ( tmp ).length2();
					if( al != bl )
						return al < bl ? -1 : al < bl ? 1 : 0;
					else
					{
						return a.m_orgIndex < b.m_orgIndex ? -1 : a.m_orgIndex > b.m_orgIndex ? 1 : 0;
					}
				}
			}
		}

		public static void GrahamScanConvexHull2D( btList<GrahamVector3> originalPoints, btList<GrahamVector3> hull, ref btVector3 normalAxis )
		{
			btVector3 axis0, axis1;
			btVector3.btPlaneSpace1( ref normalAxis, out axis0, out axis1 );


			if( originalPoints.Count <= 1 )
			{
				for( int i = 0; i < originalPoints.Count; i++ )
					hull.Add( originalPoints[0] );
				return;
			}
			{
				//step1 : find anchor point with smallest projection on axis0 and move it to first location
				for( int i = 0; i < originalPoints.Count; i++ )
				{
					//		ref btVector3 left = originalPoints[i];
					//		ref btVector3 right = originalPoints[0];
					double projL = originalPoints[i].v.dot( ref axis0 );
					double projR = originalPoints[0].v.dot( ref axis0 );
					if( projL < projR )
					{
						originalPoints.Swap( 0, i );
					}
				}
			}

			//also precompute angles
			originalPoints[0].m_angle = -btScalar.BT_LARGE_FLOAT;
			GrahamVector3[] arr_originalPoints = originalPoints.InternalArray;
			{
				for( int i = 1; i < originalPoints.Count; i++ )
				{
					btVector3 ar; arr_originalPoints[i].v.Sub( ref arr_originalPoints[0].v, out ar );
					double ar1 = axis1.dot( ref ar );
					double ar0 = axis0.dot( ref ar );
					if( ar1 * ar1 + ar0 * ar0 < btScalar.SIMD_EPSILON )
					{
						originalPoints[i].m_angle = 0.0f;
					}
					else
					{
						originalPoints[i].m_angle = btScalar.btAtan2Fast( ar1, ar0 );
					}
				}
			}
			//step 2: sort all points, based on 'angle' with this anchor
			//btAngleCompareFunc comp( originalPoints[0] );
			originalPoints.Sort( 1, originalPoints.Count - 2, new btAngleCompareFunc() );
			{
				int i;
				for( i = 0; i < 2; i++ )
					hull.Add( originalPoints[i] );

				//step 3: keep all 'convex' points and discard concave points (using back tracking)
				for( ; i != originalPoints.Count; i++ )
				{
					bool isConvex = false;
					while( !isConvex && hull.Count > 1 )
					{
						btVector3 a = hull[hull.Count - 2].v;
						btVector3 b = hull[hull.Count - 1].v;
						btVector3 tmp, tmp2;
						a.Sub( ref b, out tmp );
						a.Sub( ref arr_originalPoints[i].v, out tmp2 );
						btVector3 tmp3;
						tmp.cross( ref tmp2, out tmp3 );
                        isConvex = tmp3.dot( ref normalAxis ) > 0;
						if( !isConvex )
							hull.Count--;
						else
							hull.Add( originalPoints[i] );
					}

					if( hull.Count == 1 )
					{
						hull.Add( originalPoints[i] );
					}
				}
			}
		}

	};




}

