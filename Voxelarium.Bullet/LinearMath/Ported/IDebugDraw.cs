/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

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



	///The btIDebugDraw interface class allows hooking up a debug renderer to visually debug simulations.
	///Typical use case: create a debug drawer object, and assign it to a btCollisionWorld or btDynamicsWorld using setDebugDrawer and call debugDrawWorld.
	///A class that implements the btIDebugDraw interface has to implement the drawLine method at a minimum.
	///For color arguments the X,Y,Z components refer to Red, Green and Blue each in the range [0..1]
	public abstract class btIDebugDraw
	{


		public struct DefaultColors
		{
			btVector3 m_activeObject;
			btVector3 m_deactivatedObject;
			btVector3 m_wantsDeactivationObject;
			btVector3 m_disabledDeactivationObject;
			btVector3 m_disabledSimulationObject;
			btVector3 m_aabb;
			btVector3 m_contactPoint;

			static public void Init( out DefaultColors c )
			{
				c.m_activeObject = new btVector3( 1, 1, 1 );
				c.m_deactivatedObject = new btVector3( 0, 1, 0 );
				c.m_wantsDeactivationObject = new btVector3( 0, 1, 1 );
				c.m_disabledDeactivationObject = new btVector3( 1, 0, 0 );
				c.m_disabledSimulationObject = new btVector3( 1, 1, 0 );
				c.m_aabb = new btVector3( 1, 0, 0 );
				c.m_contactPoint = new btVector3( 1, 1, 0 );
			}
		};


		enum DebugDrawModes
		{
			DBG_NoDebug = 0,
			DBG_DrawWireframe = 1,
			DBG_DrawAabb = 2,
			DBG_DrawFeaturesText = 4,
			DBG_DrawContactPoints = 8,
			DBG_NoDeactivation = 16,
			DBG_NoHelpText = 32,
			DBG_DrawText = 64,
			DBG_ProfileTimings = 128,
			DBG_EnableSatComparison = 256,
			DBG_DisableBulletLCP = 512,
			DBG_EnableCCD = 1024,
			DBG_DrawConstraints = ( 1 << 11 ),
			DBG_DrawConstraintLimits = ( 1 << 12 ),
			DBG_FastWireframe = ( 1 << 13 ),
			DBG_DrawNormals = ( 1 << 14 ),
			DBG_DrawFrames = ( 1 << 15 ),
			DBG_MAX_DEBUG_DRAW_MODE
		};


		public virtual DefaultColors getDefaultColors() { DefaultColors colors; DefaultColors.Init( out colors ); return colors; }
		///the default implementation for setDefaultColors has no effect. A derived class can implement it and store the colors.
		public virtual void setDefaultColors( DefaultColors c ) { }

		public abstract void drawLine( ref btVector3 from, ref btVector3 to, ref btVector3 color );

		public virtual void drawLine( ref btVector3 from, ref btVector3 to, ref btVector3 fromColor, ref btVector3 toColor )
		{
			drawLine( ref from, ref to, ref fromColor );
		}

		public virtual void drawSphere( double radius, ref btTransform transform, ref btVector3 color )
		{

			btVector3 center = transform.getOrigin();
			btVector3 up = transform.getBasis().getColumn( 1 );
			btVector3 axis = transform.getBasis().getColumn( 0 );
			double minTh = -btScalar.SIMD_HALF_PI;
			double maxTh = btScalar.SIMD_HALF_PI;
			double minPs = -btScalar.SIMD_HALF_PI;
			double maxPs = btScalar.SIMD_HALF_PI;
			double stepDegrees = 30;
			drawSpherePatch( ref center, ref up, ref axis, radius, minTh, maxTh, minPs, maxPs, ref color, stepDegrees, false );
			btVector3 tmp;
			axis.Invert( out tmp );
			drawSpherePatch( ref center, ref up, ref tmp, radius, minTh, maxTh, minPs, maxPs, ref color, stepDegrees, false );
		}

		public virtual void drawSphere( ref btVector3 p, double radius, ref btVector3 color )
		{
			btTransform tr = new btTransform( ref p );
			drawSphere( radius, ref tr, ref color );
		}

		public virtual void drawTriangle( ref btVector3 v0, ref btVector3 v1, ref btVector3 v2, ref btVector3 n0, ref btVector3 n1, ref btVector3 n2, ref btVector3 color, double alpha )
		{
			drawTriangle( ref v0, ref v1, ref v2, ref color, alpha );
		}
		public virtual void drawTriangle( ref btVector3 v0, ref btVector3 v1, ref btVector3 v2, ref btVector3 color, double alpha )
		{
			drawLine( ref v0, ref v1, ref color );
			drawLine( ref v1, ref v2, ref color );
			drawLine( ref v2, ref v0, ref color );
		}

		public abstract void drawContactPoint( ref btVector3 PointOnB, ref btVector3 normalOnB, double distance, int lifeTime, ref btVector3 color );

		public abstract void reportErrorWarning( string warningString );

		public abstract void draw3dText( ref btVector3 location, string textString );

		public abstract void setDebugMode( int debugMode );

		public abstract int getDebugMode();

		public virtual void drawAabb( ref btVector3 from, ref btVector3 to, ref btVector3 color )
		{

			btVector3 halfExtents; to.SubAndScale( ref from, 0.5f, out halfExtents );
			btVector3 center; to.AddAndScale( ref from, 0.5f, out center );
			int i, j;

			btVector3 edgecoord = new btVector3( 1f, 1f, 1f ), pa, pb;
			for( i = 0; i < 4; i++ )
			{
				for( j = 0; j < 3; j++ )
				{
					edgecoord.Mult( ref halfExtents, out pa );
					pa.Add( ref center, out pa );


					int othercoord = j % 3;
					edgecoord[othercoord] *= -1f;

					edgecoord.Mult( ref halfExtents, out pb );
					pb.Add( ref center, out pb );

					drawLine( ref pa, ref pb, ref color );
				}
				edgecoord.setValue( -1, -1, -1 );
				if( i < 3 )
					edgecoord[i] *= -1;
			}
		}
		public virtual void drawTransform( ref btTransform transform, double orthoLen )
		{
			btVector3 start = transform.getOrigin();
			btVector3 a;
			btVector3 b;
			btVector3.setValue( out b, orthoLen, 0, 0 );
			transform.Apply( ref b, out a );
			b.setValue( 0.7, 0, 0 );
			drawLine( ref start, ref a, ref b );
			btVector3.setValue( out b, 0, orthoLen, 0 );
			transform.Apply( ref b, out a );
			b.setValue( 0, 0.7, 0 );
			drawLine( ref start, ref a, ref b );
			btVector3.setValue( out b, 0, 0, orthoLen );
			transform.Apply( ref b, out a );
			b.setValue( 0, 0, 0.7 );
			drawLine( ref start, ref a, ref b );
		}

		public virtual void drawArc( ref btVector3 center, ref btVector3 normal, ref btVector3 axis, double radiusA, double radiusB, double minAngle, double maxAngle,
					ref btVector3 color, bool drawSect, double stepDegrees = btScalar.BT_TEN )
		{
			btVector3 vx = axis;
			btVector3 vy; normal.cross( ref axis, out vy );
			double step = stepDegrees * btScalar.SIMD_RADS_PER_DEG;
			int nSteps = (int)btScalar.btFabs( ( maxAngle - minAngle ) / step );
			if( nSteps == 0 ) nSteps = 1;
			btVector3 prev;
			center.AddScale( ref vx, radiusA * btScalar.btCos( minAngle ), out prev );
			prev.AddScale( ref vy, radiusB * btScalar.btSin( minAngle ), out prev );
			if( drawSect )
			{
				drawLine( ref center, ref prev, ref color );
			}
			for( int i = 1; i <= nSteps; i++ )
			{
				double angle = minAngle + ( maxAngle - minAngle ) * i / (double)nSteps;
				btVector3 next;
				center.AddScale( ref vx, radiusA * btScalar.btCos( angle ), out next );
				next.AddScale( ref vy, radiusB * btScalar.btSin( angle ), out next );
				drawLine( ref prev, ref next, ref color );
				prev = next;
			}
			if( drawSect )
			{
				drawLine( ref center, ref prev, ref color );
			}
		}
		public virtual void drawSpherePatch( ref btVector3 center, ref btVector3 up, ref btVector3 axis, double radius,
			double minTh, double maxTh, double minPs, double maxPs, ref btVector3 color
			, double stepDegrees = btScalar.BT_TEN
			, bool drawCenter = true )
		{
			btVector3[] vA = new btVector3[74];
			btVector3[] vB = new btVector3[74];
			btVector3[] vT;
			//btVector3* pvA = vA, *pvB = vB, *pT;

			btVector3 npole; center.AddScale( ref up, radius, out npole );
			btVector3 spole;  center.AddScale( ref up, -radius, out spole );
			btVector3 arcStart = btVector3.Zero;
			double step = stepDegrees * btScalar.SIMD_RADS_PER_DEG;
			btVector3 kv = up;
			btVector3 iv = axis;
			btVector3 jv; kv.cross( ref iv, out jv );
			bool drawN = false;
			bool drawS = false;
			if( minTh <= -btScalar.SIMD_HALF_PI )
			{
				minTh = -btScalar.SIMD_HALF_PI + step;
				drawN = true;
			}
			if( maxTh >= btScalar.SIMD_HALF_PI )
			{
				maxTh = btScalar.SIMD_HALF_PI - step;
				drawS = true;
			}
			if( minTh > maxTh )
			{
				minTh = -btScalar.SIMD_HALF_PI + step;
				maxTh = btScalar.SIMD_HALF_PI - step;
				drawN = drawS = true;
			}
			int n_hor = (int)( ( maxTh - minTh ) / step ) + 1;
			if( n_hor < 2 ) n_hor = 2;
			double step_h = ( maxTh - minTh ) / (double)( n_hor - 1 );
			bool isClosed = false;
			if( minPs > maxPs )
			{
				minPs = -btScalar.SIMD_PI + step;
				maxPs = btScalar.SIMD_PI;
				isClosed = true;
			}
			else if( ( maxPs - minPs ) >= btScalar.SIMD_2_PI )
			{
				isClosed = true;
			}
			else
			{
				isClosed = false;
			}
			int n_vert = (int)( ( maxPs - minPs ) / step ) + 1;
			if( n_vert < 2 ) n_vert = 2;
			double step_v = ( maxPs - minPs ) / (double)( n_vert - 1 );
			for( int i = 0; i < n_hor; i++ )
			{
				double th = minTh + (double)( i ) * step_h;
				double sth = radius * btScalar.btSin( th );
				double cth = radius * btScalar.btCos( th );
				for( int j = 0; j < n_vert; j++ )
				{
					double psi = minPs + (double)( j ) * step_v;
					double sps = btScalar.btSin( psi );
					double cps = btScalar.btCos( psi );

					center.AddScale( ref iv, cth * cps, out vB[j] );
					vB[j].AddScale( ref jv, cth * sps, out vB[j] );
					vB[j].AddScale( ref kv, sth, out vB[j] );
					//pvB[j] = center + cth * cps * iv + cth * sps * jv + sth * kv;
					if( i != 0 )
					{
						drawLine( ref vA[j], ref vB[j], ref color );
					}
					else if( drawS )
					{
						drawLine( ref spole, ref vB[j], ref color );
					}
					if( j != 0 )
					{
						drawLine( ref vB[j - 1], ref vB[j], ref color );
					}
					else
					{
						arcStart = vB[j];
					}
					if( ( i == ( n_hor - 1 ) ) && drawN )
					{
						drawLine( ref npole, ref vB[j], ref color );
					}

					if( drawCenter )
					{
						if( isClosed )
						{
							if( j == ( n_vert - 1 ) )
							{
								drawLine( ref arcStart, ref vB[j], ref color );
							}
						}
						else
						{
							if( ( ( i== 0 ) || ( i == ( n_hor - 1 ) ) ) && ( ( j == 0 ) || ( j == ( n_vert - 1 ) ) ) )
							{
								drawLine( ref center, ref vB[j], ref color );
							}
						}
					}
				}
				vT = vA; vA = vB; vB = vT;
				//pT = pvA; pvA = pvB; pvB = pT;
			}
		}

		public void putLine( btVector3 a, btVector3 b, ref btVector3 c )
		{
			drawLine( ref a, ref b, ref c );
		}

		public virtual void drawBox( ref btVector3 bbMin, ref btVector3 bbMax, ref btVector3 color )
		{
			putLine( new btVector3( bbMin[0], bbMin[1], bbMin[2] ), new btVector3( bbMax[0], bbMin[1], bbMin[2] ), ref color );
			putLine( new btVector3( bbMax[0], bbMin[1], bbMin[2] ), new btVector3( bbMax[0], bbMax[1], bbMin[2] ), ref color );
			putLine( new btVector3( bbMax[0], bbMax[1], bbMin[2] ), new btVector3( bbMin[0], bbMax[1], bbMin[2] ), ref color );
			putLine( new btVector3( bbMin[0], bbMax[1], bbMin[2] ), new btVector3( bbMin[0], bbMin[1], bbMin[2] ), ref color );
			putLine( new btVector3( bbMin[0], bbMin[1], bbMin[2] ), new btVector3( bbMin[0], bbMin[1], bbMax[2] ), ref color );
			putLine( new btVector3( bbMax[0], bbMin[1], bbMin[2] ), new btVector3( bbMax[0], bbMin[1], bbMax[2] ), ref color );
			putLine( new btVector3( bbMax[0], bbMax[1], bbMin[2] ), new btVector3( bbMax[0], bbMax[1], bbMax[2] ), ref color );
			putLine( new btVector3( bbMin[0], bbMax[1], bbMin[2] ), new btVector3( bbMin[0], bbMax[1], bbMax[2] ), ref color );
			putLine( new btVector3( bbMin[0], bbMin[1], bbMax[2] ), new btVector3( bbMax[0], bbMin[1], bbMax[2] ), ref color );
			putLine( new btVector3( bbMax[0], bbMin[1], bbMax[2] ), new btVector3( bbMax[0], bbMax[1], bbMax[2] ), ref color );
			putLine( new btVector3( bbMax[0], bbMax[1], bbMax[2] ), new btVector3( bbMin[0], bbMax[1], bbMax[2] ), ref color );
			putLine( new btVector3( bbMin[0], bbMax[1], bbMax[2] ), new btVector3( bbMin[0], bbMin[1], bbMax[2] ), ref color );
		}

		public void putLine( ref btTransform trans, double a, double b, double c, double d, double e, double f, ref btVector3 color )
		{
			btVector3 av;
			btVector3 bv;
			btVector3.setValue( out av, a, b, c );
			btVector3.setValue( out bv, d, e, f );
			trans.Apply( ref av, out av ); trans.Apply( ref bv, out bv );
			drawLine( ref av, ref bv, ref color );
		}
		public virtual void drawBox( ref btVector3 bbMin, ref btVector3 bbMax, ref btTransform trans, ref btVector3 color )
		{
			putLine( ref trans, bbMin[0], bbMin[1], bbMin[2], bbMax[0], bbMin[1], bbMin[2], ref color );
			putLine( ref trans, bbMax[0], bbMin[1], bbMin[2], bbMax[0], bbMax[1], bbMin[2], ref color );
			putLine( ref trans, bbMax[0], bbMax[1], bbMin[2], bbMin[0], bbMax[1], bbMin[2], ref color );
			putLine( ref trans, bbMin[0], bbMax[1], bbMin[2], bbMin[0], bbMin[1], bbMin[2], ref color );
			putLine( ref trans, bbMin[0], bbMin[1], bbMin[2], bbMin[0], bbMin[1], bbMax[2], ref color );
			putLine( ref trans, bbMax[0], bbMin[1], bbMin[2], bbMax[0], bbMin[1], bbMax[2], ref color );
			putLine( ref trans, bbMax[0], bbMax[1], bbMin[2], bbMax[0], bbMax[1], bbMax[2], ref color );
			putLine( ref trans, bbMin[0], bbMax[1], bbMin[2], bbMin[0], bbMax[1], bbMax[2], ref color );
			putLine( ref trans, bbMin[0], bbMin[1], bbMax[2], bbMax[0], bbMin[1], bbMax[2], ref color );
			putLine( ref trans, bbMax[0], bbMin[1], bbMax[2], bbMax[0], bbMax[1], bbMax[2], ref color );
			putLine( ref trans, bbMax[0], bbMax[1], bbMax[2], bbMin[0], bbMax[1], bbMax[2], ref color );
			putLine( ref trans, bbMin[0], bbMax[1], bbMax[2], bbMin[0], bbMin[1], bbMax[2], ref color );
		}

		public virtual void drawCapsule( double radius, double halfHeight, int upAxis, ref btTransform transform, ref btVector3 color )
		{
			int stepDegrees = 30;

			btVector3 capStart = btVector3.Zero;
			capStart[upAxis] = -halfHeight;

			btVector3 capEnd = btVector3.Zero;
			capEnd[upAxis] = halfHeight;

			// Draw the ends
			{
				btTransform childTransform = transform;
				transform.Apply( ref capStart, out childTransform.m_origin );
				{
					btVector3 center = childTransform.getOrigin();
					btVector3 up = childTransform.getBasis().getColumn( ( upAxis + 1 ) % 3 );
					btVector3 axis = childTransform.getBasis().getColumn( upAxis );
					axis.Invert( out axis );
					double minTh = -btScalar.SIMD_HALF_PI;
					double maxTh = btScalar.SIMD_HALF_PI;
					double minPs = -btScalar.SIMD_HALF_PI;
					double maxPs = btScalar.SIMD_HALF_PI;

					drawSpherePatch( ref center, ref up, ref axis, radius, minTh, maxTh, minPs, maxPs, ref color, (double)stepDegrees, false );
				}
			}

			{
				btTransform childTransform = transform;
				transform.Apply( ref capEnd, out childTransform.m_origin );
				{
					btVector3 center = childTransform.getOrigin();
					btVector3 up = childTransform.getBasis().getColumn( ( upAxis + 1 ) % 3 );
					btVector3 axis = childTransform.getBasis().getColumn( upAxis );
					double minTh = -btScalar.SIMD_HALF_PI;
					double maxTh = btScalar.SIMD_HALF_PI;
					double minPs = -btScalar.SIMD_HALF_PI;
					double maxPs = btScalar.SIMD_HALF_PI;
					drawSpherePatch( ref center, ref up, ref axis, radius, minTh, maxTh, minPs, maxPs, ref color, (double)stepDegrees, false );
				}
			}

			// Draw some additional lines
			btVector3 start = transform.getOrigin();

			for( int i = 0; i < 360; i += stepDegrees )
			{
				capEnd[( upAxis + 1 ) % 3] = capStart[( upAxis + 1 ) % 3] = btScalar.btSin( (double)i * btScalar.SIMD_RADS_PER_DEG ) * radius;
				capEnd[( upAxis + 2 ) % 3] = capStart[( upAxis + 2 ) % 3] = btScalar.btCos( (double)i * btScalar.SIMD_RADS_PER_DEG ) * radius;
				btVector3 a, b;
				transform.m_basis.Apply( ref capStart, out a );
				a.Add( ref start, out a );
				transform.m_basis.Apply( ref capEnd, out b );
				a.Add( ref start, out b );
				drawLine( ref a, ref b, ref color );
			}

		}

		public virtual void drawCylinder( double radius, double halfHeight, int upAxis, ref btTransform transform, ref btVector3 color )
		{
			btVector3 start = transform.getOrigin();
			btVector3 offsetHeight = btVector3.Zero;
			offsetHeight[upAxis] = halfHeight;
			int stepDegrees = 30;
			btVector3 capStart = btVector3.Zero;
			capStart[upAxis] = -halfHeight;
			btVector3 capEnd = btVector3.Zero;
			capEnd[upAxis] = halfHeight;
			btVector3 a, b, c;
			for( int i = 0; i < 360; i += stepDegrees )
			{
				capEnd[( upAxis + 1 ) % 3] = capStart[( upAxis + 1 ) % 3] = btScalar.btSin( (double)( i ) * btScalar.SIMD_RADS_PER_DEG ) * radius;
				capEnd[( upAxis + 2 ) % 3] = capStart[( upAxis + 2 ) % 3] = btScalar.btCos( (double)( i ) * btScalar.SIMD_RADS_PER_DEG ) * radius;
				transform.m_basis.Apply( ref capStart, out a );
				a.Add( ref start, out a );
				transform.m_basis.Apply( ref capEnd, out b );
				b.Add( ref start, out b );
				drawLine( ref a, ref b, ref color );
			}
			// Drawing top and bottom caps of the cylinder
			btVector3 yaxis = btVector3.Zero;
			yaxis[upAxis] = (double)1.0;
			btVector3 xaxis = btVector3.Zero;
			xaxis[( upAxis + 1 ) % 3] = (double)1.0;
			transform.m_basis.Apply( ref offsetHeight, out a );
			start.Sub( ref a, out a );
			transform.m_basis.Apply( ref yaxis, out b );
			transform.m_basis.Apply( ref xaxis, out c );
			drawArc( ref a, ref b, ref c, radius, radius, 0, btScalar.SIMD_2_PI, ref color, false, (double)( 10.0 ) );
			transform.m_basis.Apply( ref offsetHeight, out a );
			start.Add( ref a, out a );
			drawArc( ref a, ref b, ref c, radius, radius, 0, btScalar.SIMD_2_PI, ref color, false, (double)10.0 );
		}

		public virtual void drawCone( double radius, double height, int upAxis, ref btTransform transform, ref btVector3 color )
		{
			int stepDegrees = 30;
			btVector3 start = transform.getOrigin();

			btVector3 offsetHeight = btVector3.Zero;
			double halfHeight = height * (double)0.5;
			offsetHeight[upAxis] = halfHeight;
			btVector3 offsetRadius = btVector3.Zero;
			offsetRadius[( upAxis + 1 ) % 3] = radius;
			btVector3 offset2Radius = btVector3.Zero;
			offset2Radius[( upAxis + 2 ) % 3] = radius;


			btVector3 capEnd = btVector3.Zero;
			capEnd[upAxis] = -halfHeight;

			btVector3 a, b, c, tmp;
			for( int i = 0; i < 360; i += stepDegrees )
			{
				capEnd[( upAxis + 1 ) % 3] = btScalar.btSin( (double)i * btScalar.SIMD_RADS_PER_DEG ) * radius;
				capEnd[( upAxis + 2 ) % 3] = btScalar.btCos( (double)i * btScalar.SIMD_RADS_PER_DEG ) * radius;
				transform.m_basis.Apply( ref offsetHeight, out a );
				a.Add( ref start, out a );
				transform.m_basis.Apply( ref capEnd, out b );
				b.Add( ref start, out b );
				drawLine( ref a, ref b, ref color );
			}
			transform.m_basis.Apply( ref offsetHeight, out a );
			start.Add( ref a, out a );

			offsetRadius.Sub( ref offsetHeight, out tmp );
			transform.m_basis.Apply( ref tmp, out b );
			start.Add( ref b, out b );
			drawLine( ref a, ref b, ref color );

			offsetRadius.Invert( out tmp );
			tmp.Sub( ref offsetHeight, out tmp );
			transform.m_basis.Apply( ref tmp, out b );
			start.Add( ref b, out b );
			drawLine( ref a, ref b, ref color );

			offset2Radius.Sub( ref offsetHeight, out tmp );
			transform.m_basis.Apply( ref tmp, out b );
			start.Add( ref b, out b );
			drawLine( ref a, ref b, ref color );

			offset2Radius.Invert( out tmp );
			tmp.Sub( ref offsetHeight, out tmp );
			transform.m_basis.Apply( ref tmp, out b );
			start.Add( ref b, out b );
			drawLine( ref a, ref b, ref color );

			// Drawing the base of the cone
			btVector3 yaxis = btVector3.Zero;
			yaxis[upAxis] = (double)1.0;
			btVector3 xaxis = btVector3.Zero;
			xaxis[( upAxis + 1 ) % 3] = (double)1.0;

			transform.m_basis.Apply( ref offsetHeight, out a );
			start.Sub( ref a, out a );
			transform.m_basis.Apply( ref yaxis, out b );
			transform.m_basis.Apply( ref xaxis, out c );
			drawArc( ref a, ref b, ref c, radius, radius, 0, btScalar.SIMD_2_PI, ref color, false, 10.0 );
		}

		public virtual void drawPlane( ref btVector3 planeNormal, double planeConst, ref btTransform transform, ref btVector3 color )
		{
			btVector3 planeOrigin; planeNormal.Mult( planeConst, out planeOrigin );
			btVector3 vec0, vec1;
			btVector3.btPlaneSpace1( ref planeNormal, out vec0, out vec1 );
			double vecLen = 100;
			vec0.Mult( vecLen, out vec0 );
			vec1.Mult( vecLen, out vec1 );
			btVector3 pt0; planeOrigin.Add( ref vec0, out pt0 );
			btVector3 pt1; planeOrigin.Sub( ref vec0, out pt1 );
			btVector3 pt2; planeOrigin.Add( ref vec1, out pt2 );
			btVector3 pt3; planeOrigin.Sub( ref vec1, out pt3 );
			transform.Apply( ref pt0, out pt0 );
			transform.Apply( ref pt1, out pt1 );
			transform.Apply( ref pt2, out pt2 );
			transform.Apply( ref pt3, out pt3 );
			drawLine( ref pt0, ref pt1, ref color );
			drawLine( ref pt2, ref pt3, ref color );
		}

		public virtual void flushLines()
		{
		}
	};

}


