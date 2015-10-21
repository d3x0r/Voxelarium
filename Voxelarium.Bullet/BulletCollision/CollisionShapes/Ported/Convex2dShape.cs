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

using Bullet.Collision.BroadPhase;
using Bullet.LinearMath;

namespace Bullet.Collision.Shapes
{


	///The btConvex2dShape allows to use arbitrary convex shapes as 2d convex shapes, with the Z component assumed to be 0.
	///For 2d boxes, the btBox2dShape is recommended.
	internal class btConvex2dShape : btConvexShape
	{
		btConvexShape m_childConvexShape;


		public btConvexShape getChildShape()
		{
			return m_childConvexShape;
		}

		public override string ToString()
		{
			return "Convex2dShape";
		}


		///////////////////////////


		///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version

		public btConvex2dShape( btConvexShape convexChildShape )
		{
			m_childConvexShape = ( convexChildShape );
			m_shapeType = BroadphaseNativeTypes.CONVEX_2D_SHAPE_PROXYTYPE;
		}



		public override void localGetSupportingVertexWithoutMargin( ref btVector3 vec, out btVector3 result )
		{
			m_childConvexShape.localGetSupportingVertexWithoutMargin( ref vec, out result );
		}

		public override void batchedUnitVectorGetSupportingVertexWithoutMargin( btVector3[] vectors, btVector3[] supportVerticesOut, int numVectors )
		{
			m_childConvexShape.batchedUnitVectorGetSupportingVertexWithoutMargin( vectors, supportVerticesOut, numVectors );
		}


		public override void localGetSupportingVertex( ref btVector3 vec, out btVector3 result )
		{
			m_childConvexShape.localGetSupportingVertex( ref vec, out result );
		}


		public override void calculateLocalInertia( double mass, out btVector3 inertia )
		{
			///this linear upscaling is not realistic, but we don't deal with large mass ratios...
			m_childConvexShape.calculateLocalInertia( mass, out inertia );
		}


		///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
		public override void getAabb( ref btTransform t, out btVector3 aabbMin, out btVector3 aabbMax )
		{
			m_childConvexShape.getAabb( ref t, out aabbMin, out aabbMax );
		}

		public override void getAabbSlow( ref btTransform t, out btVector3 aabbMin, out btVector3 aabbMax )
		{
			m_childConvexShape.getAabbSlow( ref t, out aabbMin, out aabbMax );
		}

		public override void setLocalScaling( ref btVector3 scaling )
		{
			m_childConvexShape.setLocalScaling( ref scaling );
		}

		public override void getLocalScaling( out btVector3 result )
		{
			m_childConvexShape.getLocalScaling( out result );
		}

		public override void setMargin( double margin )
		{
			m_childConvexShape.setMargin( margin );
		}
		public override double getMargin()
		{
			return m_childConvexShape.getMargin();
		}

		public override int getNumPreferredPenetrationDirections()
		{
			return m_childConvexShape.getNumPreferredPenetrationDirections();
		}

		public override void getPreferredPenetrationDirection( int index, out btVector3 penetrationVector )
		{
			m_childConvexShape.getPreferredPenetrationDirection( index, out penetrationVector );
		}


	};

}
