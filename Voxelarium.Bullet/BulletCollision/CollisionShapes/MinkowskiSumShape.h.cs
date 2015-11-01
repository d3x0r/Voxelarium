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
using Bullet.Collision.Shapes;
using Bullet.LinearMath;

namespace Bullet.Collision.Shapes {
/// The btMinkowskiSumShape is only for advanced users. This shape represents implicit based minkowski sum of two convex implicit shapes.
internal class btMinkowskiSumShape : btConvexInternalShape
{

	btTransform	m_transA;
	btTransform	m_transB;
	btConvexShape	m_shapeA;
	btConvexShape	m_shapeB;

/*
	btMinkowskiSumShape(btConvexShape* shapeA,btConvexShape* shapeB);
	virtual btVector3	localGetSupportingVertexWithoutMargin(ref btVector3 vec)const;
	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(btVector3* vectors,btVector3* supportVerticesOut,int numVectors);
	virtual void	calculateLocalInertia(double mass,ref btVector3 inertia);
	public virtual double getMargin();
	*/

	public void setTransformA(ref btTransform	transA) { m_transA = transA;}
	public void setTransformB(ref btTransform	transB) { m_transB = transB;}

	public btITransform getTransformA() { return m_transA;}
	public btITransform GetTransformB() { return m_transB;}



	public btConvexShape	getShapeA() {  return m_shapeA;}
	public btConvexShape	getShapeB() {  return m_shapeB;}

	public override string ToString()   	{
		return "MinkowskiSum";
	}
};

}
