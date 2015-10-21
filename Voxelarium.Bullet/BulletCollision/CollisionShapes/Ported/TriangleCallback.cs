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

using Bullet.LinearMath;

namespace Bullet.Collision.Shapes
{

	///The btTriangleCallback provides a callback for each overlapping triangle when calling processAllTriangles.
	///This callback is called by processAllTriangles for all btConcaveShape derived class, such as  btBvhTriangleMeshShape, btStaticPlaneShape and btHeightfieldTerrainShape.

	public abstract class btTriangleCallback
	{
		public abstract void processTriangle( btVector3[] triangle, int partId, int triangleIndex );
	};

	public abstract class btInternalTriangleIndexCallback
	{
		public abstract void internalProcessTriangleIndex( btVector3[] triangle, int partId, int triangleIndex );
	};


}
