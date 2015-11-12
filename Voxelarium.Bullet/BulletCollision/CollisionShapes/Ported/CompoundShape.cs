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
using Bullet.Collision.Shapes;
using Bullet.LinearMath;
using Bullet.Types;
using System.Diagnostics;

namespace Bullet.Collision.Shapes
{

	//class btOptimizedBvh;

	internal class btCompoundShapeChild
	{
		internal btTransform m_transform;
		internal btCollisionShape m_childShape;
		internal BroadphaseNativeTypes m_childShapeType;
		internal double m_childMargin;
		internal btDbvt.btDbvtNode m_node;

		public bool Equals( btCompoundShapeChild c2 )
		{
			return ( m_transform.Equals( ref c2.m_transform ) &&
				m_childShape == c2.m_childShape &&
				m_childShapeType == c2.m_childShapeType &&
				m_childMargin == c2.m_childMargin );
		}
	};


	/// The btCompoundShape allows to store multiple other btCollisionShapes
	/// This allows for moving concave collision objects. This is more general then the static concave btBvhTriangleMeshShape.
	/// It has an (optional) dynamic aabb tree to accelerate early rejection tests. 
	/// @todo: This aabb tree can also be use to speed up ray tests on btCompoundShape, see http://code.google.com/p/bullet/issues/detail?id=25
	/// Currently, removal of child shapes is only supported when disabling the aabb tree (pass 'false' in the constructor of btCompoundShape)
	internal class btCompoundShape : btCollisionShape
	{

		internal btList<btCompoundShapeChild> m_children = new btList<btCompoundShapeChild>();
		protected btVector3 m_localAabbMin;
		protected btVector3 m_localAabbMax;

		protected btDbvt m_dynamicAabbTree;

		///increment m_updateRevision when adding/removing/replacing child shapes, so that some caches can be updated
		protected int m_updateRevision;

		protected double m_collisionMargin;

		protected btVector3 m_localScaling;

		public int getNumChildShapes()
		{
			return m_children.Count;
		}

		public btCollisionShape getChildShape( int index )
		{
			return m_children[index].m_childShape;
		}

		internal btTransform getChildTransform( int index )
		{
			return m_children[index].m_transform;
		}

		internal btList<btCompoundShapeChild> getChildList()
		{
			return m_children;
		}


		public override void getLocalScaling( out btVector3 result )
		{
			result = m_localScaling;
		}

		public override void setMargin( double margin )
		{
			m_collisionMargin = margin;
		}
		public override double getMargin()
		{
			return m_collisionMargin;
		}
		public override string ToString()
		{
			return "Compound";
		}

		public btDbvt getDynamicAabbTree()
		{
			return m_dynamicAabbTree;
		}

		public int getUpdateRevision()
		{
			return m_updateRevision;
		}

#if SERIALIZE_DONE
virtual int calculateSerializeBufferSize();

///fills the dataBuffer and returns the struct name (and 0 on failure)
virtual string serialize( object dataBuffer, btSerializer* serializer );
#endif


		public btCompoundShape( bool enableDynamicAabbTree = true, int initialChildCapacity = 0 )
		{
			m_localAabbMin = btVector3.Max;
			m_localAabbMax = btVector3.Min;
			m_dynamicAabbTree = ( null );
			m_updateRevision = ( 1 );
			m_collisionMargin = ( btScalar.BT_ZERO );
			m_localScaling = btVector3.One;
			m_shapeType = BroadphaseNativeTypes.COMPOUND_SHAPE_PROXYTYPE;

			if( enableDynamicAabbTree )
			{
				m_dynamicAabbTree = new btDbvt();
			}

			m_children.Capacity = ( initialChildCapacity );
		}


		~btCompoundShape()
		{
			if( m_dynamicAabbTree != null )
			{
				m_dynamicAabbTree = null;
			}
		}

		public void addChildShape( ref btTransform localTransform, btCollisionShape shape )
		{
			m_updateRevision++;
			//m_childTransforms.Add(localTransform);
			//m_childShapes.Add(shape);
			btCompoundShapeChild child = new btCompoundShapeChild();
			child.m_node = null;
			child.m_transform = localTransform;
			child.m_childShape = shape;
			child.m_childShapeType = shape.getShapeType();
			child.m_childMargin = shape.getMargin();


			//extend the local aabbMin/aabbMax
			btVector3 localAabbMin, localAabbMax;
			shape.getAabb( ref localTransform, out localAabbMin, out localAabbMax );
			for( int i = 0; i < 3; i++ )
			{
				if( m_localAabbMin[i] > localAabbMin[i] )
				{
					m_localAabbMin[i] = localAabbMin[i];
				}
				if( m_localAabbMax[i] < localAabbMax[i] )
				{
					m_localAabbMax[i] = localAabbMax[i];
				}

			}
			if( m_dynamicAabbTree != null )
			{
				btDbvt.btDbvtVolume bounds = btDbvt.btDbvtVolume.FromMM( ref localAabbMin, ref localAabbMax );
				int index = m_children.Count;
				child.m_node = m_dynamicAabbTree.insert( ref bounds, index );
			}

			m_children.Add( child );

		}

		///set a new transform for a child, and update internal data structures (local aabb and dynamic tree)
		void updateChildTransform( int childIndex, ref btTransform newChildTransform, bool shouldRecalculateLocalAabb )
		{
			m_children[childIndex].m_transform = newChildTransform;

			if( m_dynamicAabbTree != null )
			{
				///update the dynamic aabb tree
				btVector3 localAabbMin, localAabbMax;
				m_children[childIndex].m_childShape.getAabb( ref newChildTransform, out localAabbMin, out localAabbMax );
				btDbvt.btDbvtVolume  bounds = btDbvt.btDbvtVolume.FromMM( ref localAabbMin, ref localAabbMax );
				//int index = m_children.Count-1;
				m_dynamicAabbTree.update( m_children[childIndex].m_node, ref bounds );
			}

			if( shouldRecalculateLocalAabb )
			{
				recalculateLocalAabb();
			}
		}

		public void removeChildShapeByIndex( int childShapeIndex )
		{
			m_updateRevision++;
			Debug.Assert( childShapeIndex >= 0 && childShapeIndex < m_children.Count );
			if( m_dynamicAabbTree != null )
			{
				m_dynamicAabbTree.remove( m_children[childShapeIndex].m_node );
			}
			m_children.Swap( childShapeIndex, m_children.Count - 1 );
			if( m_dynamicAabbTree != null )
				m_children[childShapeIndex].m_node.dataAsInt = childShapeIndex;
			m_children.Count--;

		}



		/// Remove all children shapes that contain the specified shape
		public virtual void removeChildShape( btCollisionShape shape )
		{
			m_updateRevision++;
			// Find the children containing the shape specified, and remove those children.
			//note: there might be multiple children using the same shape!
			for( int i = m_children.Count - 1; i >= 0; i-- )
			{
				if( m_children[i].m_childShape == shape )
				{
					removeChildShapeByIndex( i );
				}
			}



			recalculateLocalAabb();
		}

		/* Re-calculate the local Aabb. Is called at the end of removeChildShapes. 
		Use this yourself if you modify the children or their transforms. */
		void recalculateLocalAabb()
		{
			// Recalculate the local aabb
			// Brute force, it iterates over all the shapes left.

			m_localAabbMin = btVector3.Max;
			m_localAabbMax = btVector3.Min;

			//extend the local aabbMin/aabbMax
			for( int j = 0; j < m_children.Count; j++ )
			{
				btVector3 localAabbMin, localAabbMax;
				m_children[j].m_childShape.getAabb( ref m_children[j].m_transform, out localAabbMin, out localAabbMax );
				for( int i = 0; i < 3; i++ )
				{
					if( m_localAabbMin[i] > localAabbMin[i] )
						m_localAabbMin[i] = localAabbMin[i];
					if( m_localAabbMax[i] < localAabbMax[i] )
						m_localAabbMax[i] = localAabbMax[i];
				}
			}
		}

		///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
		public override void getAabb( ref btTransform trans, out btVector3 aabbMin, out btVector3 aabbMax )
		{
			btVector3 localHalfExtents; btVector3.getHalfExtent( ref m_localAabbMin, ref m_localAabbMax, out localHalfExtents );
			btVector3 localCenter; btVector3.getCenter( ref m_localAabbMin, ref m_localAabbMax, out localCenter );

			//avoid an illegal AABB when there are no children
			if( m_children.Count == 0 )
			{
				localHalfExtents.setValue( 0, 0, 0 );
				localCenter.setValue( 0, 0, 0 );
			}
			btVector3 margin = new btVector3( getMargin() );
            localHalfExtents.Add( ref margin, out localHalfExtents );


			btMatrix3x3 abs_b; trans.m_basis.absolute( out abs_b );

			btVector3 center; trans.Apply( ref localCenter, out center );

			btVector3 extent; localHalfExtents.dot3( ref abs_b, out extent );
			aabbMin = center - extent;
			aabbMax = center + extent;

		}

		public override void calculateLocalInertia( double mass, out btVector3 inertia )
		{
			//approximation: take the inertia from the aabb for now
			btVector3 aabbMin, aabbMax;
			getAabb( ref btTransform.Identity, out aabbMin, out aabbMax );

			btVector3 halfExtents = ( aabbMax - aabbMin ) * (double)( 0.5 );

			double lx = (double)( 2.0) * ( halfExtents.x );
			double ly = (double)( 2.0) * ( halfExtents.y );
			double lz = (double)( 2.0) * ( halfExtents.z );

			inertia= new btVector3( mass / ( (double)( 12.0 ) ) * ( ly * ly + lz * lz )
				, mass / ( (double)( 12.0 ) ) * ( lx * lx + lz * lz )
				, mass / ( (double)( 12.0 ) ) * ( lx * lx + ly * ly ) );

		}




		///computes the exact moment of inertia and the transform from the coordinate system defined by the principal axes of the moment of inertia
		///and the center of mass to the current coordinate system. "masses" points to an array of masses of the children. The resulting transform
		///"principal" has to be applied inversely to all children transforms in order for the local coordinate system of the compound
		///shape to be centered at the center of mass and to coincide with the principal axes. This also necessitates a correction of the world transform
		///of the collision object by the principal transform.
		void calculatePrincipalAxisTransform( double[] masses, ref btTransform principal, ref btVector3 inertia )
		{
			int n = m_children.Count;

			double totalMass = 0;
			btVector3 center = btVector3.Zero;
			int k;

			for( k = 0; k < n; k++ )
			{
				Debug.Assert( masses[k] > 0 );
				center.AddScale( ref m_children[k].m_transform.m_origin, masses[k], out center );
				totalMass += masses[k];
			}

			Debug.Assert( totalMass > 0 );

			center /= totalMass;
			principal.setOrigin( ref center );

			btMatrix3x3 tensor = new btMatrix3x3( 0, 0, 0, 0, 0, 0, 0, 0, 0);
			for( k = 0; k < n; k++ )
			{
				btVector3 i;
				m_children[k].m_childShape.calculateLocalInertia( masses[k], out i );

				btTransform t = m_children[k].m_transform;
				btVector3 o; t.m_origin.Sub(ref center, out o );

				//compute inertia tensor in coordinate system of compound shape
				btMatrix3x3 j; t.m_basis.transpose( out j);
				j.m_el0 *= i.x;
				j.m_el1 *= i.y;
				j.m_el2 *= i.z;
				btMatrix3x3 j2;
				t.m_basis.Mult( ref j, out j2 );

				//add inertia tensor
				tensor.m_el0 += j2.m_el0;
				tensor.m_el1 += j2.m_el1;
				tensor.m_el2 += j2.m_el2;

				//compute inertia tensor of pointmass at o
				double o2 = o.length2();
				j[0].setValue( o2, 0, 0 );
				j[1].setValue( 0, o2, 0 );
				j[2].setValue( 0, 0, o2 );
				j.m_el0 += o * -o.x;
				j.m_el1 += o * -o.y;
				j.m_el2 += o * -o.z;

				//add inertia tensor of pointmass
				tensor.m_el0.AddScale( ref j.m_el0, masses[k], out tensor.m_el0 );
				tensor.m_el1.AddScale( ref j.m_el1, masses[k], out tensor.m_el1 );
				tensor.m_el2.AddScale( ref j.m_el2, masses[k], out tensor.m_el2 );
			}

			tensor.diagonalize( out  principal.m_basis, (double)( 0.00001 ), 20 );
			inertia.setValue( tensor[0][0], tensor[1][1], tensor[2][2] );
		}





		public override void setLocalScaling( ref btVector3 scaling )
		{

			for( int i = 0; i < m_children.Count; i++ )
			{
				btTransform childTrans = getChildTransform( i );
				btVector3 childScale; m_children[i].m_childShape.getLocalScaling( out childScale );
				//		childScale = childScale * (childTrans.getBasis() * scaling);
				childScale.Mult( ref scaling, out childScale );
				childScale.Div( ref m_localScaling, out childScale );
				//childScale = childScale * scaling / m_localScaling;
				m_children[i].m_childShape.setLocalScaling( ref childScale );
				btVector3 tmp;
				childTrans.m_origin.Mult( ref scaling, out tmp );
				tmp.Div( ref m_localScaling, out tmp );
				childTrans.setOrigin( ref tmp );
				updateChildTransform( i,ref childTrans, false );
			}

			m_localScaling = scaling;
			recalculateLocalAabb();

		}


		void createAabbTreeFromChildren()
		{
			if( m_dynamicAabbTree == null )
			{
				m_dynamicAabbTree = new btDbvt();
				
				for( int index = 0; index < m_children.Count; index++ )
				{
					btCompoundShapeChild  child = m_children[index];

					//extend the local aabbMin/aabbMax
					btVector3 localAabbMin, localAabbMax;
					child.m_childShape.getAabb( ref child.m_transform, out localAabbMin, out localAabbMax );

					btDbvt.btDbvtVolume bounds = btDbvt.btDbvtVolume.FromMM( ref localAabbMin, ref localAabbMax );
					int index2 = index;
					child.m_node = m_dynamicAabbTree.insert( ref bounds, index2 );
				}
			}
		}

#if SERIALIZE_DONE

///fills the dataBuffer and returns the struct name (and 0 on failure)
string btCompoundShape::serialize( object dataBuffer, btSerializer* serializer )
{

	btCompoundShapeData* shapeData = (btCompoundShapeData*)dataBuffer;
	btCollisionShape::serialize( &shapeData.m_collisionShapeData, serializer );

	shapeData.m_collisionMargin = float( m_collisionMargin );
	shapeData.m_numChildShapes = m_children.Count;
	shapeData.m_childShapePtr = 0;
	if( shapeData.m_numChildShapes )
	{
		btChunk* chunk = serializer.allocate( sizeof( btCompoundShapeChildData ), shapeData.m_numChildShapes );
		btCompoundShapeChildData* memPtr = (btCompoundShapeChildData*)chunk.m_oldPtr;
		shapeData.m_childShapePtr = (btCompoundShapeChildData*)serializer.getUniquePointer( memPtr );

		for( int i = 0; i < shapeData.m_numChildShapes; i++, memPtr++ )
		{
			memPtr.m_childMargin = float( m_children[i].m_childMargin );
			memPtr.m_childShape = (btCollisionShapeData*)serializer.getUniquePointer( m_children[i].m_childShape );
			//don't serialize shapes that already have been serialized
			if( !serializer.findPointer( m_children[i].m_childShape ) )
			{
				btChunk* chunk = serializer.allocate( m_children[i].m_childShape.calculateSerializeBufferSize(), 1 );
				string structType = m_children[i].m_childShape.serialize( chunk.m_oldPtr, serializer );
				serializer.finalizeChunk( chunk, structType, BT_SHAPE_CODE, m_children[i].m_childShape );
			}

			memPtr.m_childShapeType = m_children[i].m_childShapeType;
			m_children[i].m_transform.serializeFloat( memPtr.m_transform );
		}
		serializer.finalizeChunk( chunk, "btCompoundShapeChildData", BT_ARRAY_CODE, chunk.m_oldPtr );
	}
	return "btCompoundShapeData";
}

#endif

	};

#if SERIALIZE_DONE
///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btCompoundShapeChildData
{
	btTransformFloatData	m_transform;
	btCollisionShapeData	*m_childShape;
	int						m_childShapeType;
	float					m_childMargin;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btCompoundShapeData
{
	btCollisionShapeData		m_collisionShapeData;

	btCompoundShapeChildData	*m_childShapePtr;

	int							m_numChildShapes;

	float	m_collisionMargin;

};


public	int	btCompoundShape::calculateSerializeBufferSize()
{
	return sizeof(btCompoundShapeData);
}
#endif

}
