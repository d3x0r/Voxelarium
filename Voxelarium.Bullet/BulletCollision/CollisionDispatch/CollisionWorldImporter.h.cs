/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2014 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#if SERIALIZER_DONE


class btCollisionWorldImporter
{
protected:
	btCollisionWorld* m_collisionWorld;

	int m_verboseMode;

	List<btCollisionShape*>  m_allocatedCollisionShapes;
	List<btCollisionObject> m_allocatedRigidBodies;

	List<btOptimizedBvh*>	 m_allocatedBvhs;
	List<btTriangleInfoMap*> m_allocatedTriangleInfoMaps;
	List<btTriangleIndexVertexArray*> m_allocatedTriangleIndexArrays;
	List<btStridingMeshInterfaceData*> m_allocatedbtStridingMeshInterfaceDatas;
	List<btCollisionObject> m_allocatedCollisionObjects;


	List<char*>				m_allocatedNames;

	List<int*>				m_indexArrays;
	List<short int*>		m_shortIndexArrays;
	List<string 	m_charIndexArrays;

	List<btVector3FloatData*>	m_floatVertexArrays;
	List<btVector3DoubleData*>	m_doubleVertexArrays;


	btHashMap<btHashPtr,btOptimizedBvh*>	m_bvhMap;
	btHashMap<btHashPtr,btTriangleInfoMap*>	m_timMap;

	btHashMap<btHashString,btCollisionShape*>	m_nameShapeMap;
	btHashMap<btHashString,btCollisionObject>	m_nameColObjMap;

	btHashMap<btHashPtr,string>	m_objectNameMap;

	btHashMap<btHashPtr,btCollisionShape*>	m_shapeMap;
	btHashMap<btHashPtr,btCollisionObject>	m_bodyMap;


	//methods



	char*	duplicateName(string name);

	btCollisionShape* convertCollisionShape(  btCollisionShapeData* shapeData  );


public:

	btCollisionWorldImporter(btCollisionWorld* world);

	virtual ~btCollisionWorldImporter();

    bool	convertAllObjects( btBulletSerializedArrays* arrays);

		///delete all memory collision shapes, rigid bodies, constraints etc. allocated during the load.
	///make sure you don't use the dynamics world containing objects after you call this method
	virtual void deleteAllData();

	void	setVerboseMode(int verboseMode)
	{
		m_verboseMode = verboseMode;
	}

	int getVerboseMode()
	{
		return m_verboseMode;
	}

		// query for data
	int	getNumCollisionShapes();
	btCollisionShape* getCollisionShapeByIndex(int index);
	int getNumRigidBodies();
	btCollisionObject getRigidBodyByIndex(int index);
	int getNumConstraints();

	int getNumBvhs();
	btOptimizedBvh*  getBvhByIndex(int index);
	int getNumTriangleInfoMaps();
	btTriangleInfoMap* getTriangleInfoMapByIndex(int index);

	// queris involving named objects
	btCollisionShape* getCollisionShapeByName(string name);
	btCollisionObject getCollisionObjectByName(string name);


	string	getNameForPointer(object ptr);

	///those virtuals are called by load and can be overridden by the user



	//bodies

	virtual btCollisionObject  createCollisionObject(	ref btTransform startTransform,	btCollisionShape* shape,string bodyName);

	///shapes

	virtual btCollisionShape* createPlaneShape(ref btVector3 planeNormal,double planeConstant);
	virtual btCollisionShape* createBoxShape(ref btVector3 halfExtents);
	virtual btCollisionShape* createSphereShape(double radius);
	virtual btCollisionShape* createCapsuleShapeX(double radius, double height);
	virtual btCollisionShape* createCapsuleShapeY(double radius, double height);
	virtual btCollisionShape* createCapsuleShapeZ(double radius, double height);

	virtual btCollisionShape* createCylinderShapeX(double radius,double height);
	virtual btCollisionShape* createCylinderShapeY(double radius,double height);
	virtual btCollisionShape* createCylinderShapeZ(double radius,double height);
	virtual btCollisionShape* createConeShapeX(double radius,double height);
	virtual btCollisionShape* createConeShapeY(double radius,double height);
	virtual btCollisionShape* createConeShapeZ(double radius,double height);
	virtual class btTriangleIndexVertexArray*	createTriangleMeshContainer();
	virtual	btBvhTriangleMeshShape* createBvhTriangleMeshShape(btStridingMeshInterface* trimesh, btOptimizedBvh* bvh);
	virtual btCollisionShape* createConvexTriangleMeshShape(btStridingMeshInterface* trimesh);
#if SUPPORT_GIMPACT_SHAPE_IMPORT
	virtual btGImpactMeshShape* createGimpactShape(btStridingMeshInterface* trimesh);
#endif //SUPPORT_GIMPACT_SHAPE_IMPORT
	virtual btStridingMeshInterfaceData* createStridingMeshInterfaceData(btStridingMeshInterfaceData* interfaceData);

	virtual class btConvexHullShape* createConvexHullShape();
	virtual class btCompoundShape* createCompoundShape();
	virtual class btScaledBvhTriangleMeshShape* createScaledTrangleMeshShape(btBvhTriangleMeshShape* meshShape,ref btVector3 localScalingbtBvhTriangleMeshShape);

	virtual class btMultiSphereShape* createMultiSphereShape(btVector3* positions,double* radi,int numSpheres);

	virtual btTriangleIndexVertexArray* createMeshInterface(btStridingMeshInterfaceData& meshData);

	///acceleration and connectivity structures
	virtual btOptimizedBvh*	createOptimizedBvh();
	virtual btTriangleInfoMap* createTriangleInfoMap();




};

#endif
