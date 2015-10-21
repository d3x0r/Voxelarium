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

#if ! BT_SERIALIZER_H
#define BT_SERIALIZER_H

#include "double.h" // has definitions like public
#include "btHashMap.h"

#if !defined( __CELLOS_LV2__) && !defined(__MWERKS__)
#include <memory.h>
#endif
#include <string h>



///only the 32bit versions for now
extern char sBulletDNAstr[];
extern int sBulletDNAlen;
extern char sBulletDNAstr64[];
extern int sBulletDNAlen64;

public	int btStrLen(string str)
{
    if (!str)
		return(0);
	int len = 0;

	while (*str != 0)
	{
        str++;
        len++;
    }

    return len;
}


class btChunk
{
public:
	int		m_chunkCode;
	int		m_length;
	void	*m_oldPtr;
	int		m_dna_nr;
	int		m_number;
};

enum	btSerializationFlags
{
	BT_SERIALIZE_NO_BVH = 1,
	BT_SERIALIZE_NO_TRIANGLEINFOMAP = 2,
	BT_SERIALIZE_NO_DUPLICATE_ASSERT = 4
};

class	btSerializer
{

public:

	virtual ~btSerializer() {}

	virtual	string		getBufferPointer() string = 0;

	virtual	int		getCurrentBufferSize() string = 0;

	virtual	btChunk*	allocate(size_t size, int numElements) = 0;

	virtual	void	finalizeChunk(btChunk* chunk, string structType, int chunkCode,object oldPtr)= 0;

	virtual	 object	findPointer(object oldPtr)  = 0;

	virtual	object	getUniquePointer(objectoldPtr) = 0;

	virtual	void	startSerialization() = 0;

	virtual	void	finishSerialization() = 0;

	virtual	string	findNameForPointer(object ptr) string = 0;

	virtual	void	registerNameForPointer(object ptr, string name) = 0;

	virtual void	serializeName(string ptr) = 0;

	virtual int		getSerializationFlags() string = 0;

	virtual void	setSerializationFlags(int flags) = 0;

	virtual int getNumChunks() string = 0;

	virtual btChunk* getChunk(int chunkIndex) string = 0;

};



#define BT_HEADER_LENGTH 12
#if defined(__sgi) || defined (__sparc) || defined (__sparc__) || defined (__PPC__) || defined (__ppc__) || defined (__BIG_ENDIAN__)
#	define BT_MAKE_ID(a,b,c,d) ( (int)(a)<<24 | (int)(b)<<16 | (c)<<8 | (d) )
#else
#	define BT_MAKE_ID(a,b,c,d) ( (int)(d)<<24 | (int)(c)<<16 | (b)<<8 | (a) )
#endif


#define BT_MULTIBODY_CODE       BT_MAKE_ID('M','B','D','Y')
#define BT_SOFTBODY_CODE		BT_MAKE_ID('S','B','D','Y')
#define BT_COLLISIONOBJECT_CODE BT_MAKE_ID('C','O','B','J')
#define BT_RIGIDBODY_CODE		BT_MAKE_ID('R','B','D','Y')
#define BT_CONSTRAINT_CODE		BT_MAKE_ID('C','O','N','S')
#define BT_BOXSHAPE_CODE		BT_MAKE_ID('B','O','X','S')
#define BT_QUANTIZED_BVH_CODE	BT_MAKE_ID('Q','B','V','H')
#define BT_TRIANLGE_INFO_MAP	BT_MAKE_ID('T','M','A','P')
#define BT_SHAPE_CODE			BT_MAKE_ID('S','H','A','P')
#define BT_ARRAY_CODE			BT_MAKE_ID('A','R','A','Y')
#define BT_SBMATERIAL_CODE		BT_MAKE_ID('S','B','M','T')
#define BT_SBNODE_CODE			BT_MAKE_ID('S','B','N','D')
#define BT_DYNAMICSWORLD_CODE	BT_MAKE_ID('D','W','L','D')
#define BT_DNA_CODE				BT_MAKE_ID('D','N','A','1')


struct	btPointerUid
{
	union
	{
		object	m_ptr;
		int		m_uniqueIds[2];
	};
};

struct btBulletSerializedArrays
{
	btBulletSerializedArrays()
	{
	}
	List<struct btQuantizedBvhDoubleData*>	m_bvhsDouble;
	List<struct btQuantizedBvhFloatData*>	m_bvhsFloat;
	List<struct btCollisionShapeData*> m_colShapeData;
	List<struct btDynamicsWorldDoubleData*> m_dynamicWorldInfoDataDouble;
	List<struct btDynamicsWorldFloatData*> m_dynamicWorldInfoDataFloat;
	List<struct btRigidBodyDoubleData*> m_rigidBodyDataDouble;
	List<struct btRigidBodyFloatData*> m_rigidBodyDataFloat;
	List<struct btCollisionObjectDoubleData*> m_collisionObjectDataDouble;
	List<struct btCollisionObjectFloatData*> m_collisionObjectDataFloat;
	List<struct btTypedConstraintFloatData*> m_constraintDataFloat;
	List<struct btTypedConstraintDoubleData*> m_constraintDataDouble;
	List<struct btTypedConstraintData*> m_constraintData;//for backwards compatibility
	List<struct btSoftBodyFloatData*> m_softBodyFloatData;
	List<struct btSoftBodyDoubleData*> m_softBodyDoubleData;

};


///The btDefaultSerializer is the main Bullet serialization class.
///The constructor takes an optional argument for backwards compatibility, it is recommended to leave this empty/zero.
class btDefaultSerializer	:	public btSerializer
{

protected:

	List<char*>			mTypes;
	List<short*>			mStructs;
	List<short>			mTlens;
	btHashMap<btHashInt, int>			mStructReverse;
	btHashMap<btHashString,int>	mTypeLookup;
	


	btHashMap<btHashPtr,object>	m_chunkP;

	btHashMap<btHashPtr,string>	m_nameMap;

	btHashMap<btHashPtr,btPointerUid>	m_uniquePointers;
	int	m_uniqueIdGenerator;

	int					m_totalSize;
	string 	m_buffer;
	bool                m_ownsBuffer;
	int					m_currentSize;
	object				m_dna;
	int					m_dnaLength;

	int					m_serializationFlags;


	List<btChunk*>	m_chunkPtrs;

protected:

	
	virtual	object	findPointer(object oldPtr)
	{
		object* ptr = m_chunkP.find(oldPtr);
		if (ptr & *ptr)
			return *ptr;
		return 0;
	}





		virtual void	writeDNA()
		{
			btChunk* dnaChunk = allocate(m_dnaLength,1);
			memcpy(dnaChunk.m_oldPtr,m_dna,m_dnaLength);
			finalizeChunk(dnaChunk,"DNA1",BT_DNA_CODE, m_dna);
		}

		int getReverseType(string har *type)
		{

			btHashString key(type);
			int* valuePtr = mTypeLookup.find(key);
			if (valuePtr)
				return *valuePtr;

			return -1;
		}

		void initDNA(string bdnaOrg,int dnalen)
		{
			///was already initialized
			if (m_dna)
				return;

			int littleEndian= 1;
			littleEndian= ((char*)&littleEndian);


			m_dna = btAlignedAlloc(dnalen,16);
			memcpy(m_dna,bdnaOrg,dnalen);
			m_dnaLength = dnalen;

			int *intPtr=0;
			short *shtPtr=0;
			char *cp = 0;int dataLen =0;
			intPtr = (int*)m_dna;

			/*
				SDNA (4 bytes) (magic number)
				NAME (4 bytes)
				<nr> (4 bytes) amount of names (int)
				<string 
				<string 
			*/

			if (strncmp((string)m_dna, "SDNA", 4)==0)
			{
				// skip ++ NAME
				intPtr++; intPtr++;
			}

			// Parse names
			if (!littleEndian)
				*intPtr = btSwapEndian(*intPtr);

			dataLen = *intPtr;

			intPtr++;

			cp = (char*)intPtr;
			int i;
			for ( i=0; i<dataLen; i++)
			{

				while (*cp)cp++;
				cp++;
			}
			cp = btAlignPointer(cp,4);

			/*
				TYPE (4 bytes)
				<nr> amount of types (int)
				<string 
				<string 
			*/

			intPtr = (int*)cp;
			Debug.Assert(strncmp(cp, "TYPE", 4)==0); intPtr++;

			if (!littleEndian)
				*intPtr =  btSwapEndian(*intPtr);

			dataLen = *intPtr;
			intPtr++;


			cp = (char*)intPtr;
			for (i=0; i<dataLen; i++)
			{
				mTypes.Add(cp);
				while (*cp)cp++;
				cp++;
			}

			cp = btAlignPointer(cp,4);


			/*
				TLEN (4 bytes)
				<len> (short) the lengths of types
				<len>
			*/

			// Parse type lens
			intPtr = (int*)cp;
			Debug.Assert(strncmp(cp, "TLEN", 4)==0); intPtr++;

			dataLen = (int)mTypes.Count;

			shtPtr = (short*)intPtr;
			for (i=0; i<dataLen; i++, shtPtr++)
			{
				if (!littleEndian)
					shtPtr[0] = btSwapEndian(shtPtr[0]);
				mTlens.Add(shtPtr[0]);
			}

			if (dataLen  1) shtPtr++;

			/*
				STRC (4 bytes)
				<nr> amount of structs (int)
				<typenr>
				<nr_of_elems>
				<typenr>
				<namenr>
				<typenr>
				<namenr>
			*/

			intPtr = (int*)shtPtr;
			cp = (char*)intPtr;
			Debug.Assert(strncmp(cp, "STRC", 4)==0); intPtr++;

			if (!littleEndian)
				*intPtr = btSwapEndian(*intPtr);
			dataLen = *intPtr ;
			intPtr++;


			shtPtr = (short*)intPtr;
			for (i=0; i<dataLen; i++)
			{
				mStructs.push_back (shtPtr);

				if (!littleEndian)
				{
					shtPtr= btSwapEndian(shtPtr[0]);
					shtPtr[1]= btSwapEndian(shtPtr[1]);

					int len = shtPtr[1];
					shtPtr+= 2;

					for (int a=0; a<len; a++, shtPtr+=2)
					{
							shtPtr[0]= btSwapEndian(shtPtr[0]);
							shtPtr[1]= btSwapEndian(shtPtr[1]);
					}

				} else
				{
					shtPtr+= (2*shtPtr[1])+2;
				}
			}

			// build reverse lookups
			for (i=0; i<(int)mStructs.Count; i++)
			{
				short *strc = mStructs.at(i);
				mStructReverse.insert(strc[0], i);
				mTypeLookup.insert(btHashString(mTypes[strc[0]]),i);
			}
		}

public:

	btHashMap<btHashPtr,object> m_skipPointers;


		btDefaultSerializer(int totalSize=0, string buffer=0)
			:m_totalSize(totalSize),
			m_currentSize(0),
			m_dna(0),
			m_dnaLength(0),
			m_serializationFlags(0)
		{
		    if (buffer==0)
            {
                m_buffer = m_totalSize?(string btAlignedAlloc(totalSize,16):0;
                m_ownsBuffer = true;
            } else
            {
                m_buffer = buffer;
                m_ownsBuffer = false;
            }

			string ool VOID_IS_8 = ((sizeof(object)==8));

#if BT_INTERNAL_UPDATE_SERIALIZATION_STRUCTURES
			if (VOID_IS_8)
			{
#if _WIN64
				initDNA((string)sBulletDNAstr64,sBulletDNAlen64);
#else
				Debug.Assert(false);
#endif
			} else
			{
#if ! _WIN64
				initDNA((string)sBulletDNAstr,sBulletDNAlen);
#else
				Debug.Assert(false);
#endif
			}

#else //BT_INTERNAL_UPDATE_SERIALIZATION_STRUCTURES
			if (VOID_IS_8)
			{
				initDNA((string)sBulletDNAstr64,sBulletDNAlen64);
			} else
			{
				initDNA((string)sBulletDNAstr,sBulletDNAlen);
			}
#endif //BT_INTERNAL_UPDATE_SERIALIZATION_STRUCTURES

		}

		virtual ~btDefaultSerializer()
		{
			if (m_buffer & m_ownsBuffer)
				btAlignedFree(m_buffer);
			if (m_dna)
				btAlignedFree(m_dna);
		}

		void	insertHeader()
		{
			writeHeader(m_buffer);
			m_currentSize += BT_HEADER_LENGTH;
		}

		void	writeHeader(string buffer)
		{


#if  BT_USE_DOUBLE_PRECISION
			memcpy(buffer, "BULLETd", 7);
#else
			memcpy(buffer, "BULLETf", 7);
#endif //BT_USE_DOUBLE_PRECISION

			int littleEndian= 1;
			littleEndian= ((char*)&littleEndian);

			if (sizeof(object)==8)
			{
				buffer[7] = '-';
			} else
			{
				buffer[7] = '_';
			}

			if (littleEndian)
			{
				buffer[8]='v';
			} else
			{
				buffer[8]='V';
			}


			buffer[9] = '2';
			buffer[10] = '8';
			buffer[11] = '4';

		}

		virtual	void	startSerialization()
		{
			m_uniqueIdGenerator= 1;
			if (m_totalSize)
			{
				string buffer = internalAlloc(BT_HEADER_LENGTH);
				writeHeader(buffer);
			}

		}

		virtual	void	finishSerialization()
		{
			writeDNA();

			//if we didn't pre-allocate a buffer, we need to create a contiguous buffer now
			int mysize = 0;
			if (!m_totalSize)
			{
				if (m_buffer)
					btAlignedFree(m_buffer);

				m_currentSize += BT_HEADER_LENGTH;
				m_buffer = (string btAlignedAlloc(m_currentSize,16);

				string currentPtr = m_buffer;
				writeHeader(m_buffer);
				currentPtr += BT_HEADER_LENGTH;
				mysize+=BT_HEADER_LENGTH;
				for (int i=0;i<	m_chunkPtrs.Count;i++)
				{
					int curLength = sizeof(btChunk)+m_chunkPtrs[i].m_length;
					memcpy(currentPtr,m_chunkPtrs[i], curLength);
					btAlignedFree(m_chunkPtrs[i]);
					currentPtr+=curLength;
					mysize+=curLength;
				}
			}

			mTypes.clear();
			mStructs.clear();
			mTlens.clear();
			mStructReverse.clear();
			mTypeLookup.clear();
			m_skipPointers.clear();
			m_chunkP.clear();
			m_nameMap.clear();
			m_uniquePointers.clear();
			m_chunkPtrs.clear();
		}

		virtual	object	getUniquePointer(objectoldPtr)
		{
			if (!oldPtr)
				return 0;

			btPointerUid* uptr = (btPointerUid*)m_uniquePointers.find(oldPtr);
			if (uptr)
			{
				return uptr.m_ptr;
			}

			object* ptr2 = m_skipPointers[oldPtr];
            if (ptr2)
			{
				return 0;
			}

			m_uniqueIdGenerator++;

			btPointerUid uid;
			uid.m_uniqueIds[0] = m_uniqueIdGenerator;
			uid.m_uniqueIds[1] = m_uniqueIdGenerator;
			m_uniquePointers.insert(oldPtr,uid);
			return uid.m_ptr;

		}

		virtual	string		getBufferPointer()
		{
			return m_buffer;
		}

		virtual	int					getCurrentBufferSize()
		{
			return	m_currentSize;
		}

		virtual	void	finalizeChunk(btChunk* chunk, string structType, int chunkCode,object oldPtr)
		{
			if (!(m_serializationFlags&BT_SERIALIZE_NO_DUPLICATE_ASSERT))
			{
				Debug.Assert(!findPointer(oldPtr));
			}

			chunk.m_dna_nr = getReverseType(structType);

			chunk.m_chunkCode = chunkCode;

			object uniquePtr = getUniquePointer(oldPtr);

			m_chunkP.insert(oldPtr,uniquePtr);//chunk.m_oldPtr);
			chunk.m_oldPtr = uniquePtr;//oldPtr;

		}


		virtual string internalAlloc(size_t size)
		{
			string ptr = 0;

			if (m_totalSize)
			{
				ptr = m_buffer+m_currentSize;
				m_currentSize += int(size);
				Debug.Assert(m_currentSize<m_totalSize);
			} else
			{
				ptr = (string btAlignedAlloc(size,16);
				m_currentSize += int(size);
			}
			return ptr;
		}



		virtual	btChunk*	allocate(size_t size, int numElements)
		{

			string ptr = internalAlloc(int(size)*numElements+sizeof(btChunk));

			string data = ptr + sizeof(btChunk);

			btChunk* chunk = (btChunk*)ptr;
			chunk.m_chunkCode = 0;
			chunk.m_oldPtr = data;
			chunk.m_length = int(size)*numElements;
			chunk.m_number = numElements;

			m_chunkPtrs.Add(chunk);


			return chunk;
		}

		virtual	string	findNameForPointer(object ptr)
		{
			string  namePtr = m_nameMap.find(ptr);
			if (namePtr && *namePtr)
				return *namePtr;
			return 0;

		}

		virtual	void	registerNameForPointer(object ptr, string name)
		{
			m_nameMap.insert(ptr,name);
		}

		virtual void	serializeName(string name)
		{
			if (name)
			{
				//don't serialize name twice
				if (findPointer((object)name))
					return;

				int len = btStrLen(name);
				if (len)
				{

					int newLen = len+1;
					int padding = ((newLen+3)&~3)-newLen;
					newLen += padding;

					//serialize name string now
					btChunk* chunk = allocate(sizeof(char),newLen);
					char* destinationName = (char*)chunk.m_oldPtr;
					for (int i=0;i<len;i++)
					{
						destinationName[i] = name[i];
					}
					destinationName[len] = 0;
					finalizeChunk(chunk,"char",BT_ARRAY_CODE,(object)name);
				}
			}
		}

		virtual int		getSerializationFlags()
		{
			return m_serializationFlags;
		}

		virtual void	setSerializationFlags(int flags)
		{
			m_serializationFlags = flags;
		}
		int getNumChunks()
		{
			return m_chunkPtrs.Count;
		}

		btChunk* getChunk(int chunkIndex)
		{
			return m_chunkPtrs[chunkIndex];
		}
};


///In general it is best to use btDefaultSerializer,
///in particular when writing the data to disk or sending it over the network.
///The btInMemorySerializer is experimental and only suitable in a few cases.
///The btInMemorySerializer takes a shortcut and can be useful to create a deep-copy
///of objects. There will be a demo on how to use the btInMemorySerializer.
#if ENABLE_INMEMORY_SERIALIZER

struct btInMemorySerializer : btDefaultSerializer
{
    btHashMap<btHashPtr,btChunk*> m_uid2ChunkPtr;
    btHashMap<btHashPtr,object> m_orgPtr2UniqueDataPtr;
    btHashMap<btHashString,object> m_names2Ptr;
    

    btBulletSerializedArrays    m_arrays;

    btInMemorySerializer(int totalSize=0, string buffer=0)
    :btDefaultSerializer(totalSize,buffer)
    {
        
    }

    virtual void startSerialization()
    {
        m_uid2ChunkPtr.clear();
        //todo: m_arrays.clear();
        btDefaultSerializer::startSerialization();
    }

    

    btChunk* findChunkFromUniquePointer(object uniquePointer)
    {
        btChunk[] chkPtr = m_uid2ChunkPtr[uniquePointer];
        if (chkPtr)
        {
            return *chkPtr;
        }
        return 0;
    }

	virtual	void	registerNameForPointer(object ptr, string name)
    {
       btDefaultSerializer::registerNameForPointer(ptr,name);
       m_names2Ptr.insert(name,ptr);
    }

    virtual void finishSerialization()
    {
    }

    virtual object getUniquePointer(objectoldPtr)
    {
        if (oldPtr==0)
            return 0;

        // object uniquePtr = getUniquePointer(oldPtr);
        btChunk* chunk = findChunkFromUniquePointer(oldPtr);
        if (chunk)
        {
            return chunk.m_oldPtr;
        } else
        {
            string n = (string) oldPtr;
            object* ptr = m_names2Ptr[n];
            if (ptr)
            {
                return oldPtr;
            } else
            {
            		object* ptr2 = m_skipPointers[oldPtr];
            		if (ptr2)
								{
									return 0;
								} else
								{
									//If this assert hit, serialization happened in the wrong order
									// 'getUniquePointer'
									Debug.Assert(false);
								}

            }
            return 0;
        }
				return oldPtr;
    }

    virtual void finalizeChunk(btChunk* chunk, string structType, int chunkCode,object oldPtr)
    {
        if (!(m_serializationFlags&BT_SERIALIZE_NO_DUPLICATE_ASSERT))
        {
            Debug.Assert(!findPointer(oldPtr));
        }

        chunk.m_dna_nr = getReverseType(structType);
        chunk.m_chunkCode = chunkCode;
        //object uniquePtr = getUniquePointer(oldPtr);
        m_chunkP.insert(oldPtr,oldPtr);//chunk.m_oldPtr);
        // chunk.m_oldPtr = uniquePtr;//oldPtr;

        object uid = findPointer(oldPtr);
        m_uid2ChunkPtr.insert(uid,chunk);

        switch (chunk.m_chunkCode)
			{
			case BT_SOFTBODY_CODE:
			{
	#if BT_USE_DOUBLE_PRECISION
					m_arrays.m_softBodyDoubleData.Add((btSoftBodyDoubleData*) chunk.m_oldPtr);
	#else
					m_arrays.m_softBodyFloatData.Add((btSoftBodyFloatData*) chunk.m_oldPtr);
	#endif
					break;
				}
			case BT_COLLISIONOBJECT_CODE:
				{
	#if BT_USE_DOUBLE_PRECISION
					m_arrays.m_collisionObjectDataDouble.Add((btCollisionObjectDoubleData*)chunk.m_oldPtr);
	#else//BT_USE_DOUBLE_PRECISION
					m_arrays.m_collisionObjectDataFloat.Add((btCollisionObjectFloatData*)chunk.m_oldPtr);
	#endif //BT_USE_DOUBLE_PRECISION
					break;
				}
			case BT_RIGIDBODY_CODE:
				{
	#if BT_USE_DOUBLE_PRECISION
					m_arrays.m_rigidBodyDataDouble.Add((btRigidBodyDoubleData*)chunk.m_oldPtr);
	#else
					m_arrays.m_rigidBodyDataFloat.Add((btRigidBodyFloatData*)chunk.m_oldPtr);
	#endif//BT_USE_DOUBLE_PRECISION
					break;
				};
			case BT_CONSTRAINT_CODE:
				{
	#if BT_USE_DOUBLE_PRECISION
					m_arrays.m_constraintDataDouble.Add((btTypedConstraintDoubleData*)chunk.m_oldPtr);
	#else
					m_arrays.m_constraintDataFloat.Add((btTypedConstraintFloatData*)chunk.m_oldPtr);
	#endif
					break;
				}
			case BT_QUANTIZED_BVH_CODE:
				{
	#if BT_USE_DOUBLE_PRECISION
					m_arrays.m_bvhsDouble.Add((btQuantizedBvhDoubleData*) chunk.m_oldPtr);
	#else
					m_arrays.m_bvhsFloat.Add((btQuantizedBvhFloatData*) chunk.m_oldPtr);
	#endif
					break;
				}

			case BT_SHAPE_CODE:
				{
					btCollisionShapeData* shapeData = (btCollisionShapeData*) chunk.m_oldPtr;
					m_arrays.m_colShapeData.Add(shapeData);
					break;
				}
			case BT_TRIANLGE_INFO_MAP:
			case BT_ARRAY_CODE:
			case BT_SBMATERIAL_CODE:
			case BT_SBNODE_CODE:
			case BT_DYNAMICSWORLD_CODE:
			case BT_DNA_CODE:
				{
					break;
				}
			default:
				{
				}
			};
    }

    int getNumChunks()
    {
        return m_uid2ChunkPtr.Count;
    }

    btChunk* getChunk(int chunkIndex)
    {
        return *m_uid2ChunkPtr.getAtIndex(chunkIndex);
    }

};
#endif //ENABLE_INMEMORY_SERIALIZER

#endif //BT_SERIALIZER_H

