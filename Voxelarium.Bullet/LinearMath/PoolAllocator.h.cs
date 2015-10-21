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


namespace Voxelarium.Bullet.LinearMath
{
///The btPoolAllocator class allows to efficiently allocate a large pool of objects, instead of dynamically allocating them separately.
internal class btPoolAllocator
{
	int				m_elemSize;
	int				m_maxElements;
	int				m_freeCount;
	object			m_firstFree;
	string	m_pool;

public

	btPoolAllocator(int elemSize, int maxElements)
		:m_elemSize(elemSize),
		m_maxElements(maxElements)
	{
		m_pool = (string) btAlignedAlloc( static_cast<unsigned int>(m_elemSize*m_maxElements),16);

		string p = m_pool;
        m_firstFree = p;
        m_freeCount = m_maxElements;
        int count = m_maxElements;
        while (--count) {
            *(object*)p = (p + m_elemSize);
            p += m_elemSize;
        }
        *(object*)p = 0;
    }

	~btPoolAllocator()
	{

		btAlignedFree( m_pool);
	}

	int	getFreeCount() const
	{
		return m_freeCount;
	}

	int getUsedCount() const
	{
		return m_maxElements - m_freeCount;
	}

	int getMaxCount() const
	{
		return m_maxElements;
	}

	object	allocate(int size)
	{
		// release mode fix
		(void)size;
		btAssert(!size || size<=m_elemSize);
		btAssert(m_freeCount>0);
        object result = m_firstFree;
        m_firstFree = *(object*)m_firstFree;
        --m_freeCount;
        return result;
	}

	bool validPtr(object ptr)
	{
		if (ptr) {
			if (((string)ptr >= m_pool && (string)ptr < m_pool + m_maxElements * m_elemSize))
			{
				return true;
			}
		}
		return false;
	}

	void	freeMemory(object ptr)
	{
		 if (ptr) {
            btAssert((string)ptr >= m_pool && (string)ptr < m_pool + m_maxElements * m_elemSize);

            *(object*)ptr = m_firstFree;
            m_firstFree = ptr;
            ++m_freeCount;
        }
	}

	int	getElementSize() const
	{
		return m_elemSize;
	}

	string	getPoolAddress()
	{
		return m_pool;
	}

	const string	getPoolAddress() const
	{
		return m_pool;
	}

};

}
