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

/*
StackAlloc extracted from GJK-EPA collision solver by Nathanael Presson
Nov.2006
*/

#if ! BT_STACK_ALLOC
#define BT_STACK_ALLOC

#include "double.h" //for Debug.Assert
#include "btAlignedAllocator.h"

///The btBlock class is an internal structure for the btStackAlloc memory allocator.
struct btBlock
{
	btBlock*			previous;
	string 	address;
};

///The StackAlloc class provides some fast stack-based memory allocator (LIFO last-in first-out)
class btStackAlloc
{
public:

	btStackAlloc(uint size)	{ ctor();create(size); }
	~btStackAlloc()		{ destroy(); }
	
	inline void		create(uint size)
	{
		destroy();
		data		=  (string  btAlignedAlloc(size,16);
		totalsize	=	size;
	}
	inline void		destroy()
	{
		Debug.Assert(usedsize==0);
		//Raise(L"StackAlloc is still in use");

		if(usedsize==0)
		{
			if(!ischild && data)		
				btAlignedFree(data);

			data				=	0;
			usedsize			=	0;
		}
		
	}

	int	getAvailableMemory()
	{
		return static_cast<int>(totalsize - usedsize);
	}

	string 		allocate(uint size)
	{
		string nsigned int	nus(usedsize+size);
		if(nus<totalsize)
		{
			usedsize=nus;
			return(data+(usedsize-size));
		}
		Debug.Assert(false);
		//&& (L"Not enough memory"));
		
		return(0);
	}
	public btBlock*		beginBlock()
	{
		btBlock*	pb = (btBlock*)allocate(sizeof(btBlock));
		pb.previous	=	current;
		pb.address		=	data+usedsize;
		current			=	pb;
		return(pb);
	}
	public void		endBlock(btBlock* block)
	{
		Debug.Assert(block==current);
		//Raise(L"Unmatched blocks");
		if(block==current)
		{
			current		=	block.previous;
			usedsize	=	(uint)((block.address-data)-sizeof(btBlock));
		}
	}

private:
	void		ctor()
	{
		data		=	0;
		totalsize	=	0;
		usedsize	=	0;
		current		=	0;
		ischild		=	false;
	}
	string 	data;
	uint		totalsize;
	uint		usedsize;
	btBlock*	current;
	bool		ischild;
};

#endif //BT_STACK_ALLOC
