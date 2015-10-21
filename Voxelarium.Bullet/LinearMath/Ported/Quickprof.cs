/*

****************************************[]
*
* profile.cpp
*
* Real-Time Hierarchical Profiling for Game Programming Gems 3
*
* by Greg Hjelstrom & Byon Garrabrant
*
****************************************[]/

// Credits: The Clock class was inspired by the Timer classes in
// Ogre (www.ogre3d.org).


/*****************************************
*
* Real-Time Hierarchical Profiling for Game Programming Gems 3
*
* by Greg Hjelstrom & Byon Garrabrant
*
****************************************[]/

// Credits: The Clock class was inspired by the Timer classes in 
// Ogre (www.ogre3d.org).


//To disable built-in profiling, please comment out next line
//#define BT_NO_PROFILE 1
#if !BT_NO_PROFILE

using System;
using System.Diagnostics;

namespace Bullet.LinearMath
{
	///The btClock is a portable basic clock that measures accurate time in seconds, use for profiling.
	public class btClock
	{
		internal static btClock gProfileClock = new btClock();
		internal static uint tickRate;
		Stopwatch m_data;
		public btClock()
		{
			m_data = new Stopwatch();
			m_data.Start();
			tickRate = (uint)( 1E9 / Stopwatch.Frequency );
			//Console.WriteLine( "The minimum measurable time on this system is: {0} nanoseconds", resolution );
		}
		/*
		public btClock( btClock other)
			{
			m_data = new Stopwatch();
            }
			*/
		~btClock()
		{
			m_data = null;
		}

		/// Resets the initial reference time.
		internal void reset()
		{
			m_data.Reset();
		}

		/// Returns the time in ms since the last call to reset or since 
		/// the btClock was created.
		public long getTimeMilliseconds()
		{
			return m_data.ElapsedMilliseconds;
		}

		/// Returns the time in us since the last call to reset or since 
		/// the Clock was created.
		public ulong getTimeMicroseconds()
		{
			return (ulong)m_data.ElapsedTicks;
		}

		/// Returns the time in s since the last call to reset or since 
		/// the Clock was created.
		public double getTimeSeconds()
		{
			return m_data.Elapsed.TotalSeconds;
		}

	};

	///A node in the Profile Hierarchy Tree
	class CProfileNode
	{

		/**************************************

		  INPUT:                                                                                      

		  name - pointer to a static string which is the name of this profile node                    

		  parent - parent pointer                                                                     

		                                                                                              

		  WARNINGS:                                                                                   

		  The name is assumed to be a static pointer, only the pointer is stored and compared for     

		  efficiency reasons.                                                                         

		 =============================================================================================*/
		public CProfileNode( string name, CProfileNode parent )
		{
			Name = name;
			TotalCalls = 0;
			TotalTime = 0;
			StartTime = 0;
			RecursionCounter = 0;
			Parent = parent;
			Child = null;
			Sibling = null;
			m_userPtr = null;
			Reset();
		}
		~CProfileNode()
		{
			CleanupMemory();
		}

		/**************************************

		  INPUT:                                                                                      

		  name - static string pointer to the name of the node we are searching for                   

		                                                                                              

		  WARNINGS:                                                                                   

		  All profile names are assumed to be static strings so this function uses pointer compares   

		  to find the named node.                                                                     

		 =============================================================================================*/
		public CProfileNode Get_Sub_Node( string name )
		{
			// Try to find this sub node
			CProfileNode child = Child;
			while( child != null )
			{
				if( String.Compare( child.Name, name ) == 0 )
				{
					return child;
				}
				child = child.Sibling;
			}
			// We didn't find it, so add it

			CProfileNode node = new CProfileNode( name, this );
			node.Sibling = Child;
			Child = node;
			return node;
		}

		public CProfileNode Get_Parent() { return Parent; }
		public CProfileNode Get_Sibling() { return Sibling; }
		public CProfileNode Get_Child() { return Child; }

		public void CleanupMemory()
		{
			Child = null;
			Sibling = null;
		}

		public void Reset()
		{
			TotalCalls = 0;
			TotalTime = 0.0f;


			if( Child != null )
			{
				Child.Reset();
			}
			if( Sibling != null )
			{
				Sibling.Reset();
			}
		}
		internal static ulong Profile_Get_Ticks()
		{
			return btClock.gProfileClock.getTimeMicroseconds();
		}

		public void Call()
		{
			TotalCalls++;
			if( RecursionCounter++ == 0 )
			{
				StartTime = Profile_Get_Ticks();
			}
		}

		float Profile_Get_Tick_Rate()
		{
			//	return 1000000f;
			return btClock.tickRate;

		}

		public bool Return()
		{
			if( --RecursionCounter == 0 && TotalCalls != 0 )
			{
				ulong time;
				time = Profile_Get_Ticks();
				time -= StartTime;
				TotalTime += time;
			}
			return ( RecursionCounter == 0 );
		}

		internal string Get_Name()
		{
			return Name;
		}
		internal int Get_Total_Calls() { return TotalCalls; }
		internal float Get_Total_Time() { return TotalTime; }
		internal object GetUserPointer() { return m_userPtr; }
		internal void SetUserPointer( object ptr ) { m_userPtr = ptr; }

		protected string Name;
		protected int TotalCalls;
		protected float TotalTime;
		protected ulong StartTime;
		protected int RecursionCounter;

		protected CProfileNode Parent;
		protected CProfileNode Child;
		protected CProfileNode Sibling;
		protected object m_userPtr;
	}

	///An iterator to navigate through the tree
	class CProfileIterator
	{

		// Access all the children of the current parent
		public void First()
		{
			CurrentChild = CurrentParent.Get_Child();
		}
		public void Next()
		{
			CurrentChild = CurrentChild.Get_Sibling();
		}
		public bool Is_Done()
		{
			return CurrentChild == null;
		}
		public bool Is_Root() { return ( CurrentParent.Get_Parent() == null ); }

		public void Enter_Child( int index )      // Make the given child the new parent
		{
			CurrentChild = CurrentParent.Get_Child();
			while( ( CurrentChild != null ) && ( index != 0 ) )
			{
				index--;
				CurrentChild = CurrentChild.Get_Sibling();
			}

			if( CurrentChild != null )
			{
				CurrentParent = CurrentChild;
				CurrentChild = CurrentParent.Get_Child();
			}
		}
		public void Enter_Largest_Child()   // Make the largest child the new parent
		{
		}
		public void Enter_Parent()          // Make the current parent's parent the new parent
		{
			if( CurrentParent.Get_Parent() != null )
			{
				CurrentParent = CurrentParent.Get_Parent();
			}
			CurrentChild = CurrentParent.Get_Child();
		}

		// Access the current child
		public string Get_Current_Name() { return CurrentChild.Get_Name(); }
		public int Get_Current_Total_Calls() { return CurrentChild.Get_Total_Calls(); }
		public float Get_Current_Total_Time() { return CurrentChild.Get_Total_Time(); }

		public object Get_Current_UserPointer() { return CurrentChild.GetUserPointer(); }
		public void Set_Current_UserPointer( object ptr ) { CurrentChild.SetUserPointer( ptr ); }
		// Access the current parent
		public string Get_Current_Parent_Name() { return CurrentParent.Get_Name(); }
		public int Get_Current_Parent_Total_Calls() { return CurrentParent.Get_Total_Calls(); }
		public float Get_Current_Parent_Total_Time() { return CurrentParent.Get_Total_Time(); }




		protected CProfileNode CurrentParent;
		protected CProfileNode CurrentChild;


		internal CProfileIterator( CProfileNode start )
		{
			CurrentParent = start;
			CurrentChild = CurrentParent.Get_Child();
		}
	};


	///The Manager for the Profile system
	class CProfileManager
	{

		/**************************************

		  CProfileManager::Start_Profile -- Begin a named profile                                    

		                                                                                              

		  Steps one level deeper into the tree, if a child already exists with the specified name     

		  then it accumulates the profiling; otherwise a new child node is added to the profile tree. 

		                                                                                              

		  INPUT:                                                                                      

		  name - name of this profiling record                                                        

		                                                                                              

		  WARNINGS:                                                                                   

		  The string used is assumed to be a static string; pointer compares are used throughout      

		  the profiling code for efficiency.                                                          

		 =============================================================================================*/
		public static void Start_Profile( string name )
		{
			if( String.Compare( name, CurrentNode.Get_Name() ) == 0 )
			{
				CurrentNode = CurrentNode.Get_Sub_Node( name );
			}

			CurrentNode.Call();
		}
		public static void Stop_Profile()
		{
			// Return will indicate whether we should back up to our parent (we may
			// be profiling a recursive function)
			if( CurrentNode.Return() )
			{
				CurrentNode = CurrentNode.Get_Parent();
			}
		}

		public static void CleanupMemory()
		{
			Root.CleanupMemory();
		}

		/**************************************

		  CProfileManager::Reset -- Reset the contents of the profiling system                       

		                                                                                              

		     This resets everything except for the tree structure.  All of the timing data is reset.  

		 =============================================================================================*/
		public static void Reset()
		{
			btClock.gProfileClock.reset();
			Root.Reset();
			Root.Call();
			FrameCounter = 0;
			ResetTime = CProfileNode.Profile_Get_Ticks(  );
		}
		/**************************************

		  CProfileManager::Increment_Frame_Counter -- Increment the frame counter                    

		 =============================================================================================*/
		public static void Increment_Frame_Counter()
		{
			FrameCounter++;
		}
		public static int Get_Frame_Count_Since_Reset() { return FrameCounter; }
		public static float Get_Time_Since_Reset()
		{
			ulong time;
			time = CProfileNode.Profile_Get_Ticks();
			time -= ResetTime;
			return time;
		}

		public static CProfileIterator Get_Iterator()
		{
			return new CProfileIterator( Root );
		}
		public static void Release_Iterator( CProfileIterator iterator ) { /*delete iterator;*/ }

		public static void dumpRecursive( CProfileIterator profileIterator, int spacing )
		{
			profileIterator.First();
			if( profileIterator.Is_Done() )
				return;

			float accumulated_time = 0, parent_time = profileIterator.Is_Root() ? CProfileManager.Get_Time_Since_Reset() : profileIterator.Get_Current_Parent_Total_Time();
			int i;
			int frames_since_reset = CProfileManager.Get_Frame_Count_Since_Reset();
			for( i = 0; i < spacing; i++ ) Console.WriteLine( "." );
			Console.WriteLine( "----------------------------------\n" );
			for( i = 0; i < spacing; i++ ) Console.WriteLine( "." );
			Console.WriteLine( "Profiling: " + profileIterator.Get_Current_Parent_Name()
				+ " (total running time: " + parent_time + " ms) ---\n" );
			float totalTime = 0;


			int numChildren = 0;

			for( i = 0; !profileIterator.Is_Done(); i++, profileIterator.Next() )
			{
				numChildren++;
				float current_total_time = profileIterator.Get_Current_Total_Time();
				accumulated_time += current_total_time;
				float fraction = parent_time > btScalar.SIMD_EPSILON ? ( current_total_time / parent_time ) * 100 : 0;
				{
					int s; for( s = 0; s < spacing; s++ ) Console.WriteLine( "." );
				}
				Console.WriteLine( 
					i
					+ "-- "
					+ profileIterator.Get_Current_Name()
					+ "(" 
					+ fraction
					+ " %) :: "
					+ ( current_total_time / (double)frames_since_reset )
					+ " ms / frame("
					+ profileIterator.Get_Current_Total_Calls() 
					+ " calls )"
					);
				totalTime += current_total_time;
				//recurse into children
			}

			if( parent_time < accumulated_time )
			{
				//Console.WriteLine("what's wrong\n");
			}
			for( i = 0; i < spacing; i++ ) Console.WriteLine( "." );
			Console.WriteLine( "Unaccounted:  ("
				+ ( parent_time > btScalar.SIMD_EPSILON 
				  ? ( ( parent_time - accumulated_time ) / parent_time ) * 100 
				  : 0 )
				+ " %%) :: "
				+ ( parent_time - accumulated_time )
				+ " ms\n" );

			for( i = 0; i < numChildren; i++ )
			{
				profileIterator.Enter_Child( i );
				dumpRecursive( profileIterator, spacing + 3 );
				profileIterator.Enter_Parent();
			}
		}

		public static void dumpAll()
		{
			CProfileIterator profileIterator = null;
			profileIterator = CProfileManager.Get_Iterator();

			dumpRecursive( profileIterator, 0 );

			CProfileManager.Release_Iterator( profileIterator );
		}


		static CProfileNode Root = new CProfileNode( "Root", null );
		static CProfileNode CurrentNode = Root;
		static int FrameCounter = 0;
		static ulong ResetTime = 0;
	};


	///ProfileSampleClass is a simple way to profile a function's scope
	///Use the CProfileSample sample = new CProfileSample macro at the start of scope to time
	class CProfileSample : CProfileManager
	{
		public
	CProfileSample( string name )
		{
			CProfileManager.Start_Profile( name );
		}

		~CProfileSample()
		{
			CProfileManager.Stop_Profile();
		}
	}

	//#define CProfileSample sample = new CProfileSample( name )			CProfileSample __profile( name )


#endif //BT_NO_PROFILE
}

