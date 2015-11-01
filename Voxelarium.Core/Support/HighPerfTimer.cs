using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

namespace Voxelarium.Core.Support
{
	public struct HighPerfTimer
	{
		long Result;
		System.Diagnostics.Stopwatch sw;
		static Stopwatch core = new Stopwatch();

		public static long GetActualTime() // Return time counter in microseconds
		{
			core.Start();
			return core.ElapsedTicks;
		}
		public void Start() { if( sw == null ) sw = new System.Diagnostics.Stopwatch(); sw.Reset(); sw.Start(); }
		public void End()
		{
			sw.Stop();
			Result = sw.ElapsedTicks;
		}

		public int GetResult() { return (int)Result; }
		public long GetLongResult() { return Result; }
	}
}
