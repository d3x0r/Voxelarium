using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

namespace Voxelarium.Core.Support
{
	public struct HighPerfTimer
	{
		long StartTime;
		long EndTime;
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
			EndTime = sw.ElapsedTicks;
			Result = EndTime - StartTime;
			if( StartTime > EndTime ) Result = 1; // Workaround for timechange causing problems.
		}

		public long GetResult() { return Result; }
		public long GetLongResult() { return Result; }
	}
}
