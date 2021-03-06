﻿/*
 * This file is part of Voxelarium.
 *
 * Copyright 2015-2016 James Buckeyne  *** Added 11/22/2015
 *
 * Voxelarium is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Voxelarium is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
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
