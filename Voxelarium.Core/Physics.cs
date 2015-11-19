using BEPUphysics;
using BEPUutilities;
using BEPUutilities.Threading;
using System;
using System.Collections.Generic;
using System.Text;
//using BulletSharp;
namespace Voxelarium.Core
{
    public class Physics
    {
        Space world;
		public Physics()
        {
			ParallelLooper parallelLooper = new ParallelLooper();
			for( int i = 0; i < ( Environment.ProcessorCount - 3 ); i++ )
			{
				parallelLooper.AddThread();
			}

			world = new Space();
			world.ForceUpdater.Gravity = new Vector3( 0, -9.81f, 0 );

		}

	}
}
