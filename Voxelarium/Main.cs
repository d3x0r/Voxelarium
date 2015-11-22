using System;
using System.Collections.Generic;
using System.Text;
using Voxelarium.Core.UI;
using Voxelarium.Core;

namespace Voxelarium
{
    public class Loader
    {
        public static void Main( string [] args )
        {
            Voxelarium.Core.VoxelGameEnvironment game = new Voxelarium.Core.VoxelGameEnvironment();

            Display d = new Display( game );
            d.Run( 60.0 );
        }
    }
}
