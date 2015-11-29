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

            GameDisplay d = new GameDisplay( game );
            d.Run( 60.0 );
        }
    }
}
