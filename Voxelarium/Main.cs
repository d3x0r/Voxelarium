using System;
using System.Collections.Generic;
using System.Text;
using Voxelarium.Core.Graphics;
using Voxelarium.Core;

namespace Voxelarium
{
    public class Loader
    {
        public static void Main( string [] args )
        {
            Physics p = new Physics();

            Display d = new Display();
            d.Run( 60.0 );
        }
    }
}
