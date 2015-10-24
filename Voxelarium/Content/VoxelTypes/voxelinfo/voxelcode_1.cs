using System;
namespace Voxelarium.Core.Voxels
{

public class BlackRockType : VoxelType {
	
	public BlackRockType(  )
        {
	        Console.WriteLine( "tmp" );
        }
        
        public override bool React( ref VoxelRef self, double tick )
        {    	
        	return false;
        }
}

public class BlackRockExtension: VoxelExtension {
	BlackRockExtension()
        {
        }
}

}
