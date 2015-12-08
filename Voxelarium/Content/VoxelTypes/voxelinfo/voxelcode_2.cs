using System;
namespace Voxelarium.Core.Voxels
{

public class OrangeRock : VoxelType {
	
	public OrangeRock(  )
        {
	        Console.WriteLine( "tmp OrangeRock" );
        }
        
        public override bool React( ref VoxelRef self, double tick )
        {    	
        	return false;
        }
}
	/*
public class OrangeRockExtension: VoxelExtension {
	OrangeRockExtension()
        {
        }
}
*/
}
