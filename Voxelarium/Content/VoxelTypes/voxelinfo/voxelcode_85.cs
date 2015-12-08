using System; // Console.WriteLine
using Voxelarium.Common;

namespace Voxelarium.Core.Voxels
{

	public class WaterType : VoxelType
	{
		public WaterType()
		{
			Console.WriteLine( "Water Type Created" );
		}

		public override bool React( ref VoxelRef self, double tick )
		{
			NearVoxelRef near_below; VoxelRef.GetNearBelowVoxelRef( out near_below, ref self );
			//Log.log( "Reacting water... {0} {1} {2} {3} {4} {5} {6}"
			//	, self.x, self.y, self.z 
			//	, self.Offset, near_below.Offset
			//	, self.Type.properties.Type
			//	, near_below.Type != null?near_below.Type.properties.Type.ToString() :"NoBelow"
			//	);
			if( near_below.Sector != null 
				&& near_below.Type.properties.Is_CanBeReplacedBy_Water )
			{
				//Log.log( "Swap below..." );
				World.Swap( ref self, ref near_below, VoxelSector.ModifiedFieldFlags.UNIMPORTANT );
				near_below.Sector.ModifTracker.Set( near_below.Offset );

				return true;
			}

			// Eau qui coule. Flowing water.
			{
				NearVoxelRef near_left, near_right, near_ahead, near_behind;
				VoxelRef.GetNearLeftVoxelRef( out near_left  , ref self );
				VoxelRef.GetNearRightVoxelRef( out near_right, ref self );
				VoxelRef.GetNearAheadVoxelRef( out near_ahead, ref self );
				VoxelRef.GetNearBehindVoxelRef( out near_behind, ref self );

				int vCount = 0, WaveCount = 0;
				int j;

				bool DirEn0, DirEn1, DirEn2, DirEn3;
				bool WaveDirEn0, WaveDirEn1, WaveDirEn2, WaveDirEn3;

				//self.GetVoxelRefs( out St, out Offsets, true );
				//self.GetVoxelRefs( out nearby, true );


				// Test if we can fall downward
				//cx = x+1 ; cy = y ; cz = z+1; SecondaryOffset[i] = If_x[cx]+If_y[cy]+If_z[cz];St[i] = SectorTable[ Of_x[cx] + Of_y[cy] + Of_z[cz] ]; Vp[i] = &St[i]->Data[ SecondaryOffset[i] ].Data;

				if( near_left.Sector != null &&
					near_left.Type.properties.Is_CanBeReplacedBy_Water )
				{
					VoxelRef.GetNearBelowVoxelRef( out near_below, ref near_left );
					if( near_below.Type != null && near_below.Type.properties.Is_CanBeReplacedBy_Water )
					{ DirEn0 = true; vCount++; }
					else DirEn0 = false;
					WaveCount++; WaveDirEn0 = true;
				}
				else { DirEn0 = false; WaveDirEn0 = false; }

				if( near_right.Sector != null &&
					near_right.Type.properties.Is_CanBeReplacedBy_Water )
				{
					VoxelRef.GetNearBelowVoxelRef( out near_below, ref near_right );
					if( near_below.Type != null && near_below.Type.properties.Is_CanBeReplacedBy_Water )
					{ DirEn1 = true; vCount++; }
					else DirEn1 = false;
					WaveCount++; WaveDirEn1 = true;
				}
				else { DirEn1 = false; WaveDirEn1 = false; }

				if( near_ahead.Sector != null &&
					near_ahead.Type.properties.Is_CanBeReplacedBy_Water )
				{
					VoxelRef.GetNearBelowVoxelRef( out near_below, ref near_ahead );
					if( near_below.Type != null && near_below.Type.properties.Is_CanBeReplacedBy_Water )
					{ DirEn2 = true; vCount++; }
					else DirEn2 = false;
					WaveCount++; WaveDirEn2 = true;
				}
				else { DirEn2 = false; WaveDirEn2 = false; }

				if( near_behind.Sector != null &&
					near_behind.Type.properties.Is_CanBeReplacedBy_Water )
				{
					VoxelRef.GetNearBelowVoxelRef( out near_below, ref near_behind );
					if( near_below.Type != null && near_below.Type.properties.Is_CanBeReplacedBy_Water )
					{ DirEn3 = true; vCount++; }
					else DirEn3 = false;
					WaveCount++; WaveDirEn3 = true;
				}
				else { DirEn3 = false; WaveDirEn3 = false; }

				if( vCount > 0 )
				{
					j = ( VoxelReactor.Random.GetEntropy(2, false) % vCount ) + 1;
					//Log.log( "Reacting Entropy {0} {1}", vCount, j );
					if( DirEn0 )
					{
						j--; if( j == 0 )
						{
							//Log.log( "swap left" );
							World.Swap( ref self, ref near_left, VoxelSector.ModifiedFieldFlags.UNIMPORTANT );
							near_left.Sector.ModifTracker.Set( near_left.Offset );
							return true;
						}
					}
					if( DirEn1 )
					{
						j--; if( j == 0 )
						{
							//Log.log( "swap right" );
							World.Swap( ref self, ref near_right, VoxelSector.ModifiedFieldFlags.UNIMPORTANT );
							near_right.Sector.ModifTracker.Set( near_right.Offset );
							return true;
						}
					}
					if( DirEn2 )
					{
						j--; if( j == 0 )
						{
							//Log.log( "swap ahead" );
							World.Swap( ref self, ref near_ahead, VoxelSector.ModifiedFieldFlags.UNIMPORTANT );
							near_ahead.Sector.ModifTracker.Set( near_ahead.Offset );
							return true;
						}
					}
					if( DirEn3 )
					{
						j--; if( j == 0 )
						{
							//Log.log( "swap behind" );
							World.Swap( ref self, ref near_behind, VoxelSector.ModifiedFieldFlags.UNIMPORTANT );
							near_behind.Sector.ModifTracker.Set( near_behind.Offset );
							return true;
						}
					}
				}
				else if( WaveCount > 0 && WaveCount < 4 )
				{
					j = ( VoxelReactor.Random.GetEntropy( 2, false ) % WaveCount ) + 1;
					//Log.log( "Reacting Wave {0} {1}", WaveCount, j );
					if( WaveDirEn0 )
					{
						//Log.log( "swap left 0" );
						j--; if( j == 0 )
						{
							World.Swap( ref self, ref near_left, VoxelSector.ModifiedFieldFlags.UNIMPORTANT );
							near_left.Sector.ModifTracker.Set( near_left.Offset );
							return true;
						}
					}
					if( WaveDirEn1 )
					{
						//Log.log( "swap right 1" );
						j--; if( j == 0 )
						{
							World.Swap( ref self, ref near_right, VoxelSector.ModifiedFieldFlags.UNIMPORTANT );
							near_right.Sector.ModifTracker.Set( near_right.Offset );
							return true;
						}
					}
					if( WaveDirEn2 )
					{
						//Log.log( "swap ahead {0} {1}", World, near_ahead.Sector );
						j--; if( j == 0 )
						{
							//Log.log( "going to swap....." );
							World.Swap( ref self, ref near_ahead, VoxelSector.ModifiedFieldFlags.UNIMPORTANT );
							//Log.log( "... {0}", near_ahead.Sector.ModifTracker );
							near_ahead.Sector.ModifTracker.Set( near_ahead.Offset );
							return true;
						}
					}
					if( WaveDirEn3 )
					{
						//Log.log( "swap behind" );
						j--; if( j == 0 )
						{
							World.Swap( ref self, ref near_behind, VoxelSector.ModifiedFieldFlags.UNIMPORTANT );
							near_behind.Sector.ModifTracker.Set( near_behind.Offset );							
							return true;
						}
					}
				}
			}
			return false;
		}
	}
	/*
	public class WaterExtension : VoxelExtension
	{
		public WaterExtension()
		{
		}
	}
	*/
}

