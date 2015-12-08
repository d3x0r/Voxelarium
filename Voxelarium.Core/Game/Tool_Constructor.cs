using OpenTK;
using OpenTK.Input;
using System;
using System.Collections.Generic;
using Voxelarium.Core.Game.GameWindows;
using Voxelarium.Core.Voxels;
using Voxelarium.Core.Voxels.Types;
using Voxelarium.LinearMath;

namespace Voxelarium.Core.Game
{
	internal class Tool_Constructor : Tool_Manager.Tool
	{
		//protected:
		int OldToolNum;
		bool MiningInProgress;
		VoxelCoords MinedVoxel;
		float Mining_MaterialResistanceCounter;
		internal float[] ToolForce = new float[Tool_Manager.TOOL_TOOLTYPESNUMBER];
		internal bool[] ToolCompatibleTypes = new bool[Tool_Manager.TOOL_TOOLTYPESNUMBER];
		//void* SoundHandle;

		public Tool_Constructor()
		{
			int i;
			OldToolNum = -1;
			MiningInProgress = false;
			for( i = 0; i < Tool_Manager.TOOL_TOOLTYPESNUMBER; i++ ) ToolForce[i] = 0;
			for( i = 0; i < Tool_Manager.TOOL_TOOLTYPESNUMBER; i++ ) ToolCompatibleTypes[i] = false;
			ToolForce[1] = 0.8f;
			ToolCompatibleTypes[1] = true;
			Mining_MaterialResistanceCounter = 0;
			//SoundHandle = 0;
		}

		internal override bool Tool_MouseButtonClick( MouseButton Button )
		{
			Actor Actor;

			Actor = GameEnv.GetActiveActor();

			// printf("Click : %ld\n",Button);

			if( Actor != null )
			{
				switch( Button )
				{
				case MouseButton.Button2: // Right mouse button
					if( Actor.PointedVoxel.Collided )
					{
						Inventory.Entry InventorySlot;
						ushort VoxelType;
						//ULong  OtherInfos;
						btVector3 VoxelCenter, VoxelDistance;
						bool IsOnGround; ;


						// Ensure you are not to close to the voxel position you want to build.
						bool NotTooClose;
						NotTooClose = true;
						Actor.PointedVoxel.PredPointedVoxel.GetVoxelCenterCoords( out VoxelCenter );
						VoxelCenter.Sub( ref Actor.ViewDirection.m_origin, out VoxelDistance );
						//printf("VoxelDistance : %lf,%lf,%lf\n", VoxelDistance.x, VoxelDistance.y, VoxelDistance.z);
						if( VoxelDistance.y < 512.0 && VoxelDistance.y > -127.0 )
						{
							if( ( VoxelDistance.x < 203.0 && VoxelDistance.x > -203.0 )
								 && ( VoxelDistance.z < 203.0 && VoxelDistance.z > -203.0 ) )
							{ NotTooClose = false; GameEnv.GameWindow_Advertising.Advertise( "TOO CLOSE", GameWindow_Advertising.Visibility.VISIBILITY_HIGH, 0, 2000, 1000 ); }
						}



						InventorySlot = Actor.Inventory.GetActualItemSlot();
						VoxelType = InventorySlot.VoxelType;
						//OtherInfos = 0;
						// printf("Location (%lf,%lf,%lf) Voxel (%lf,%lf,%lf)\n", Actor.Location.x,Actor.Location.y,Actor.Location.z, VoxelCenter.x, VoxelCenter.y, VoxelCenter.z);

#if COMPILEOPTION_ALLOWJUMPANDBUILD
						IsOnGround = true;
#else
						IsOnGround = Actor.IsOnGround;
#endif


						if( VoxelType > 0 && InventorySlot.Quantity > 0 && NotTooClose && IsOnGround )
						{
							if( 1 == Actor.Inventory.UnstoreBlocks( VoxelType, 1 ) )
							{
								//VoxelExtensionType = GameEnv.VoxelTypeManager.GetVoxelType(VoxelType).ExtensionType;
								//if (VoxelExtensionType!=0) OtherInfos = (ULong)GameEnv.World.ExtensionFactory.CreateVoxelExtension(VoxelExtensionType);
								//GameEnv.World.SetVoxel_WithCullingUpdate(Actor.PointedVoxel.PredPointedVoxel.X, Actor.PointedVoxel.PredPointedVoxel.Y, Actor.PointedVoxel.PredPointedVoxel.Z, VoxelType, OtherInfos);

								// new

								VoxelRef VLoc;
								if( GameEnv.World.SetVoxel_WithCullingUpdate( Actor.PointedVoxel.PredPointedVoxel.X
																			, Actor.PointedVoxel.PredPointedVoxel.Y
																			, Actor.PointedVoxel.PredPointedVoxel.Z
																			, VoxelType
																			, VoxelSector.ModifiedFieldFlags.CRITICAL, true
																			, out VLoc ) )
									VLoc.Sector.Flag_HighPriorityRefresh = true;
								if( false )
								{
									int dx = Actor.PointedVoxel.PredPointedVoxel.X - Actor.PointedVoxel.PointedVoxel.X;
									int dy = Actor.PointedVoxel.PredPointedVoxel.Y - Actor.PointedVoxel.PointedVoxel.Y;
									int dz = Actor.PointedVoxel.PredPointedVoxel.Z - Actor.PointedVoxel.PointedVoxel.Z;
									if( dx != 0 )
									{
										if( GameEnv.World.SetVoxel_WithCullingUpdate( Actor.PointedVoxel.PredPointedVoxel.X + dx * 3, Actor.PointedVoxel.PredPointedVoxel.Y, Actor.PointedVoxel.PredPointedVoxel.Z, VoxelType, VoxelSector.ModifiedFieldFlags.CRITICAL, true, out VLoc ) )
											VLoc.Sector.Flag_HighPriorityRefresh = true;
										if( GameEnv.World.SetVoxel_WithCullingUpdate( Actor.PointedVoxel.PredPointedVoxel.X + dx * 3 + 1, Actor.PointedVoxel.PredPointedVoxel.Y, Actor.PointedVoxel.PredPointedVoxel.Z, VoxelType, VoxelSector.ModifiedFieldFlags.CRITICAL, true, out VLoc ) )
											VLoc.Sector.Flag_HighPriorityRefresh = true;
										if( GameEnv.World.SetVoxel_WithCullingUpdate( Actor.PointedVoxel.PredPointedVoxel.X + dx * 3 - 1, Actor.PointedVoxel.PredPointedVoxel.Y, Actor.PointedVoxel.PredPointedVoxel.Z, VoxelType, VoxelSector.ModifiedFieldFlags.CRITICAL, true, out VLoc ) )
											VLoc.Sector.Flag_HighPriorityRefresh = true;
										if( GameEnv.World.SetVoxel_WithCullingUpdate( Actor.PointedVoxel.PredPointedVoxel.X + dx * 3, Actor.PointedVoxel.PredPointedVoxel.Y + 1, Actor.PointedVoxel.PredPointedVoxel.Z, VoxelType, VoxelSector.ModifiedFieldFlags.CRITICAL, true, out VLoc ) )
											VLoc.Sector.Flag_HighPriorityRefresh = true;
										if( GameEnv.World.SetVoxel_WithCullingUpdate( Actor.PointedVoxel.PredPointedVoxel.X + dx * 3, Actor.PointedVoxel.PredPointedVoxel.Y - 1, Actor.PointedVoxel.PredPointedVoxel.Z, VoxelType, VoxelSector.ModifiedFieldFlags.CRITICAL, true, out VLoc ) )
											VLoc.Sector.Flag_HighPriorityRefresh = true;
										if( GameEnv.World.SetVoxel_WithCullingUpdate( Actor.PointedVoxel.PredPointedVoxel.X + dx * 3, Actor.PointedVoxel.PredPointedVoxel.Y, Actor.PointedVoxel.PredPointedVoxel.Z + 1, VoxelType, VoxelSector.ModifiedFieldFlags.CRITICAL, true, out VLoc ) )
											VLoc.Sector.Flag_HighPriorityRefresh = true;
										if( GameEnv.World.SetVoxel_WithCullingUpdate( Actor.PointedVoxel.PredPointedVoxel.X + dx * 3, Actor.PointedVoxel.PredPointedVoxel.Y, Actor.PointedVoxel.PredPointedVoxel.Z - 1, VoxelType, VoxelSector.ModifiedFieldFlags.CRITICAL, true, out VLoc ) )
											VLoc.Sector.Flag_HighPriorityRefresh = true;
									}
									if( dy != 0 )
									{
										if( GameEnv.World.SetVoxel_WithCullingUpdate( Actor.PointedVoxel.PredPointedVoxel.X, Actor.PointedVoxel.PredPointedVoxel.Y + dy * 3, Actor.PointedVoxel.PredPointedVoxel.Z, VoxelType, VoxelSector.ModifiedFieldFlags.CRITICAL, true, out VLoc ) )
											VLoc.Sector.Flag_HighPriorityRefresh = true;
										if( GameEnv.World.SetVoxel_WithCullingUpdate( Actor.PointedVoxel.PredPointedVoxel.X + 1, Actor.PointedVoxel.PredPointedVoxel.Y + dy * 3, Actor.PointedVoxel.PredPointedVoxel.Z, VoxelType, VoxelSector.ModifiedFieldFlags.CRITICAL, true, out VLoc ) )
											VLoc.Sector.Flag_HighPriorityRefresh = true;
										if( GameEnv.World.SetVoxel_WithCullingUpdate( Actor.PointedVoxel.PredPointedVoxel.X - 1, Actor.PointedVoxel.PredPointedVoxel.Y + dy * 3, Actor.PointedVoxel.PredPointedVoxel.Z, VoxelType, VoxelSector.ModifiedFieldFlags.CRITICAL, true, out VLoc ) )
											VLoc.Sector.Flag_HighPriorityRefresh = true;
										if( GameEnv.World.SetVoxel_WithCullingUpdate( Actor.PointedVoxel.PredPointedVoxel.X, Actor.PointedVoxel.PredPointedVoxel.Y + 1 + dy * 3, Actor.PointedVoxel.PredPointedVoxel.Z, VoxelType, VoxelSector.ModifiedFieldFlags.CRITICAL, true, out VLoc ) )
											VLoc.Sector.Flag_HighPriorityRefresh = true;
										if( GameEnv.World.SetVoxel_WithCullingUpdate( Actor.PointedVoxel.PredPointedVoxel.X, Actor.PointedVoxel.PredPointedVoxel.Y - 1 + dy * 3, Actor.PointedVoxel.PredPointedVoxel.Z, VoxelType, VoxelSector.ModifiedFieldFlags.CRITICAL, true, out VLoc ) )
											VLoc.Sector.Flag_HighPriorityRefresh = true;
										if( GameEnv.World.SetVoxel_WithCullingUpdate( Actor.PointedVoxel.PredPointedVoxel.X, Actor.PointedVoxel.PredPointedVoxel.Y + 0 + dy * 3, Actor.PointedVoxel.PredPointedVoxel.Z + 1, VoxelType, VoxelSector.ModifiedFieldFlags.CRITICAL, true, out VLoc ) )
											VLoc.Sector.Flag_HighPriorityRefresh = true;
										if( GameEnv.World.SetVoxel_WithCullingUpdate( Actor.PointedVoxel.PredPointedVoxel.X, Actor.PointedVoxel.PredPointedVoxel.Y + 0 + dy * 3, Actor.PointedVoxel.PredPointedVoxel.Z - 1, VoxelType, VoxelSector.ModifiedFieldFlags.CRITICAL, true, out VLoc ) )
											VLoc.Sector.Flag_HighPriorityRefresh = true;
									}
									if( dz != 0 )
									{
										if( GameEnv.World.SetVoxel_WithCullingUpdate( Actor.PointedVoxel.PredPointedVoxel.X, Actor.PointedVoxel.PredPointedVoxel.Y, Actor.PointedVoxel.PredPointedVoxel.Z, VoxelType, VoxelSector.ModifiedFieldFlags.CRITICAL, true, out VLoc ) )
											VLoc.Sector.Flag_HighPriorityRefresh = true;
										if( GameEnv.World.SetVoxel_WithCullingUpdate( Actor.PointedVoxel.PredPointedVoxel.X + 1, Actor.PointedVoxel.PredPointedVoxel.Y, Actor.PointedVoxel.PredPointedVoxel.Z, VoxelType, VoxelSector.ModifiedFieldFlags.CRITICAL, true, out VLoc ) )
											VLoc.Sector.Flag_HighPriorityRefresh = true;
										if( GameEnv.World.SetVoxel_WithCullingUpdate( Actor.PointedVoxel.PredPointedVoxel.X - 1, Actor.PointedVoxel.PredPointedVoxel.Y, Actor.PointedVoxel.PredPointedVoxel.Z, VoxelType, VoxelSector.ModifiedFieldFlags.CRITICAL, true, out VLoc ) )
											VLoc.Sector.Flag_HighPriorityRefresh = true;
										if( GameEnv.World.SetVoxel_WithCullingUpdate( Actor.PointedVoxel.PredPointedVoxel.X, Actor.PointedVoxel.PredPointedVoxel.Y + 1, Actor.PointedVoxel.PredPointedVoxel.Z, VoxelType, VoxelSector.ModifiedFieldFlags.CRITICAL, true, out VLoc ) )
											VLoc.Sector.Flag_HighPriorityRefresh = true;
										if( GameEnv.World.SetVoxel_WithCullingUpdate( Actor.PointedVoxel.PredPointedVoxel.X, Actor.PointedVoxel.PredPointedVoxel.Y - 1, Actor.PointedVoxel.PredPointedVoxel.Z, VoxelType, VoxelSector.ModifiedFieldFlags.CRITICAL, true, out VLoc ) )
											VLoc.Sector.Flag_HighPriorityRefresh = true;
										if( GameEnv.World.SetVoxel_WithCullingUpdate( Actor.PointedVoxel.PredPointedVoxel.X, Actor.PointedVoxel.PredPointedVoxel.Y + 0, Actor.PointedVoxel.PredPointedVoxel.Z + 1, VoxelType, VoxelSector.ModifiedFieldFlags.CRITICAL, true, out VLoc ) )
											VLoc.Sector.Flag_HighPriorityRefresh = true;
										if( GameEnv.World.SetVoxel_WithCullingUpdate( Actor.PointedVoxel.PredPointedVoxel.X, Actor.PointedVoxel.PredPointedVoxel.Y + 0, Actor.PointedVoxel.PredPointedVoxel.Z - 1, VoxelType, VoxelSector.ModifiedFieldFlags.CRITICAL, true, out VLoc ) )
											VLoc.Sector.Flag_HighPriorityRefresh = true;
									}
								}

								GameEnv.Sound.PlaySound( 7 );
							}
						}
					}
					break;

				case MouseButton.Button5: // Wheel Button
					if( Actor.PointedVoxel.Collided )
					{
						VoxelExtension OtherInfos;
						ushort VoxelType;
						int x, y, z;
						x = Actor.PointedVoxel.PointedVoxel.X;
						y = Actor.PointedVoxel.PointedVoxel.Y;
						z = Actor.PointedVoxel.PointedVoxel.Z;
						VoxelType = GameEnv.World.GetVoxelExt( x, y, z, out OtherInfos );
						GameEnv.VoxelTypeManager.GetVoxelType( VoxelType ).UserAction_Activate( OtherInfos, x, y, z );
						// printf("Location x:%ld y:%ld z:%ld \n", x, y, z);
					}
					break;
				case MouseButton.Button1: // Left button
					{
						ushort Voxel;
						VoxelType VoxelType;

						if( Actor.PointedVoxel.Collided )
						{
							Voxel = GameEnv.World.GetVoxel( Actor.PointedVoxel.PointedVoxel.X, Actor.PointedVoxel.PointedVoxel.Y, Actor.PointedVoxel.PointedVoxel.Z );
							VoxelType = GameEnv.VoxelTypeManager.GetVoxelType( Voxel );
							if( ToolCompatibleTypes[VoxelType.properties.MiningType] )
							{
								Mining_MaterialResistanceCounter = VoxelType.properties.MiningHardness;
								MiningInProgress = true;
								MinedVoxel = Actor.PointedVoxel.PointedVoxel;
								GameEnv.GameProgressBar.SetCompletion( 0.0f );
								GameEnv.GameProgressBar.Show();
#if COMPILEOPTION_FNX_SOUNDS_1
								if( SoundHandle == 0 ) SoundHandle = GameEnv.Sound.Start_PlaySound( 5, true, true, 1.0, 0 );
#endif
							}
							else
							{
								GameEnv.GameWindow_Advertising.Advertise( "TOO HARD", GameWindow_Advertising.Visibility.VISIBILITY_MEDIUM, 1, 1000, 200 );
							}
						}
						/*
						UShort VoxelType;
						VoxelType = GameEnv.World.GetVoxel(Actor.PointedVoxel.PointedVoxel.x, Actor.PointedVoxel.PointedVoxel.y, Actor.PointedVoxel.PointedVoxel.z);
						GameEnv.World.SetVoxel_WithCullingUpdate(Actor.PointedVoxel.PointedVoxel.x, Actor.PointedVoxel.PointedVoxel.y, Actor.PointedVoxel.PointedVoxel.z,0);
						Actor.Inventory.StoreBlocks(VoxelType,1);
						printf("Mining\n");
						*/
					}
					break;
				case MouseButton.Button3: // Mouse scroll down
										  //if( 
					Actor.Inventory.Select_PreviousItem();

					// printf("Selected : %ld : %s\n", Actor.Inventory.GetActualItemSlotNum(), GameEnv.VoxelTypeManager.VoxelTable[Actor.Inventory.GetActualItemSlot().VoxelType].VoxelTypeName.String);
					break;
				case MouseButton.Button4: // Mouse scroll up
					Actor.Inventory.Select_NextItem();

					// printf("Selected : %ld : %s\n", Actor.Inventory.GetActualItemSlotNum(), GameEnv.VoxelTypeManager.VoxelTable[Actor.Inventory.GetActualItemSlot().VoxelType].VoxelTypeName.String);
					break;


				}
			}
			return ( true );
		}

		internal override bool Tool_MouseButtonRelease( MouseButton Button )
		{
			Actor Actor;

			Actor = GameEnv.GetActiveActor();

			// printf("Release : %ld\n",Button);

			if( Actor != null )
			{
				switch( Button )
				{
				case MouseButton.Button2: // Right mouse button
					break;

				case MouseButton.Button3: // Wheel Button
					break;
				case MouseButton.Button1: // Left button
					{
						//UShort Voxel;
						if( MiningInProgress )
						{
							Mining_MaterialResistanceCounter = 0;
							MiningInProgress = false;
							GameEnv.GameProgressBar.SetCompletion( 0.0f );
							GameEnv.GameProgressBar.Hide();
#if COMPILEOPTION_FNX_SOUNDS_1
							//if( SoundHandle != 0 ) { GameEnv.Sound.Stop_PlaySound( SoundHandle ); SoundHandle = 0; }
#endif
						}
					}
					break;
				case MouseButton.Button4: // Mouse scroll down
					break;
				case MouseButton.Button5: // Mouse scroll up
					break;


				}
			}
			return ( true );
		}


		internal override bool Tool_StillEvents( double FrameTime, MouseDevice MouseButtonMatrix, KeyboardDevice KeyboardMatrix )
		{
			Actor Actor;
			Actor = GameEnv.GetActiveActor(); if( Actor == null ) return ( true );


			// Breaking material in progress

			if( MouseButtonMatrix[MouseButton.Button2] && MiningInProgress )
			{
				ushort Voxel;
				VoxelType VoxelType;

				// Get actualy pointed voxel

				if( !Actor.PointedVoxel.Collided )
				{
					Mining_MaterialResistanceCounter = 1000;
					GameEnv.GameProgressBar.SetCompletion( 0.0f );
					return ( true );
				}


				Voxel = GameEnv.World.GetVoxel( Actor.PointedVoxel.PointedVoxel.X, Actor.PointedVoxel.PointedVoxel.Y, Actor.PointedVoxel.PointedVoxel.Z );
				VoxelType = GameEnv.VoxelTypeManager.GetVoxelType( Voxel );

				// Uhhh, the player has moved is tool on another voxel, so resetting mining.

				if( !Actor.PointedVoxel.PointedVoxel.Equals( ref MinedVoxel ) )
				{
					// Does this tool can break this material ?
					if( ToolCompatibleTypes[VoxelType.properties.MiningType] )
					{
						Mining_MaterialResistanceCounter = VoxelType.properties.MiningHardness;
						MiningInProgress = true;
						MinedVoxel = Actor.PointedVoxel.PointedVoxel;

					}
				}

				// Material resistance is slowly going down

				Mining_MaterialResistanceCounter -= this.ToolForce[VoxelType.properties.MiningType] * (float)FrameTime;

				// printf("Resistance :%lf\n",Mining_MaterialResistanceCounter);
				GameEnv.GameProgressBar.SetCompletion( ( 100.0f / VoxelType.properties.MiningHardness ) * ( VoxelType.properties.MiningHardness - Mining_MaterialResistanceCounter ) );
				// Ok, it's breaking

				if( Mining_MaterialResistanceCounter < 0.0 )
				{
					Mining_MaterialResistanceCounter = 10000.0f;
					if( Actor.Inventory.StoreBlocks( Voxel, 1 ) )
					{
						VoxelRef VLoc;
						if( GameEnv.World.SetVoxel_WithCullingUpdate( Actor.PointedVoxel.PointedVoxel.X, Actor.PointedVoxel.PointedVoxel.Y, Actor.PointedVoxel.PointedVoxel.Z, 0, VoxelSector.ModifiedFieldFlags.CRITICAL, true, out VLoc ) )
							VLoc.Sector.Flag_HighPriorityRefresh = true;
					}
#if COMPILEOPTION_FNX_SOUNDS_1
					//GameEnv.Sound.PlaySound( 6 );
					//if( SoundHandle != 0 ) { GameEnv.Sound.Stop_PlaySound( SoundHandle ); SoundHandle = 0; }
#endif
					// Sector.Flag_HighPriorityRefresh
				}
			}
			return ( true );
		}



		internal override void Start_Tool()
		{
			MiningInProgress = false;
			if( !GameEnv.VoxelTypeBar.Is_Shown() ) GameEnv.VoxelTypeBar.Show();
		}

		internal override void End_Tool()
		{
			if( GameEnv.VoxelTypeBar.Is_Shown() ) GameEnv.VoxelTypeBar.Hide();
		}

		internal override void Display()
		{
			GameWindow_VoxelTypeBar VoxelTypeBar;
			Actor Actor;
			int ActualSlotNum, i;

			int Slot;



			Actor = GameEnv.GetActiveActor();
			VoxelTypeBar = GameEnv.VoxelTypeBar;
			ActualSlotNum = Actor.Inventory.GetActualItemSlotNum();

			//if (ActualSlotNum != OldToolNum)
			{


				// Descending order;

				Slot = ActualSlotNum;
				Slot -= 6; // Slot offset
				if( Slot < (int)Inventory.SlotType.Inventory_StartSlot ) { Slot -= (int)Inventory.SlotType.Inventory_StartSlot; Slot = (int)Inventory.SlotType.Inventory_EndSlot + Slot + 1; }

				for( i = 0; i < 14; i++ )
				{
					InventoryBox Box;
					Inventory.Entry InvSlot;

					Box = VoxelTypeBar.GetInventoryBox( i );
					InvSlot = Actor.Inventory.GetSlotRef( Slot );

					Box.SetEntry( InvSlot );

					Slot++; if( Slot > (int)Inventory.SlotType.Inventory_EndSlot ) Slot = (int)Inventory.SlotType.Inventory_StartSlot;

				}

				OldToolNum = ActualSlotNum;
			}

		}
	}

	internal class Tool_Constructor_P2 : Tool_Constructor
	{

		internal Tool_Constructor_P2()
		{
			ToolForce[1] = 1.6f;
			ToolForce[2] = 0.8f;
			ToolCompatibleTypes[1] = true;
			ToolCompatibleTypes[2] = true;
		}
	};

	internal class Tool_Constructor_P3 : Tool_Constructor
	{

		internal Tool_Constructor_P3()
		{
			ToolForce[1] = 3.2f;
			ToolForce[2] = 1.6f;
			ToolForce[3] = 0.8f;
			ToolCompatibleTypes[1] = true;
			ToolCompatibleTypes[2] = true;
			ToolCompatibleTypes[3] = true;
		}
	};

	internal class Tool_Constructor_P4 : Tool_Constructor
	{

		internal Tool_Constructor_P4()
		{
			ToolForce[1] = 6.4f;
			ToolForce[2] = 6.4f;
			ToolForce[3] = 3.2f;
			ToolCompatibleTypes[1] = true;
			ToolCompatibleTypes[2] = true;
			ToolCompatibleTypes[3] = true;
		}
	};

	internal class Tool_Constructor_P5 : Tool_Constructor
	{

		internal Tool_Constructor_P5()
		{
			ToolForce[1] = 12.8f;
			ToolForce[2] = 12.8f;
			ToolForce[3] = 12.8f;
			ToolForce[4] = 12.8f;
			ToolForce[5] = 12.8f;
			ToolCompatibleTypes[1] = true;
			ToolCompatibleTypes[2] = true;
			ToolCompatibleTypes[3] = true;
			ToolCompatibleTypes[4] = true;
			ToolCompatibleTypes[5] = true;
		}
	};


}
