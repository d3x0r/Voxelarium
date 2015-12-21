/*
 * Before porting, this header appeared inmost sources.  Of course
 * the change from C++ to C# required significant changes an no part
 * is entirely original.
 * 
 * This file is part of Blackvoxel (Now Voxelarium)
 *
 * Copyright 2010-2014 Laurent Thiebaut & Olivia Merle
 * Copyright 2015-2016 James Buckeyne
 *
 * Blackvoxel is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Blackvoxel is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
 * ZTools.cpp
 *
 *  Created on: 12 juil. 2011
 *      Author: laurent
 */

using OpenTK.Input;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Voxelarium.Core.Game
{
	internal class Tool_Manager
	{
		internal const int TOOL_TOOLTYPESNUMBER = 256;
		internal class Tool
		{
			protected VoxelGameEnvironment GameEnv;

			internal void SetGameEnv( VoxelGameEnvironment GameEnv ) { this.GameEnv = GameEnv; }

			internal Tool()
			{
			}

			~Tool()
			{
				GameEnv = null;
			}

			internal virtual void Start_Tool()
			{

			}

			internal virtual void End_Tool()
			{

			}

			internal virtual void Display()
			{

			}

			internal virtual bool Tool_MouseButtonClick( MouseButton Button ) { return ( false ); }
			internal virtual bool Tool_MouseButtonRelease( MouseButton Button ) { return ( false ); }
			internal virtual bool Tool_StillEvents( double FrameTime, MouseDevice MouseButtonMatrix, KeyboardDevice KeyboardMatrix ) { return ( true ); }

		};

		internal class ToolManager
		{
			// Allow Direct access to ToolList
			internal List<Tool> ToolList = new List<Tool>();

			VoxelGameEnvironment GameEnv;

			int ActualTool_VoxelType;


			internal void SetGameEnv( VoxelGameEnvironment GameEnv ) { this.GameEnv = GameEnv; }

			internal ToolManager()
			{
				ActualTool_VoxelType = -1;
			}

			~ToolManager()
			{
				ToolList = null;
			}

			void AddTool( int ToolNum, Tool Tool ) { ToolList[ToolNum] = Tool; Tool.SetGameEnv( GameEnv ); }
			void RemoveTool( int ToolNum ) { ToolList[ToolNum] = null; }
			Tool GetTool( int ToolNum ) { return ( ToolList[ToolNum] ); }

			internal void ProcessAndDisplay()
			{
				Actor Actor;

				Inventory.Entry ToolSlot;

				Tool Tool;
				ushort NewToolType;

				Actor = GameEnv.GetActiveActor();

				ToolSlot = Actor.Inventory.GetActualToolSlot();
				NewToolType = ToolSlot.VoxelType;

				if( NewToolType != this.ActualTool_VoxelType )
				{
					// Stop the actual tool service.
					if( ActualTool_VoxelType != -1 )
					{
						Tool = GetTool( ActualTool_VoxelType );
						if( Tool != null ) Tool.End_Tool();
					}

					// Start the new tool service.

					Tool = GetTool( NewToolType );
					if( Tool != null ) Tool.Start_Tool();

					// Make the tool the actual tool

					ActualTool_VoxelType = NewToolType;
				}

				Tool = GetTool( NewToolType );
				if( Tool != null ) Tool.Display();

			}
		};

	}
}
