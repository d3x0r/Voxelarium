using System;
using System.Collections.Generic;
using System.Text;
using Voxelarium.Core.Voxels;

namespace Voxelarium.Core.UI.Controls
{
	public class FrameListbox : Frame
	{
		public class ListboxItem {

		}
		List<ListboxItem> Items;
		int first_item;

		public FrameListbox()
		{
			FrameType = VoxelUtils.MulticharConst( 'L', 'S', 'B', 'X' ); ;
		}
	}
}
