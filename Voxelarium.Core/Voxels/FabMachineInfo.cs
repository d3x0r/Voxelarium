using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.Voxels
{
	public class FabMachineInfo
	{
		public class FabMaterial
		{
			public ushort VoxelType;
			public ushort SlotNum;
		};


		public class FabEntry
		{
			public ushort Index;
			public ushort Quantity;
		};
		class ZTransformation
		{
			public const int Max_ResultVoxels = 16;
			public FabEntry[] FabList;
			public uint EntryCount;
			public uint MaxEntryCount;
			public ushort[] Result_VoxelType = new ushort[Max_ResultVoxels];
			public uint[] Result_Quantity = new uint[Max_ResultVoxels];


			public ZTransformation()
			{
				uint i;

				FabList = null;
				EntryCount = 0;
				MaxEntryCount = 0;

				for( i = 0; i < Max_ResultVoxels; i++ ) { Result_VoxelType[i] = 0; Result_Quantity[i] = 0; }
			}
			~ZTransformation()
			{
				FabList = null;
				EntryCount = 0;
				MaxEntryCount = 0;
			}
		};

		FabMaterial[] MaterialTable;
		uint MaterialCount;
		uint MaxMaterialCount;
		ZTransformation[] TransformationTable;
		uint TransformationCount;
		uint MaxTransformationCount;
		ushort PurgeSlot;
		uint PurgeCount;
		ushort ValidationSlot;
		uint ValidationCount;

		public FabMachineInfo()
		{
			MaterialTable = null;
			MaterialCount = 0;
			MaxMaterialCount = 0;

			TransformationTable = null;
			TransformationCount = 0;
			MaxTransformationCount = 0;
			PurgeSlot = 0;
			PurgeCount = 1000000;
			ValidationSlot = 0;
			ValidationCount = 1;
		}

		~FabMachineInfo()
		{
			MaterialTable = null;
			TransformationTable = null;

			MaterialCount = 0;
			MaxMaterialCount = 0;
			TransformationCount = 0;
			MaxTransformationCount = 0;
		}

		void AddMaterial( ushort VoxelType )
		{
			uint i;
			if( MaterialTable ==null)
			{
				MaxMaterialCount = 128; MaterialTable = new FabMaterial[MaxMaterialCount];
				for( i = 0; i < MaxMaterialCount; i++ ) { MaterialTable[i].SlotNum = 0; MaterialTable[i].VoxelType = 0; }
			}
			if( MaterialCount < MaxMaterialCount )
			{
				MaterialTable[MaterialCount].SlotNum = (ushort)MaterialCount;
				MaterialTable[MaterialCount].VoxelType = VoxelType;
				MaterialCount++;
			}
		}

		uint AddTransformation()
		{
			uint i;
			if( TransformationTable == null )
			{
				TransformationCount = 0;
				MaxTransformationCount = 64;
				TransformationTable = new ZTransformation[MaxTransformationCount];
			}
			if( TransformationCount < MaxTransformationCount )
			{
				for( i = 0; i < ZTransformation.Max_ResultVoxels; i++ ) { TransformationTable[TransformationCount].Result_VoxelType[i] = 0; TransformationTable[TransformationCount].Result_Quantity[i] = 0; }
				return ( TransformationCount++ );
			}
			return ( 0xFFFFFFFFU );
		}

		void AddCondition( uint TransformationNum, ushort Index, ushort Quantity )
		{
			if( TransformationTable == null ) return;
			if( TransformationTable[TransformationNum].FabList == null )
			{

				TransformationTable[TransformationNum].MaxEntryCount = 16;
				TransformationTable[TransformationNum].EntryCount = 0;
				TransformationTable[TransformationNum].FabList = new FabEntry[TransformationTable[TransformationNum].MaxEntryCount];
			}

			if( TransformationTable[TransformationNum].EntryCount < TransformationTable[TransformationNum].MaxEntryCount )
			{
				TransformationTable[TransformationNum].FabList[TransformationTable[TransformationNum].EntryCount].Index = Index;
				TransformationTable[TransformationNum].FabList[TransformationTable[TransformationNum].EntryCount].Quantity = Quantity;
				TransformationTable[TransformationNum].EntryCount++;
			}
		}

		void SetResult( uint TransformationNum, ushort ResultNum, ushort VoxelType, uint Quantity )
		{
			if( ResultNum < ZTransformation.Max_ResultVoxels )
			{
				TransformationTable[TransformationNum].Result_VoxelType[ResultNum] = VoxelType;
				TransformationTable[TransformationNum].Result_Quantity[ResultNum] = Quantity;
			}
		}

		void SetPurgeCondition( ushort Index, uint Quantity )
		{
			PurgeSlot = Index;
			PurgeCount = Quantity;
		}
		void SetValidationCondition( ushort Index, uint Quantity )
		{
			ValidationSlot = Index;
			ValidationCount = Quantity;
		}

	}
}
