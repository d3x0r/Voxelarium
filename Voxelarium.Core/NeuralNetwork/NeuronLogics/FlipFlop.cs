using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.NeuralNetwork.Logic
{
	public class FlipFlop : NeuronAlgorithm
	{
		int min = 0;
		int max = 1024;
		bool prior;
		bool locked;

		void Update( ref int input, ref int threshold )
		{
			if( input > threshold )
			{
				if( prior )
				{
					locked = !locked;
				}
				prior = true;
			}
			else
				prior = false;
		}

		int NeuronAlgorithm.truth( int input, int threshold )
		{
			Update( ref input, ref threshold );
			return locked ? max : min;
		}

		int NeuronAlgorithm.falacy( int input, int threshold )
		{
			Update( ref input, ref threshold );
			return !locked ? max : min;
		}

		void NeuronAlgorithm.Properties()
		{
		}

		#region NeuronAlgorithm Members


		int NeuronAlgorithm.min
		{
			get
			{
				throw new NotImplementedException();
			}
			set
			{
				throw new NotImplementedException();
			}
		}

		int NeuronAlgorithm.max
		{
			get
			{
				throw new NotImplementedException();
			}
			set
			{
				throw new NotImplementedException();
			}
		}

		#endregion
	}
}
