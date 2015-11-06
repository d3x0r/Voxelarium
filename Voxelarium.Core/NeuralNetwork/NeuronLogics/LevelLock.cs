using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.NeuralNetwork.Logic
{
	public class LevelLockLogic: NeuronAlgorithm
	{
		bool prior_falacy;
		bool locked;
		int max;

		public LevelLockLogic( int max )
		{
			this.max = max;
		}
		public LevelLockLogic( )
		{
			this.max = Neuron.DefaultMax;
		}

		#region NeuronAlgorithm Members

		int NeuronAlgorithm.truth( int input, int threshold )
		{
			if( prior_falacy )
			{
				locked = !locked;
				prior_falacy = false;
			}
			if( locked )
				return max;
			else
				return 0;
		}

		int NeuronAlgorithm.falacy( int input, int threshold )
		{
			prior_falacy = true;
			if( locked )
				return max;
			else
				return 0;			
		}

		void NeuronAlgorithm.Properties()
		{
		}

		#endregion

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
