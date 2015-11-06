using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.NeuralNetwork.Logic
{
	public class Oscillator: NeuronAlgorithm
	{
		int level;

		int maximum_output;
		int frequency; // time between 0 and max;

		// should pipe this through a sin wave function.

		//FlipFlop flop;
		//LevelLock lock1;
		//Synapse s1;
		//Neuron feed;
		//public Oscillator(Brain b)
		//{
			//feed = b.GetNeuron( "Feed" );
			//feed.threshold = -1;
			//s1 += feed;
			//lock1 += s1;
		//}

		public Oscillator( int frequency, int max_output )
		{
			maximum_output = max_output;
			this.frequency = frequency;
		}
		public Oscillator( int frequency )
		{
			maximum_output = 1024;
			this.frequency = frequency;
		}
		public Oscillator()
		{
			maximum_output = 1024;
			this.frequency = 1000;
		}
		void SetLevel()
		{
			//throw new Exception( "The method or operation is not implemented." );
			long now = DateTime.Now.Ticks/ ( 10 * 1000 ); // convert to milliseconds.
			long steps = now / frequency;
			long step = now % frequency;
			if( ( steps & 1 ) != 0  )
				level = (int)(maximum_output - ( ( maximum_output * step ) / frequency ) );
			else
				level = (int)( ( maximum_output * step ) / frequency );
		}

		#region NeuronAlgorithm Members


		int NeuronAlgorithm.truth( int input, int threshold )
		{
			SetLevel();
			return level;
		}

		int NeuronAlgorithm.falacy( int input, int threshold )
		{
			SetLevel();
			return level;
		}

		void NeuronAlgorithm.Properties()
		{
		}

		#endregion
		public override string ToString()
		{
			return "Oscillator";
			//return base.ToString();
		}

        #region NeuronAlgorithm Members


        public int min
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

        public int max
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
