/*
 * This file is part of Voxelarium.
 * Original code from Xperdex; and SACK (System abstraction component kit) 
 * https://github.com/d3x0r/sack
 * https://sf.net/projects/xperdex
 *
 * Copyright 2015-2016 James Buckeyne  *** Added 11/22/2015
 *
 * Voxelarium is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Voxelarium is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
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
