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
