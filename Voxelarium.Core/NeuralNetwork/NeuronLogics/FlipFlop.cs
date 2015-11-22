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
