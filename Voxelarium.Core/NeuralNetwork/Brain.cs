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

namespace Voxelarium.NeuralNetwork
{
	public partial class Brain
	{
		internal NeuronPool neurons;
		internal SynapsePool synapses;

		List<Neuron> anchor_neurons; // these are things that are known to cause update... (they are final targets)

		int _cycle;
		public int Cycle { 
			get { return _cycle; } 
			set {
				if( _cycle != value )
				{
					List<Neuron> bad_anchors = new List<Neuron>();
					_cycle = value;

					foreach( Neuron n in anchor_neurons )
					{
						if( n.Cycle != value )
						{
							n.Cycle = value;
						}
						else
						{
							// already updated, is not an anchor...
							bad_anchors.Add( n );
							// have to restart the list search...
						}
					}

					foreach( Neuron n in bad_anchors )
					{
						anchor_neurons.Remove( n );
					}

					foreach( Neuron n in neurons )
					{
						// build new anchors....
						if( n.Cycle != value )
						{
							n.Cycle = value;
							anchor_neurons.Add( n );
						}
					}
				}
			} 
		}

		public Brain()
		{
			anchor_neurons = new List<Neuron>();
			neurons = new NeuronPool( this );
			synapses = new SynapsePool( this );
			_default_neuron = new Neuron();
			_default_synapse = new Synapse();
		}
		public Neuron GetNeuron()
		{
			return neurons;
		}
		public Synapse GetSynapse()
		{
			return synapses;
		}

		Neuron _default_neuron;
		public Neuron DefaultNeuron { get { return _default_neuron; } }

		Synapse _default_synapse;
		public Synapse DefaultSynapse { get { return _default_synapse; } }


		public Neuron GetNeuron( String n1 )
		{
			Neuron n = neurons;
			n.Name = n1;
			return n;
		}

		public Synapse GetSynapse( String s1 )
		{
			Synapse s = synapses;
			s.Name = s1;
			return s;
		}


	}
}
