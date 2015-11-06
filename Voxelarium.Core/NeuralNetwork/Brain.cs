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
