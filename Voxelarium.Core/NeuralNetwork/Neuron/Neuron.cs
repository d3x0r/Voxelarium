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
using Voxelarium.NeuralNetwork.Types;

namespace Voxelarium.NeuralNetwork
{

	public interface NeuronAlgorithm
	{
		int truth( int input, int threshold );
		int falacy( int input, int threshold );
		void Properties();
		int min { get; set; }
		int max { get; set; }
	}

	public class NeuronLogic : NeuronAlgorithm
	{
		int _max, _min;
		public int max { get { return _max; } set { _max = value; } }
		public int min { get { return _min; } set { _min = value; } }

		public enum Algorithm
		{
			digital // 0 is false 1 is true
			, inverted_digital // 1 is false 0 is true
			, low_true  // 0 is false, -1 is true;
			, low_true_inverted // -1 is false, 0 is true
			, analog         // 
			, invted_analog
			, power
			, power_inverted
			, ground
		}

		public NeuronLogic( Algorithm a )
		{
			_algorithm = a;
		}

		public NeuronLogic()
		{
			_algorithm = Algorithm.analog;
		}

		Algorithm _algorithm;

		public Algorithm algorithm
		{
			get { return this._algorithm; }
			set { this._algorithm = value; }
		}

		public int truth( int input, int threshold )
		{
			switch( algorithm )
			{
				case Algorithm.digital:
					return max;
				case Algorithm.inverted_digital:
					return 0;
				case Algorithm.low_true:
					return min;
				case Algorithm.low_true_inverted:
					return 0;
				case Algorithm.analog:
					return input - threshold;
				case Algorithm.invted_analog:
					return threshold - input; // -(input-threshold) == (-input +threshold)
				case Algorithm.power:
					return max;
				case Algorithm.power_inverted:
					return min;
				case Algorithm.ground:
					return 0;
			}
			return 0;
		}

		public int falacy( int input, int threshold )
		{
			switch( algorithm )
			{
				case Algorithm.digital:
					return 0;
				case Algorithm.inverted_digital:
					return max;
				case Algorithm.low_true:
					return 0;
				case Algorithm.low_true_inverted:
					return min;
				case Algorithm.analog:
					return 0;
				case Algorithm.invted_analog:
					return 0; // -(input-threshold) == (-input +threshold)
				case Algorithm.power:
					return max;
				case Algorithm.power_inverted:
					return min;
				case Algorithm.ground:
					return 0;
			}
			return 0;
		}

		public override string ToString()
		{
			return "Perceptron";
			//return base.ToString();
		}
		public static string NiceName()
		{
			return "Perceptron";
		}

		void NeuronAlgorithm.Properties()
		{
		}

	}

	public class Neuron : List<Synapse>, IBrainMatter
	{
		public static int DefaultMax = 1024;
		//List<Synapse> connections;
		public Brain brain;
		public string Name;
		NeuronAlgorithm logic;

		delegate int something();

		int _cycle; // for use by the brain to track who's caught up to the current tick.
		internal int Cycle
		{
			get { return _cycle; }
			set
			{
				if( _cycle != value )
				{
					int total = 0;
					//Log.log( "Neuron(" + Name + ") to update" );
					_cycle = value;
					foreach( Synapse synapse in this )
					{
						synapse.Cycle = value;
						total += synapse;
					}
					input = total;

					if( input >= _threshold ) // 0 threshold and 0 input is 0...
						_output = logic.truth( input, _threshold );
					else
						_output = logic.falacy( input, _threshold );
				}
			}
		}
		int _min;
		int _max;
		int _threshold;

		public int Min
		{
			get { return _min; }
			set { _min = value; }

		}
		public int Max
		{
			get { return _max; }
			set { _max = value; }
		}
		public int Threshold
		{
			get { return _threshold; }
			set { _threshold = value; }
		}

		int input;
		int _output;

		public int Output
		{
			get
			{
				return _output;
			}
		}
		int bias; // offset of 0 ... min and max still relative to 0

		void Init( Brain brain, Neuron _default )
		{
			//connections = new List<Synapse>();
			this.brain = brain;
			if( _default == null )
			{
				_min = -1024;
				_max = 1024;
				logic = new NeuronLogic( NeuronLogic.Algorithm.analog );
				//_algorithm = Algorithm.analog;
				_threshold = 0;
			}
			else
			{
				_min = _default._min;
				_max = _default._max;
				logic = new NeuronLogic( NeuronLogic.Algorithm.analog );
				_threshold = _default._threshold;
			}
		}
		public Neuron()
		{
			Init( null, null );
		}
#if asdfasdf
		Neuron( Brain brain, Neuron _default )
		{
			
			connections = new List<ISynapse>();
			this.brain = brain;
			_min = _default.min;
			_max = _default.max;
			_algorithm = _default.algorithm;
			_threshold = _default.threshold;

			input = 0;
			output = 0;
			cycle = 0;
			bias = 0;
			//Init( brain, _default );
		}
#endif
		public NeuronAlgorithm Logic
		{
			set
			{
				this.logic = value;
			}
			get
			{
				return this.logic;
			}
		}

		public static implicit operator int ( Neuron n )
		{
			if( n.Cycle != n.brain.Cycle )
				n.Cycle = n.brain.Cycle;
			return n._output;
		}

		int Value
		{
			get
			{
				return this;
			}
		}

		#region IBrainMatter Members

		void IBrainMatter.Init( Brain brain )
		{
			Init( brain, brain.DefaultNeuron );
		}

		void IBrainMatter.Clear()
		{
			this._cycle = 0;
			this.input = 0;
			this._min = 0;
			this._max = 0;
			this._threshold = 0;
			this._output = 0;
			//throw new Exception( "The method or operation is not implemented." );
		}

		#endregion


		public static Neuron operator +( Neuron n, Synapse s )
		{
			if( ( s.to == null ) && ( n.IndexOf( s ) < 0 ) )
			{
				n.Add( s );
				s.to = n;
				return n;
			}
			return null;
		}

		public static Neuron operator +( Neuron n1, Neuron n2 )
		{
			Synapse syn = n1.brain.GetSynapse();
			n1 += syn += n2;
			return n1;
		}

		public static Synapse operator *( float gain, Neuron n )
		{
			Synapse syn = n.brain.GetSynapse();
			syn.Gain = gain;
			syn += n;
			return syn;
		}

		public static Neuron operator &( Synapse s1, Neuron n2 )
		{
			//Synapse s1 = n1.brain.GetSynapse();
			//s1.gain = 0.25F;
			s1.Gain = s1.Gain * 0.25F;
			Synapse s2 = n2.brain.GetSynapse();
			s2.Gain = 0.25F;
			Neuron output = n2.brain.GetNeuron( "("
				+ ( s1.from == null ? "?" : s1.from.Name ) + "&"
				+ n2.Name + ")"
				);
			//s1 += n1;
			s2 += n2;
			output += s1;
			output += s2;
			output._threshold = ( s1.from._max + n2._max - 1 ) / 4;
			output.Logic = new NeuronLogic( NeuronLogic.Algorithm.digital );
			return output;
		}

		public static Neuron operator &( Neuron n1, Neuron n2 )
		{
			Synapse s1 = n1.brain.GetSynapse();
			s1.Gain = 0.25F;
			Synapse s2 = n1.brain.GetSynapse();
			s2.Gain = 0.25F;
			Neuron output = n1.brain.GetNeuron( "(" + n1.Name + "&" + n2.Name + ")" );
			s1 += n1;
			s2 += n2;
			output += s1;
			output += s2;
			output._threshold = ( n1._max + n2._max - 1 ) / 4;
			output.Logic = new NeuronLogic( NeuronLogic.Algorithm.digital );
			return output;
		}

		public static Neuron operator |( Neuron n1, Neuron n2 )
		{
			Synapse s1 = n1.brain.GetSynapse();
			s1.Gain = 0.25F;
			Synapse s2 = n1.brain.GetSynapse();
			s2.Gain = 0.25F;
			Neuron output = n1.brain.GetNeuron( "(" + n1.Name + "|" + n2.Name + ")" );
			s1 += n1;
			s2 += n2;
			output += s1;
			output += s2;
			output._threshold = ( Math.Min( n1._max, n2._max ) - 1 ) / 4;
			output.Logic = new NeuronLogic( NeuronLogic.Algorithm.digital );
			return output;
		}

		public static Neuron operator ^( Neuron n1, Neuron n2 )
		{
			Synapse s1 = n1.brain.GetSynapse( "^S1" );
			s1.Gain = 0.25F;
			Synapse s2 = n1.brain.GetSynapse( "^S2" );
			s2.Gain = 0.25F;
			Synapse s3 = n1.brain.GetSynapse( "^S3" );
			s3.Gain = -1.0F;
			Neuron output = n1.brain.GetNeuron( "^Output" );
			output.Logic = new NeuronLogic( NeuronLogic.Algorithm.digital );
			Neuron extra = n1.brain.GetNeuron( "^Extra" );
			s1 += n1;
			s2 += n2;
			output += s1;
			output += s2;
			output += s3;

			s3 += extra;
			extra._threshold = ( n1._max + n2._max - 1 ) / 4;
			extra.Logic = new NeuronLogic( NeuronLogic.Algorithm.digital );
			output._threshold = ( Math.Min( n1._max, n2._max ) - 1 ) / 4;
			output.Logic = new NeuronLogic( NeuronLogic.Algorithm.digital );
			return output;
		}

		/// <summary>
		/// Connect this with 'to' and return the synapse used.
		/// </summary>
		/// <param name="to"></param>
		/// <returns></returns>
		public Synapse ConnectTo( Neuron to )
		{
			Synapse s = brain.synapses;
			s.From = this;
			s.To = to;
			return s;
			//throw new Exception( "The method or operation is not implemented." );
		}

#if ALLOW_LOGGING
		public void Log( string prefix )
		{
			Log.log( "Neuron (" + ( prefix == null ? "" : prefix ) + ( Name == null ? "?" : Name )
				+ " = " + (int)this
				+ " T=" + _threshold
				+ " I=" + input
				+ " O=" + _output
				);
		}
		public void Log()
		{
			Log.log( "Neuron (" + ( Name == null ? "?" : Name )
				+ " = " + (int)this
				+ " T=" + _threshold
				+ " I=" + input
				+ " O=" + _output
				);
		}
#endif
	}
}
