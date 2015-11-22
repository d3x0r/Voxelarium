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
using Voxelarium.NeuralNetwork.Types;

namespace Voxelarium.NeuralNetwork
{
	public class Synapse: IBrainMatter
	{
		public string Name;
		// Neuron methods operate directly on these.
		internal Neuron from;
		internal Neuron to;
		Brain brain;
		int _cycle;
		int result;
		int _gain;  // scaled int 1024 ( >> 10 )

		public float Gain
		{
			set { _gain = (int)(( 1 << 10 ) * value); }
			get { return ((float)_gain) / 1024.0F; }
		}

		public int Cycle
		{
			set
			{
				if( _cycle != value )
				{
					_cycle = value;

					from.Cycle = value;
					//Log.log( "Synapse(" + Name + ") to update" );
					result = ( ( from.Output * _gain ) >> 10 );
				}			 
			}
			get
			{
				return _cycle;
			}
		}

		public Synapse()
		{
			_gain = 1024;
		}


		public int this[Neuron n]
		{
			get
			{
				if( n.Equals( to ) )
				{
					_cycle = n.Cycle;
					return result;
				}
				return 0;
			}
		}


		public static Synapse operator +( Synapse s, Neuron n )
		{
			if( ( s.from == null ) && ( n.IndexOf( s ) < 0 ) )
			{
				s.from = n;
				n.Add( s );
				return s;
			}
			return null;
		}


		public static implicit operator int( Synapse n )
		{
			return n.result;
		}

		public Neuron From
		{
			set {
				if( from != null )
					from.Remove( this );
				if( value != null )
					value.Add( this );

				from = value;
			}

			get { return from; }
		}

		public Neuron To
		{
			set
			{
				if( to != null )
				{
					to.Remove( this );
				}
				if( value != null )
					value.Add( this );
				to = value;
			}

			get { return to; }
		}


		void Init( Brain b, Synapse _default )
		{
			brain = b;
			if( _default == null )
			{
				_gain = 1024;
			}
			else
			{
				Synapse s = _default;
				_gain = s._gain;
			}
			this._cycle = 0;
		}

		#region IBrainMatter Members

		void IBrainMatter.Clear()
		{
			_gain = 1 << 10;
		}

		void IBrainMatter.Init( Brain b )
		{
			Init( b, b.DefaultSynapse );
		}

		#endregion
	}
}
