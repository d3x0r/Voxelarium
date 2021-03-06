﻿/*
 * Before porting, this header appeared inmost sources.  Of course
 * the change from C++ to C# required significant changes an no part
 * is entirely original.
 * 
 * This file is part of Blackvoxel. (Now Voxelarium)
 *
 * Copyright 2010-2014 Laurent Thiebaut & Olivia Merle
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
using OpenTK.Audio;
using OpenTK.Audio.OpenAL;

namespace Voxelarium.Core.UI
{
	internal class Sound
	{
		const int ZSOUND_MAX_SOUNDFILES = 512;

		internal struct SoundFile
		{
			internal bool Used;
			internal short[] SoundData;
			internal uint SoundLen;
		};

		int[] ALSources;
		SoundFile[] SoundBank = new SoundFile[ZSOUND_MAX_SOUNDFILES];
		uint SampleCount;
		internal delegate void EndSignal();

		internal class SoundData
		{
			internal SoundData NextSound;
			internal SoundData PrevSound;

			internal uint SoundNum;
			internal ushort[] Data;
			internal uint Pos;
			internal uint Len;
			internal uint RLen;
			internal uint Volume;
			internal uint Calls;
			// New
			internal bool DeletePending;
			internal bool Repeat;
			internal uint RepeatPos;
			internal double DRepeatPos;
			//internal bool[] FlagOnEnd;
			internal EndSignal FlagOnEnd;
			internal bool UseIntCompute;
			internal double FrequencyVar;
			internal double DPos;
		};
		static SoundData PlayingSoundList;

		bool SoundActivated;
		static uint[] SoundBuffer = new uint[65536];

		internal void PlaySound( uint sound_id )
		{
		}

		static void mixaudio( object unused, short[] stream, uint len )
		{
			SoundData Snd, OldSound;
			uint PlayLen, i, Buffer_Len, Buffer_RemainLen;

			uint PlayPos;
			uint nSounds;

			Buffer_Len = len;

			for( i = 0; i < Buffer_Len; i++ )
				SoundBuffer[i] = 0;
			nSounds = 0;

			Snd = PlayingSoundList;
			while( Snd != null )
			{
				if( Snd.UseIntCompute )
				{
					Buffer_RemainLen = Buffer_Len;

					while( Buffer_RemainLen != 0 )
					{
						PlayLen = ( Snd.RLen > Buffer_RemainLen ) ? Buffer_RemainLen : Snd.RLen;
						if( PlayLen == 0 )
						{

							if( Snd.Repeat )
							{
								Snd.Pos = Snd.RepeatPos;
								Snd.RLen = Snd.Len - Snd.RepeatPos;
							}
							else
							{
								if( Snd.FlagOnEnd != null ) Snd.FlagOnEnd();
								Snd.DeletePending = true;
								break;
							}
						}
						else
						{
							for( i = 0; i < PlayLen / 2; i++ ) SoundBuffer[i] += Snd.Data[i+Snd.Pos];
							Buffer_RemainLen -= PlayLen;
							Snd.RLen -= PlayLen;
							Snd.Pos += PlayLen;
						}
					}
					//SDL_MixAudio(stream, Snd.Data + Snd.Pos, PlayLen, SDL_MIX_MAXVOLUME);
					//for (i=0;i<PlayLen-2;i+=2) {((UShort *)stream)[i]=0x0;((UShort *)stream)[i+1]=0x8000;}
				}
				else
				{
					double Coef, Temp;
					uint BufferPos;
					uint MaxOffset;
					Buffer_RemainLen = Buffer_Len >> 1;
					BufferPos = 0;
					MaxOffset = ( Snd.Len >> 1 ) - 1;

					double S1, S2, Ir;
					uint Offset;



					while( Buffer_RemainLen > 0 && ( !Snd.DeletePending ) )
					{
						Coef = Snd.DPos - ( Temp = Math.Floor( Snd.DPos ) );
						Offset = (uint)Temp;
						if( Offset < MaxOffset )
						{
							S1 = Snd.Data[Offset];
							S2 = Snd.Data[Offset + 1];
							Ir = S1 * ( 1.0 - Coef ) + ( S2 * Coef );
							SoundBuffer[BufferPos++] += (uint)Ir;
							Buffer_RemainLen--;
							Snd.DPos += Snd.FrequencyVar;
						}
						else
						{
							if( Snd.Repeat )
							{
								Snd.DPos = Snd.DRepeatPos;
							}
							else
							{
								if( Snd.FlagOnEnd != null ) Snd.FlagOnEnd();// = true;
								Snd.DeletePending = true;
								break;
							}
						}
					}
				}

#if false
				OldSound = Snd;
				Snd = Snd.NextSound;

				// If sound must be deleted

				if( OldSound.DeletePending )
				{
					if( OldSound.PrevSound == null ) PlayingSoundList = OldSound.NextSound;
					else OldSound.PrevSound.NextSound = OldSound.NextSound;
					if( OldSound.NextSound != null ) OldSound.NextSound.PrevSound = OldSound.PrevSound;
					OldSound.NextSound = null;
					OldSound.PrevSound = null;
					delete OldSound;
				}
#endif

				nSounds++;
			}

			// if (nSounds) for (i=0;i<Buffer_Len/2;i++) ((Short *)stream) [i] = (Short)(SoundBuffer[i]/nSounds );
			if( nSounds != 0 ) for( i = 0; i < Buffer_Len / 2; i++ ) ( stream )[i] = (short)( SoundBuffer[i] );
			//else
				//memset( stream, 0, len );

		}



		SoundData FindSound( SoundData SoundHandle )
		{
			SoundData Snd;
			Snd = PlayingSoundList;
			while( Snd != null )
			{
				if( Snd == SoundHandle )
				{
					return ( Snd );
				}

				Snd = Snd.NextSound;
			}

			return ( null );
		}


		public Sound()
		{
			int i;
			SampleCount = 0;
			SoundActivated = false; PlayingSoundList = null;
			for( i = 0; i < ZSOUND_MAX_SOUNDFILES; i++ ) { SoundBank[i].Used = false;
				SoundBank[i].SoundData = null;
				SoundBank[i].SoundLen = 0; }

		}
		~Sound()
		{
			int i;

			for( i = 0; i < ZSOUND_MAX_SOUNDFILES; i++ )
			{
				if( SoundBank[i].Used )
				{
					SoundBank[i].Used = false;
					SoundBank[i].SoundData = null;
					SoundBank[i].SoundLen = 0;
				}
			}
			SampleCount = 0;

		}

		uint GetSampleCount() { return SampleCount; }

		void SampleModify_Volume( uint SoundNum, double NewVolume )
		{
			uint i, Len;
			short[] Samples;
			double Dat;

			if( !SoundBank[SoundNum].Used ) return;
			Len = SoundBank[SoundNum].SoundLen >> 1;
			Samples = SoundBank[SoundNum].SoundData;

			for( i = 0; i < Len; i++ )
			{
				Dat = (double)Samples[i];
				Dat *= NewVolume;
				Samples[i] = (short)Dat;
			}
		}

		internal bool LoadSoundFiles()
		{
/*
			for( i = 0; i < ZSOUND_MAX_SOUNDFILES; i++ )
			{

				FileName.Clear();
				FileName << i << ".wav";
				FileSpec = COMPILEOPTION_DATAFILESPATH;
				FileSpec.AddToPath( "Sound" ).AddToPath( FileName );
			}
			*/
			return ( true );
		}


		SoundData Start_PlaySound( uint SoundNum, bool Repeat = false, bool UseIntFastMath = true, double Frequency = 1.0, EndSignal FlagToSetAtSoundEnd = null )
		{
			return null;
		}

		bool IsPlayed( SoundData SoundHandle )
		{
			return true;
		}

		void Stop_PlaySound( SoundData SoundHandle )
		{
		}

		void Stop_AllSounds()
		{
		}

		void ModifyFrequency( SoundData SoundHandle, double NewFrequency )
		{
		}
		void EndAudio()
		{
		}

	}
}
