/*
 * This file is part of Voxelarium.
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
using OpenTK.Input;
using System;
using System.Collections.Generic;
using System.Data;
using System.Text;

namespace Voxelarium.Common
{
	public class Settings
	{
		static DataSet settings;
		static DataTable setting_table;
		const string SettingFilename = "settings.xml";
		static Settings()
		{
			settings = new DataSet( "settings" );
			settings.Tables.Add( setting_table = new DataTable( "settings" ) );
			setting_table.Columns.Add( "name", typeof( string ) );
			setting_table.Columns.Add( "value", typeof( string ) );
			try
			{
				settings.ReadXml( SettingFilename );
			}
			catch( Exception e )
			{
				Log.log( "Error loading setitngs: " + e.Message );
			}
		}

		public static bool Read( string name, bool default_val = false )
		{
			DataRow[] setting = settings.Tables["settings"].Select( "name='" + name + "'" );
			if( setting.Length > 0 )
				return Convert.ToBoolean(setting[0]["value"]);
			else
				Write( name, default_val );
			return default_val;
		}

		public static float Read( string name, float default_val )
		{
			DataRow[] setting = settings.Tables["settings"].Select( "name='" + name + "'" );
			if( setting.Length > 0 )
				return (float)Convert.ToDouble(setting[0]["value"]);
			else
				Write( name, default_val );
			return default_val;
		}

		public static void Write( string name, bool val )
		{
			DataRow[] setting = settings.Tables["settings"].Select( "name='" + name + "'" );
			if( setting.Length > 0 )
			{
				setting[0]["value"] = val;
			}
			else
			{
				DataRow row = setting_table.NewRow();
				row["name"] = name;
				row["value"] = val;
				setting_table.Rows.Add( row );
				settings.WriteXml( SettingFilename );
			}
		}
		public static int Read( string name, int default_val = 0 )
		{
			DataRow[] setting = settings.Tables["settings"].Select( "name='" + name + "'" );
			if( setting.Length > 0 )
				return (Convert.ToInt32(setting[0]["value"]));
			else
				Write( name, default_val );
			return default_val;
		}
		public static Key Read( string name, Key default_val = 0 )
		{
			DataRow[] setting = settings.Tables["settings"].Select( "name='" + name + "'" );
			if( setting.Length > 0 )
			{
				return (Key)Enum.Parse( typeof( Key ), (string)setting[0]["value"] );
			}
			else
				Write( name, default_val.ToString() );
			return default_val;
		}

		public static void Write( string name, int val )
		{
			DataRow[] setting = settings.Tables["settings"].Select( "name='" + name + "'" );
			if( setting.Length > 0 )
			{
				setting[0]["value"] = val;
			}
			else
			{
				DataRow row = setting_table.NewRow();
				row["name"] = name;
				row["value"] = val;
				setting_table.Rows.Add( row );
				settings.WriteXml( SettingFilename );
			}
		}
		public static void Write( string name, Key val )
		{
			DataRow[] setting = settings.Tables["settings"].Select( "name='" + name + "'" );
			if( setting.Length > 0 )
			{
				setting[0]["value"] = val.ToString();
			}
			else
			{
				DataRow row = setting_table.NewRow();
				row["name"] = name;
				row["value"] = val.ToString();
				setting_table.Rows.Add( row );
				settings.WriteXml( SettingFilename );
			}
		}

		public static void Write( string name, float val )
		{
			DataRow[] setting = settings.Tables["settings"].Select( "name='" + name + "'" );
			if( setting.Length > 0 )
			{
				setting[0]["value"] = val;
			}
			else
			{
				DataRow row = setting_table.NewRow();
				row["name"] = name;
				row["value"] = val;
				setting_table.Rows.Add( row );
				settings.WriteXml( SettingFilename );
			}
		}
		public static string Read( string name, string default_val = "" )
		{
			DataRow[] setting = settings.Tables["settings"].Select( "name='" + name + "'" );
			if( setting.Length > 0 )
				return (string)setting[0]["value"];
			else
				Write( name, default_val );
			return default_val;
		}

		public static void Write( string name, string val )
		{
			DataRow[] setting = settings.Tables["settings"].Select( "name='" + name + "'" );
			if( setting.Length > 0 )
			{
				setting[0]["value"] = val;
			}
			else
			{
				DataRow row = setting_table.NewRow();
				row["name"] = name;
				row["value"] = val;
				setting_table.Rows.Add( row );
				settings.WriteXml( SettingFilename );
			}
		}

	}
}
