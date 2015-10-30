using System;
using System.Collections.Generic;
using System.Data;
using System.Text;

namespace Voxelarium.Core
{
	internal class Settings
	{
		static DataSet settings;
		static DataTable setting_table;
		const string SettingFilename = "settings.xml";
		static Settings()
		{
			settings = new DataSet( "settings" );
			settings.Tables.Add( setting_table = new DataTable( "settings" ) );
			setting_table.Columns.Add( "name", typeof( string ) );
			setting_table.Columns.Add( "value", typeof( object ) );
			try
			{
				settings.ReadXml( SettingFilename );
			}
			catch( Exception e )
			{
			}
		}

		public static bool Read( string name, bool default_val = false )
		{
			DataRow[] setting = settings.Tables["settings"].Select( "name='" + name + "'" );
			if( setting.Length > 0 )
				return (bool)setting[0]["value"];
			else
				Write( name, default_val );
			return default_val;
		}

		public static float Read( string name, float default_val )
		{
			DataRow[] setting = settings.Tables["settings"].Select( "name='" + name + "'" );
			if( setting.Length > 0 )
				return (float)setting[0]["value"];
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
				return (int)setting[0]["value"];
			else
				Write( name, default_val );
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
