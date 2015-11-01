using System;
using System.Drawing;
using Voxelarium.Core.Support;

namespace Voxelarium.Core.Types
{
	public class XColor : System.Xml.Serialization.IXmlSerializable
	{
		System.Drawing.Color color;


		static public implicit operator Color( XColor m )
		{
			return m.color;
		}

		static public implicit operator XColor( Color m )
		{
			return new XColor( m );
		}

		static public implicit operator uint( XColor m )
		{
			return (uint)m.color.ToArgb();
		}

		public XColor()
		{
			color = new Color();
		}

		public XColor( Color c )
		{
			color = c;
		}

		public XColor( XColor xc )
		{
			color = xc.color;
		}

		public XColor( int p )
		{
			color = Color.FromArgb( p );
		}



		public enum ColorFormat
		{
			NamedColor,
			ARGBColor
		}

		public string SerializeColor( )
		{
			if( color.IsNamedColor )
				return string.Format( "{0}:{1}",
					ColorFormat.NamedColor, color.Name );
			else
				return string.Format( "{0}:{1}:{2}:{3}:{4}",
					ColorFormat.ARGBColor,
					color.A, color.R, color.G, color.B );
		}

		public static Color DeserializeColor( string color )
		{
			byte a, r, g, b;

			string[] pieces = color.Split( new char[] { ':' } );
			try
			{
				ColorFormat colorType = (ColorFormat)
					Enum.Parse( typeof( ColorFormat ), pieces[0], true );

				switch( colorType )
				{
					case ColorFormat.NamedColor:
						return Color.FromName( pieces[1] );

					case ColorFormat.ARGBColor:
						a = byte.Parse( pieces[1] );
						r = byte.Parse( pieces[2] );
						g = byte.Parse( pieces[3] );
						b = byte.Parse( pieces[4] );

						return Color.FromArgb( a, r, g, b );
				}
			}
			catch( Exception e )
			{
				Log.log( "Color format error in [{0]] : {1}", color, e.Message);
			}
			return Color.Empty;
		}

		public override string ToString()
		{
			return color.ToString();
		}

		System.Xml.Schema.XmlSchema System.Xml.Serialization.IXmlSerializable.GetSchema()
		{
			throw new NotImplementedException();
		}

		void System.Xml.Serialization.IXmlSerializable.ReadXml( System.Xml.XmlReader reader )
		{
			String value = reader.ReadString();
			color = DeserializeColor( value );
		}

		void System.Xml.Serialization.IXmlSerializable.WriteXml( System.Xml.XmlWriter writer )
		{
			writer.WriteString( this.SerializeColor() );
		}
	}
}
