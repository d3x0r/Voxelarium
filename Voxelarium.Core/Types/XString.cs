using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.Types
{
	static internal class StringData
	{
		internal static char[] tokens = new char[] { '\n'
			, ' ', '\t'
			// white space usually dropped... recovered if associated with \n
			, '\r'
			// - is sometimes a hyphen.... 
			, '-', '+'
			// grouping quotes
			, '\'', '\"'
			// grouping brackets
			, '(', ')', '{', '}', '[', ']', '<', '>'
			// web addresss have these sometimes...
			, ':', '@'
			// most other punctuators...
			, '.', '\\', '%', '/', ',', ';', '!', '?', '=', '*', '&', '$', '^', '~', '#', '`' };

	}

	enum stringflags
	{

	};


	public class XStringSeg
	{
		//static XString Preload;
		internal String Text;
		internal String PreText; // if this is an 'actual' segment, (parenthized), PreText and Text are rendered around the content.
		XStringSeg next;
		XStringSeg prior;
		public XString actual;
		public int tabs;
		public int spaces;

		public XStringSeg Next
		{
			get
			{
				return next;
			}
			set
			{
				next = value;
			}
		}
		public XStringSeg Prior
		{
			get
			{
				return prior;
			}
			set
			{
				prior = value;
			}
		}

		public XStringSeg( XStringSeg clone )
		{
			this.tabs = clone.tabs;
			this.spaces = clone.spaces;
			this.Text = clone.Text;
			this.PreText = clone.PreText;
		}
		public XStringSeg( string s )
		{
			Text = s;
		}
		public XStringSeg( int tabs, int spaces )
		{
			this.spaces = spaces;
			this.tabs = tabs;
			//Preload = this;
		}
		public XStringSeg( int tabs, int spaces, string s )
		{
			this.spaces = spaces;
			this.tabs = tabs;
			Text = s;
		}
		public XStringSeg( int tabs, int spaces
			, string PreText
			, XString SubText
			, string Text )
		{
			this.spaces = spaces;
			this.tabs = tabs;
			this.Text = Text;
			this.PreText = PreText;
			this.actual = SubText;
		}

		class BurstParenState
		{
			// counts of tabs and spaced leading up to expression
			internal int tabs;
			internal int spaces;
			internal char OpenChar;  // to emit when complete
			internal char CloseChar; // to locate end
			internal StringBuilder sb_previous; // 
			internal XString outdata_previous; // these are segs contained within.
			//internal XString outdata; // these are segs contained within.
		}

		class ParenStack : Stack<BurstParenState>
		{
			internal void PushParen(char open, char close, ref XString outdata, ref StringBuilder sb, ref int tabs, ref int spaces)
			{
				BurstParenState bps = new BurstParenState();
				bps.spaces = spaces;
				bps.tabs = tabs;
				bps.sb_previous = sb;
				bps.outdata_previous = outdata;
				bps.OpenChar = open;
				bps.CloseChar = close;
				this.Push(bps);
				tabs = 0;
				spaces = 0;
				sb = new StringBuilder();
				outdata = new XString();
			}
		}

		static void CollapseStringBuilder( StringBuilder sb, XString outdata, ref int tabs, ref int spaces )
		{
						if( sb.Length > 0 )
						{
							outdata.Append( new XStringSeg( tabs, spaces, sb.ToString() ) );
							sb.Length = 0;
							tabs = 0;
							spaces = 0;
						}
		}

		public static void Burst( XString outdata, string s, bool quote, bool parenthize, bool allow_escapes )
		{
			//lock( Preload )
			{
				XStringSeg seg;
				ParenStack paren_stack = parenthize ? new ParenStack() : null;
				StringBuilder sb = new StringBuilder( s.Length );
				int spaces = 0;
				int tabs = 0;
				char quote_char = '\0';
				bool escape = false;
				int charnum;
				bool elipses = false;

				for( charnum = 0; charnum < s.Length; charnum++ )
				{
					Char character = s[charnum];

					if( parenthize && paren_stack.Count > 0 )
					{
						BurstParenState paren_state = paren_stack.Peek();
						if( character == paren_state.CloseChar )
						{
							if( escape )
								escape = false;
							else
							{
								CollapseStringBuilder( sb, outdata, ref tabs, ref spaces );

								paren_state.outdata_previous.Append( new XStringSeg( 
									paren_state.tabs
									, paren_state.spaces
									, paren_state.OpenChar.ToString()
									, outdata
									, paren_state.CloseChar.ToString() ) );
								paren_stack.Pop();

								outdata = paren_state.outdata_previous;
								sb = paren_state.sb_previous;
								continue;
							}
						}
						else
						{
							if( allow_escapes && character == '\\' )
							{
								escape = true;
								continue; 
							}
						}
					}

					if( quote_char != 0 )
					{
						if( character == quote_char )
						{
							if( escape )
							{
								escape = false;
							}
							else
								quote_char = '\0';
						}
						else
						{
							if( character == '\\' )
								escape = true;
							sb.Append( character );
						}
						continue;
					}

					if( elipses && character != '.' )
					{
						CollapseStringBuilder( sb, outdata, ref tabs, ref spaces );
						elipses = false;
					}
					else if( elipses ) // elipses and character is . - continue
					{
						sb.Append( character );
						continue;
					}

					switch( character )
					{
						case '\n':
							CollapseStringBuilder( sb, outdata, ref tabs, ref spaces );
							outdata.Add( seg = new XStringSeg( 0, 0, "" ) );
							break;
						case ' ':
							CollapseStringBuilder( sb, outdata, ref tabs, ref spaces );
							spaces++;
							break;
						case '\t':
							CollapseStringBuilder( sb, outdata, ref tabs, ref spaces );
							if( spaces != 0 )
							{
								//lprintf( WIDE("Input stream has mangled spaces and tabs.") );
								spaces = 0;
							}
							tabs++;
							break;
						case '\r': // a space space character...
							CollapseStringBuilder( sb, outdata, ref tabs, ref spaces );
							break;
						case '.': // handle multiple periods grouped (elipses)
							//goto NormalPunctuation;
							{
								char c;
								if( ( !elipses &&
									( ( c = ( ( charnum + 1 < s.Length ) ? s[charnum + 1] : (char)0 ) ) != 0 ) &&
									  ( c == '.' ) ) )
								{
									CollapseStringBuilder( sb, outdata, ref tabs, ref spaces );
									sb.Append( "." );
									elipses = true;
									break;
								}
								if( ( ( c = ( ( charnum + 1 < s.Length ) ? s[charnum + 1] : (char)0 ) ) != 0 ) &&
									 ( c >= '0' && c <= '9' ) )
								{
									// gather together as a floating point number...
									sb.Append( character );
									break;
								}
							}
							goto continue_case;
						case '-':  // work seperations flaming-long-sword
							goto continue_case;
						case '+':
						continue_case:
							{
								int c;
								if( ( ( c = ( ( charnum + 1 < s.Length ) ? s[charnum + 1] : 0 ) ) != 0 ) &&
									 ( c >= '0' && c <= '9' ) )
								{
									CollapseStringBuilder( sb, outdata, ref tabs, ref spaces );
									// gather together as a sign indication on a number.
									sb.Append( character );
									break;
								}
							}
							goto continue_case2;
						case '\'': // single quote bound
						case '\"': // double quote bound
							if( quote )
							{
								CollapseStringBuilder( sb, outdata, ref tabs, ref spaces );
								paren_stack.PushParen( character, character, ref outdata, ref sb, ref tabs, ref spaces );
								break;
							}
							goto continue_case2;
						case '(': // expression bounders
							if( parenthize )
							{
								CollapseStringBuilder( sb, outdata, ref tabs, ref spaces );
								paren_stack.PushParen( character, ')', ref outdata, ref sb, ref tabs, ref spaces );
								break;
							}
							goto continue_case2;
						case '{':
							if( parenthize )
							{
								CollapseStringBuilder( sb, outdata, ref tabs, ref spaces );
								paren_stack.PushParen( character, '}', ref outdata, ref sb, ref tabs, ref spaces );
								break;
							}
							goto continue_case2;
						case '[':
							if( parenthize )
							{
								CollapseStringBuilder( sb, outdata, ref tabs, ref spaces );
								paren_stack.PushParen( character, ']', ref outdata, ref sb, ref tabs, ref spaces );
								break;
							}
							goto continue_case2;
						case '<':
							if( parenthize )
							{
								CollapseStringBuilder( sb, outdata, ref tabs, ref spaces );
								paren_stack.PushParen( character, '>', ref outdata, ref sb, ref tabs, ref spaces );
								break;
							}
							goto continue_case2;
						case ')': // expression closers
						case '}':
						case ']':
						case '>':

						case '\\': // escape next thingy... unusable in c processor
						case ':':  // internet addresses
						case '@':  // email addresses
						case '%':
						case '/':
						case ',':
						case ';':
						case '!':
						case '?':
						case '=':
						case '*':
						case '&':
						case '$':
						case '^':
						case '~':
						case '#':
						case '`':
						continue_case2:
							//         NormalPunctuation:
							CollapseStringBuilder( sb, outdata, ref tabs, ref spaces );

							sb.Append( character );

							/*
							if( parenthize && paren_stack.Count > 0 )
							{
								BurstParenState state = paren_stack.Peek();

								// if it is in a paren condition, then
								// don't put this out yet... otherwise emit directly.
								state.outdata.Add( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
								if( state.outdata.Count > 1 )
								{
									seg.prior = state.outdata[state.outdata.Count - 2];
									state.outdata[state.outdata.Count - 2].next = seg;
								}
								sb.Length = 0;
								tabs = 0;
								spaces = 0;
							}
							else
							*/
							CollapseStringBuilder( sb, outdata, ref tabs, ref spaces );
							break;

						default:
							if( elipses )
							{
								if( sb.Length > 0 )
								{
									outdata.Append( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
									sb.Length = 0;
									tabs = 0;
									spaces = 0;
								}
								elipses = false;
							}
							sb.Append( character );
							break;
					}
				}
				// here at the end I have collection without a push.
				if( sb.Length > 0 )
				{
					outdata.Append( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
					sb.Length = 0;
					tabs = 0;
					spaces = 0;
				}
			}
		}


		public static XString Burst( string s, bool quote, bool parenthize )
		{
			XString seglist = new XString();
			Burst( seglist, s, quote, parenthize, true );
			return seglist;
		}
		public static XString Burst( string s )
		{
			return Burst( s, false, false );
		}


		public static void TextParse( List<XStringSeg> outdata, String input, String punctuation, String filter_space, bool bTabs, bool bSpaces )
		{
			/* takes a line of input and creates a line equivalent to it, but
			   burst into its block peices.*/
			StringBuilder sb;
			XStringSeg seg;

			int charnum;
			int size;

			char character;
			bool elipses = false;
			int spaces = 0, tabs = 0;

			if( input == null )        // if nothing new to process- return nothing processed.
				return;

			sb = new StringBuilder();

			//while (input)  // while there is data to process...
			{
				//tempText = GetText(input);  // point to the data to process...
				size = input.Length;
				spaces = 0;
				tabs = 0;
				//Log1( WIDE("Assuming %d spaces... "), spaces );
				for( charnum = 0; ( charnum < size ); charnum++ ) // while not at the
				// end of the line.
				{
					character = input[charnum];
					if( elipses && character != '.' )
					{
						if( sb.Length > 0 )
						{
							outdata.Add( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
							if( outdata.Count > 1 )
							{
								seg.prior = outdata[outdata.Count - 2];
								outdata[outdata.Count - 2].next = seg;
							}
							sb.Length = 0;
							tabs = 0;
							spaces = 0;
						}
						elipses = false;
					}
					else if( elipses ) // elipses and character is . - continue
					{
						sb.Append( character );
						continue;
					}
					if( filter_space != null && filter_space.IndexOf( character ) > -1 )
					{
						if( sb.Length > 0 )
						{
							outdata.Add( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
							if( outdata.Count > 1 )
							{
								seg.prior = outdata[outdata.Count - 2];
								outdata[outdata.Count - 2].next = seg;
							}
							sb.Length = 0;
							tabs = 0;
							spaces = 0;
						}
						spaces++;
					}
					else if( punctuation != null && punctuation.IndexOf( character ) > -1 )
					{
						switch( character )
						{
							case '.': // handle multiple periods grouped (elipses)
								//goto NormalPunctuation;
								{
									char c;
									if( ( !elipses &&
										  ( ( c = ( ( charnum + 1 < input.Length ) ? input[charnum + 1] : (char)0 ) ) != 0 ) &&
										( c == '.' ) ) )
									{
										if( sb.Length > 0 )
										{
											outdata.Add( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
											if( outdata.Count > 1 )
											{
												seg.prior = outdata[outdata.Count - 2];
												outdata[outdata.Count - 2].next = seg;
											}
											sb.Length = 0;
											tabs = 0;
											spaces = 0;
										}
										sb.Append( '.' );
										elipses = true;
										break;
									}
									if( ( ( c = ( ( charnum + 1 < input.Length ) ? input[charnum + 1] : (char)0 ) ) != 0 ) &&
									  ( c >= '0' && c <= '9' ) )
									{
										// gather together as a floating point number...
										sb.Append( character );
										break;
									}
								}
								goto skip1;
							case '-':  // work seperations flaming-long-sword
							case '+':
								{
									int c;
									if( ( ( c = ( ( charnum + 1 < input.Length ) ? input[charnum + 1] : (char)0 ) ) != 0 ) &&
										( c >= '0' && c <= '9' ) )
									{
										if( sb.Length > 0 )
										{
											outdata.Add( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
											if( outdata.Count > 1 )
											{
												seg.prior = outdata[outdata.Count - 2];
												outdata[outdata.Count - 2].next = seg;
											}
											sb.Length = 0;
											tabs = 0;
											spaces = 0;
											// gather together as a sign indication on a number.
										}
										sb.Append( character );
										break;
									}
								}
								//         NormalPunctuation:
								if( sb.Length > 0 )
								{
									outdata.Add( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
									if( outdata.Count > 1 )
									{
										seg.prior = outdata[outdata.Count - 2];
										outdata[outdata.Count - 2].next = seg;
									}
									sb.Length = 0;
									tabs = 0;
									spaces = 0;
									sb.Append( character );
									outdata.Add( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
									if( outdata.Count > 1 )
									{
										seg.prior = outdata[outdata.Count - 2];
										outdata[outdata.Count - 2].next = seg;
									}
									sb.Length = 0;
								}
								else
								{
									sb.Append( character );
									outdata.Add( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
									if( outdata.Count > 1 )
									{
										seg.prior = outdata[outdata.Count - 2];
										outdata[outdata.Count - 2].next = seg;
									}
									sb.Length = 0;
									tabs = 0;
									spaces = 0;
								}
								break;
							default:
							skip1:
								if( elipses )
								{
									if( sb.Length > 0 )
									{
										outdata.Add( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
										if( outdata.Count > 1 )
										{
											seg.prior = outdata[outdata.Count - 2];
											outdata[outdata.Count - 2].next = seg;
										}
										sb.Length = 0;
										tabs = 0;
										spaces = 0;
									}
									elipses = false;
								}
								if( sb.Length > 0 )
								{
									outdata.Add( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
									if( outdata.Count > 1 )
									{
										seg.prior = outdata[outdata.Count - 2];
										outdata[outdata.Count - 2].next = seg;
									}
									sb.Length = 0;
									tabs = 0;
									spaces = 0;

									sb.Append( character );

									outdata.Add( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
									if( outdata.Count > 1 )
									{
										seg.prior = outdata[outdata.Count - 2];
										outdata[outdata.Count - 2].next = seg;
									}
									sb.Length = 0;
								}
								else
								{
									sb.Append( character );
									outdata.Add( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
									if( outdata.Count > 1 )
									{
										seg.prior = outdata[outdata.Count - 2];
										outdata[outdata.Count - 2].next = seg;
									}
									sb.Length = 0;
									tabs = 0;
									spaces = 0;
								}
								break;
						}

					}
					else switch( character )
						{
							case '\n':
								if( sb.Length > 0 )
								{
									outdata.Add( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
									if( outdata.Count > 1 )
									{
										seg.prior = outdata[outdata.Count - 2];
										outdata[outdata.Count - 2].next = seg;
									}
									sb.Length = 0;
									tabs = 0;
									spaces = 0;
								}

								outdata.Add( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
								if( outdata.Count > 1 )
								{
									seg.prior = outdata[outdata.Count - 2];
									outdata[outdata.Count - 2].next = seg;
								}
								sb.Length = 0;
								tabs = 0;
								spaces = 0;
								break;
							case ' ':
								if( bSpaces )
								{
									if( sb.Length > 0 )
									{
										outdata.Add( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
										if( outdata.Count > 1 )
										{
											seg.prior = outdata[outdata.Count - 2];
											outdata[outdata.Count - 2].next = seg;
										}
										sb.Length = 0;
										tabs = 0;
										spaces = 0;
									}
									spaces++;
									break;
								}
								goto skip1;
							case '\t':
								if( bTabs )
								{
									if( sb.Length > 0 )
									{
										outdata.Add( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
										if( outdata.Count > 1 )
										{
											seg.prior = outdata[outdata.Count - 2];
											outdata[outdata.Count - 2].next = seg;
										}
										sb.Length = 0;
										tabs = 0;
										spaces = 0;
									}
									if( spaces > 0 )
									{
										//lprintf( WIDE("Input stream has mangled spaces and tabs.") );
										spaces = 0; // assume that the tab takes care of appropriate spacing
									}
									tabs++;
									break;
								}
								goto skip1;
							case '\r': // a space space character...
								if( sb.Length > 0 )
								{
									outdata.Add( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
									if( outdata.Count > 1 )
									{
										seg.prior = outdata[outdata.Count - 2];
										outdata[outdata.Count - 2].next = seg;
									}
									sb.Length = 0;
									tabs = 0;
									spaces = 0;
								}
								break;
							case '.': // handle multiple periods grouped (elipses)
								//goto NormalPunctuation;
								{
									char c;
									if( ( !elipses &&
										  ( ( c = ( ( charnum + 1 < input.Length ) ? input[charnum + 1] : (char)0 ) ) != 0 ) &&
										( c == '.' ) ) )
									{
										if( sb.Length > 0 )
										{
											outdata.Add( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
											if( outdata.Count > 1 )
											{
												seg.prior = outdata[outdata.Count - 2];
												outdata[outdata.Count - 2].next = seg;
											}
											sb.Length = 0;
											tabs = 0;
											spaces = 0;
										}
										sb.Append( '.' );
										elipses = true;
										break;
									}
									if( ( ( c = ( ( charnum + 1 < input.Length ) ? input[charnum + 1] : (char)0 ) ) != 0 ) &&
									  ( c >= '0' && c <= '9' ) )
									{
										// gather together as a floating point number...
										sb.Append( character );
										break;
									}
								}
								goto skip1;
#if asefsdf
						case '-':  // work seperations flaming-long-sword
						case '+':
							{
								int c;
								if( ( ( c = ( ( charnum + 1 < input.Length ) ? input[charnum + 1] : (char)0 ) ) != 0 ) &&
									( c >= '0' && c <= '9' ) )
								{
									if( sb.Length > 0 )
									{
										outdata.Add( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
										if( outdata.Count > 1 )
										{
											seg.prior = outdata[outdata.Count - 2];
											outdata[outdata.Count - 2].next = seg;
										}
										sb.Length = 0;
										tabs = 0;
										spaces = 0;
										// gather together as a sign indication on a number.
									}
									sb.Append( character );
									break;
								}
							}
							//         NormalPunctuation:
							if( sb.Length > 0 )
							{
								outdata.Add( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
								if( outdata.Count > 1 )
								{
									seg.prior = outdata[outdata.Count - 2];
									outdata[outdata.Count - 2].next = seg;
								}
								sb.Length = 0;
								tabs = 0;
								spaces = 0;
								sb.Append( character );
								outdata.Add( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
								if( outdata.Count > 1 )
								{
									seg.prior = outdata[outdata.Count - 2];
									outdata[outdata.Count - 2].next = seg;
								}
								sb.Length = 0;
							}
							else
							{
								sb.Append( character );
								outdata.Add( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
								if( outdata.Count > 1 )
								{
									seg.prior = outdata[outdata.Count - 2];
									outdata[outdata.Count - 2].next = seg;
								}
								sb.Length = 0;
								tabs = 0;
								spaces = 0;
							}
							break;
#endif
							default:
							skip1:
								if( elipses )
								{
									if( sb.Length > 0 )
									{
										outdata.Add( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
										if( outdata.Count > 1 )
										{
											seg.prior = outdata[outdata.Count - 2];
											outdata[outdata.Count - 2].next = seg;
										}
										sb.Length = 0;
										tabs = 0;
										spaces = 0;
									}
									elipses = false;
								}
								sb.Append( character );
								break;
						}
				}
				//input = NEXTLINE( input );
			}

			if( sb.Length > 0 ) // any generic outstanding data?
			{
				outdata.Add( seg = new XStringSeg( tabs, spaces, sb.ToString() ) );
				if( outdata.Count > 1 )
				{
					seg.prior = outdata[outdata.Count - 2];
					outdata[outdata.Count - 2].next = seg;
				}
				sb.Length = 0;
				tabs = 0;
				spaces = 0;
			}
		}



		public void Expand( StringBuilder sb )
		{
			for( int i = 0; i < tabs; i++ )
				sb.Append( '\t' );
			for( int i = 0; i < spaces; i++ )
				sb.Append( ' ' );

			if( PreText != null )
				sb.Append( PreText );
			if( actual != null )
			{
				XStringSeg tmp = actual.firstseg;
				for( ; tmp != null; tmp = tmp.next )
					tmp.Expand( sb );
			}
			if( Text != null )
				sb.Append( Text );
		}

		public string Expand()
		{
			StringBuilder sb = new StringBuilder();
			XStringSeg cur = this.Next;
			sb.Append( this.Text );
			for( ; cur != null; cur = cur.next )
				cur.Expand( sb );
			return sb.ToString();
		}


		public override string ToString()
		{
			return Text;
		}

		public static implicit operator String( XStringSeg seg )
		{
			return seg.Text;
		}

		internal XStringSeg Clone()
		{
			return new XStringSeg( this );
		}

		public static XStringSeg Append( XStringSeg phrase, XStringSeg xStringSeg )
		{
			XStringSeg save = phrase;
			while( phrase != null && phrase.next != null )
				phrase = phrase.next;
			if( phrase != null )
			{
				phrase.next = xStringSeg;
				xStringSeg.prior = phrase;
			}
			else
				save = xStringSeg;
			return save;
		}

		public void Append( XStringSeg xStringSeg )
		{
			XStringSeg phrase = this;
			while( phrase != null && phrase.next != null )
				phrase = phrase.next;
			if( phrase != null )
			{
				phrase.next = xStringSeg;
				xStringSeg.prior = phrase;
			}
		}

		public void Replace( String new_content )
		{
			PreText = null;
			Text = new_content;
			actual = null;
		}

		/// <summary>
		/// Clear spaces and tabs before this segment
		/// </summary>
		public void Originate()
		{
			this.tabs = 0;
			this.spaces = 0;
		}
		/// <summary>
		/// Allow lazy empty comparison
		/// </summary>
		/// <param name="seg"></param>
		/// <returns></returns>
		public static implicit operator bool( XStringSeg seg )
		{
			return seg != null;
		}
		public char this[int index]
		{
			get
			{
				return this.Text[index];
			}
		}
	}


	public class XString :  List<XStringSeg>
	{
		public void Add( XString segments )
		{
			foreach( XStringSeg seg in segments )
				this.Add( seg );
		}

		public XStringSeg firstseg
		{
			get
			{
				return this[0];
			}
			set
			{
				this[0] = value;
			}
		}

		public XString()
		{
		}

		public XString( String s, bool quote = false, bool parenthize = false, bool allow_escapes = true )
		{
			XStringSeg.Burst( this, s, quote, parenthize, allow_escapes );
		}

		public XString( string s, String separator_chars, String whitespace_chars, bool tabs, bool spaces )
		{
			XStringSeg.TextParse( this, s, separator_chars, whitespace_chars, tabs, spaces );
		}

		/// <summary>
		/// replace string content with new content
		/// </summary>
		/// <param name="new_content"></param>
		public void Replace( String new_content )
		{
			this.Clear();
			this.Append( new XStringSeg( new_content ) );
		}

		public static implicit operator String( XString segs )
		{
			StringBuilder sb = new StringBuilder();
			foreach( XStringSeg s in segs )
			{
				s.Expand( sb );
			}
			return sb.ToString();
		}

		public override string ToString()
		{
			return (String)this;
		}

		new public void RemoveAt( int index )
		{
			if( this[index].Prior != null )
				this[index].Prior.Next = this[index].Next;
			if( this[index].Next != null )
				this[index].Next.Prior = this[index].Prior;
			base.RemoveAt( index );
			//base.re
		}

		public void Append( XStringSeg seg )
		{
			if( Count > 0 )
			{
				seg.Prior = this[Count - 1];
				this[Count - 1].Next = seg;
			}
			this.Add( seg );
		}

		public void Originate()
		{
			if( Count > 0 )
				this[0].Originate();
		}

		/// <summary>
		/// Allow lazy empty comparison
		/// </summary>
		/// <param name="seg"></param>
		/// <returns></returns>
		public static implicit operator bool( XString seg )
		{
			return seg != null;
		}

		public static XString Burst( string s )
		{
			return XStringSeg.Burst( s, false, false );
		}
	}


}
