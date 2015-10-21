using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

namespace Bullet
{
    static internal class Assert
    {

        [Conditional( "DEBUG" )]

        internal static void x( bool expr )
        {
            Debug.Assert( expr );
        }
    }
}
