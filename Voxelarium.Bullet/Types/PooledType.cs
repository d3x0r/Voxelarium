using System.Collections.Generic;

using System.Diagnostics;

namespace Bullet.Types
{

	public class PooledType<T> where T : new()
	{
		private Stack<T> m_pool = new Stack<T>();
		int created;
		public T Get()
		{
			if( m_pool.Count == 0 )
			{
				created++;
				m_pool.Push( new T() );
			}
			return m_pool.Pop();
		}

		public void Free( T obj )
		{
			Debug.Assert( !m_pool.Contains( obj ), "Object already in pool" );
			m_pool.Push( obj );
		}

	}
}
