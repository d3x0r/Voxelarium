using System;
using System.Collections.Generic;

using System.Text;
using System.Diagnostics;

namespace Bullet.Types
{

    public class PooledType<T> where T : new()
    {

        public T Get()
        {
            if (m_pool.Count == 0)
            {
                m_pool.Push(new T());
            }
            return m_pool.Pop();
        }

        public void Free(T obj)
        {
            Debug.Assert(!m_pool.Contains(obj));
            m_pool.Push(obj);
        }


        private Stack<T> m_pool = new Stack<T>();
    }
}
