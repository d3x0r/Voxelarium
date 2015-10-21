using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.UI
{
	public class EventManager
	{
		internal LinkedList<EventConsumer> ConsumerList = new LinkedList<EventConsumer>();

		public void AddConsumer_ToTail( EventConsumer EventConsumer )
		{
			ConsumerList.AddLast( EventConsumer );
		}

		public void RemoveConsumer( EventConsumer EventConsumer )
		{
			ConsumerList.Remove( EventConsumer );
		}


	}
}
