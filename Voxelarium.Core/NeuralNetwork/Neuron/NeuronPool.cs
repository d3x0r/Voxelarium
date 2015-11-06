using Voxelarium.NeuralNetwork.Types;

namespace Voxelarium.NeuralNetwork
{
	internal class NeuronPool: GenericSet< Neuron >
	{
		Brain _brain;
		public NeuronPool( Brain brain )
			: base( )
		{
			_brain = brain;
		}

		public static implicit operator Neuron ( NeuronPool pool )
		{
			Neuron n = pool.Get();
			IBrainMatter matter = (IBrainMatter)n;
			matter.Init( pool._brain );
			return n;
		}
	}
}
