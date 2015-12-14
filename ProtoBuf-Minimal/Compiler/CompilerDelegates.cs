#if FEAT_COMPILER
using System;


namespace ProtoBuf.Compiler
{
    internal delegate void ProtoSerializer(Type useType,object value, ProtoWriter dest);
    internal delegate object ProtoDeserializer(object value, ProtoReader source);
}
#endif