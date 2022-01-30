using Hakoniwa.PluggableAsset.Communication.Pdu;
using Hakoniwa.PluggableAsset.Communication.Pdu.ROS;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Hakoniwa.PluggableAsset.Communication.Pdu.ROS.MINI;

using RosMessageTypes.Geometry;
using RosMessageTypes.Rule;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.ROS.MINI
{
    public class RosTopicPduWriterConverter : IPduWriterConverter
    {
        public IPduCommData ConvertToIoData(IPduWriter src)
        {
            RosTopicPduWriter pdu_writer = src as RosTopicPduWriter;
            return new RosTopicPduCommTypedData(pdu_writer);
        }
        

        static private void ConvertToMessage(IPduReadOperation src, HakoEnvMsg dst)
        {
			dst.simtime = src.GetDataUInt64("simtime");
        }
        static private void ConvertToMessage(IPduReadOperation src, TwistMsg dst)
        {
            ConvertToMessage(src.Ref("linear").GetPduReadOps(), dst.linear);
            ConvertToMessage(src.Ref("angular").GetPduReadOps(), dst.angular);
        }
        static private void ConvertToMessage(IPduReadOperation src, Vector3Msg dst)
        {
			dst.x = src.GetDataFloat64("x");
			dst.y = src.GetDataFloat64("y");
			dst.z = src.GetDataFloat64("z");
        }
        
        
        static public Message ConvertToMessage(IPduReadOperation src, string type)
        {

            if (type.Equals("rule_msgs/HakoEnv"))
            {
            	HakoEnvMsg ros_topic = new HakoEnvMsg();
                ConvertToMessage(src, ros_topic);
                return ros_topic;
            }
            if (type.Equals("geometry_msgs/Twist"))
            {
            	TwistMsg ros_topic = new TwistMsg();
                ConvertToMessage(src, ros_topic);
                return ros_topic;
            }
            if (type.Equals("geometry_msgs/Twist"))
            {
            	TwistMsg ros_topic = new TwistMsg();
                ConvertToMessage(src, ros_topic);
                return ros_topic;
            }
            if (type.Equals("geometry_msgs/Twist"))
            {
            	TwistMsg ros_topic = new TwistMsg();
                ConvertToMessage(src, ros_topic);
                return ros_topic;
            }
            throw new InvalidCastException("Can not find ros message type:" + type);
        }
        
        static public Message ConvertToMessage(RosTopicPduWriter pdu_writer)
        {
            return ConvertToMessage(pdu_writer.GetReadOps(), pdu_writer.GetTypeName());
        }
    }

}
