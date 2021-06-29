using Hakoniwa.PluggableAsset.Communication.Pdu;
using Hakoniwa.PluggableAsset.Communication.Pdu.ROS;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RosMessageTypes.Hackev;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.ROS
{
    class RosTopicPduWriterConverter : IPduWriterConverter
    {
        public IPduCommData ConvertToIoData(IPduWriter src)
        {
            RosTopicPduWriter pdu_writer = src as RosTopicPduWriter;
            return new RosTopicPduCommTypedData(pdu_writer);
        }
        static public Message ConvertToMessage(RosTopicPduWriter pdu_writer)
        {
{% for msg in container.msgs: %}
            if (pdu_writer.GetTypeName().Equals("{{msg.name}}"))
            {
                M{{msg.name}} tmp_topic = new M{{msg.name}}();
{% for item in msg.json_data["fields"]: %}
                tmp_topic.{{item["name"]}} = pdu_writer.GetReadOps().GetData{{container.to_conv(item["type"])}}("{{item["name"]}}");
{%- endfor %}

                return tmp_topic;
            }
{%- endfor %}
            
            throw new InvalidCastException("Can not find ros message type:" + pdu_writer.GetTypeName());

        }
    }

}
