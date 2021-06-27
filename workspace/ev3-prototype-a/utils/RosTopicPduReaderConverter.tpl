using Hakoniwa.PluggableAsset.Communication.Pdu;
using Hakoniwa.PluggableAsset.Communication.Pdu.ROS;
using RosMessageTypes.RoboticsDemo;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.ROS
{
    class RosTopicPduReaderConverter : IPduReaderConverter
    {
        public IPduCommData ConvertToIoData(IPduReader src)
        {
            RosTopicPduReader pdu_reader = src as RosTopicPduReader;
            return new RosTopicPduCommTypedData(pdu_reader);
        }

        public void ConvertToPduData(IPduCommData src, IPduReader dst)
        {
            RosTopicPduCommTypedData ros_topic = src as RosTopicPduCommTypedData;

            RosTopicPduReader ros_pdu_reader = dst as RosTopicPduReader;
{% for msg in container.msgs: %}
            if (ros_pdu_reader.GetTypeName().Equals("{{msg.name}}"))
            {
                var ros_topic_data = ros_topic.GetTopicData() as M{{msg.name}};
{% for item in msg.json_data["fields"]: %}
                dst.GetWriteOps().SetData("{{item["name"]}}", ros_topic_data.{{item["name"]}});
{%- endfor %}
                return;
            }
{%- endfor %}
            throw new InvalidCastException("Can not find ros message type:" + ros_pdu_reader.GetTypeName());

        }
        static public Message ConvertToMessage(RosTopicPduReader pdu_reader)
        {
{% for msg in container.msgs: %}
            if (pdu_reader.GetTypeName().Equals("{{msg.name}}"))
            {
                M{{msg.name}} tmp_topic = new M{{msg.name}}();
{% for item in msg.json_data["fields"]: %}
                tmp_topic.{{item["name"]}} = pdu_reader.GetReadOps().GetData{{container.to_conv(item["type"])}}("{{item["name"]}}");
{%- endfor %}
                return tmp_topic;
            }
{%- endfor %}
            throw new InvalidCastException("Can not find ros message type:" + pdu_reader.GetTypeName());
        }
    }
}
