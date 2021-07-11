using Hakoniwa.PluggableAsset.Communication.Pdu;
using Hakoniwa.PluggableAsset.Communication.Pdu.ROS;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RosMessageTypes.Std;
using RosMessageTypes.Ev3;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using Hakoniwa.PluggableAsset.Communication.Pdu.ROS.{{container.pkg_name.upper()}};

namespace Hakoniwa.PluggableAsset.Communication.Pdu.ROS.{{container.pkg_name.upper()}}
{
    class RosTopicPduReaderConverter : IPduReaderConverter
    {
        public IPduCommData ConvertToIoData(IPduReader src)
        {
            RosTopicPduReader pdu_reader = src as RosTopicPduReader;
            return new RosTopicPduCommTypedData(pdu_reader);
        }
        
{% for msg in container.msgs: %}
        private void ConvertToPdu(M{{msg.name}} src, IPduWriteOperation dst)
        {
{%- 	for item in msg.json_data["fields"]: %}
{%-		if (container.is_primitive(item["type"]) or container.is_primitive_array(item["type"])): %}
            dst.SetData("{{item["name"]}}", src.{{item["name"]}});
{%-		else: %}
{%-			if (container.is_array(item["type"])): %}
            foreach (var e in dst.Refs("{{item["name"]}}"))
            {
                int index = Array.IndexOf(dst.Refs("{{item["name"]}}"), e);
                if (src.{{item["name"]}}[index] == null) {
                    src.{{item["name"]}}[index] = new M{{container.get_array_type(item["type"])}}();
                }
				ConvertToPdu(src.{{item["name"]}}[index], e.GetPduWriteOps());
            }
{%-			else: %}
			ConvertToPdu(src.{{item["name"]}}, dst.Ref("{{item["name"]}}").GetPduWriteOps());
{%-			endif %}
{%-		endif %}

{%- 	endfor %}
        }
{%- endfor %}

        public void ConvertToPduData(IPduCommData src, IPduReader dst)
        {
            RosTopicPduCommTypedData ros_topic = src as RosTopicPduCommTypedData;

            RosTopicPduReader ros_pdu_reader = dst as RosTopicPduReader;
{% for topic in container.ros_topics["fields"]: %}
            if (ros_pdu_reader.GetTypeName().Equals("{{topic.topic_type_name}}"))
            {
                var ros_topic_data = ros_topic.GetTopicData() as M{{topic.topic_type_name}};
                ConvertToPdu(ros_topic_data, dst.GetWriteOps());
                return;
            }
{%- endfor %}
            throw new InvalidCastException("Can not find ros message type:" + ros_pdu_reader.GetTypeName());

        }
        static public Message ConvertToMessage(RosTopicPduReader pdu_reader)
        {
            return RosTopicPduWriterConverter.ConvertToMessage(pdu_reader.GetReadOps(), pdu_reader.GetTypeName());
        }
    }
}
