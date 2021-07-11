using Hakoniwa.PluggableAsset.Communication.Pdu;
using Hakoniwa.PluggableAsset.Communication.Pdu.ROS;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RosMessageTypes.Std;
{% if container.has_msg == "true" %}
using RosMessageTypes.{{container.pkg_name.capitalize()}};
{% endif %}
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using Hakoniwa.PluggableAsset.Communication.Pdu.ROS.{{container.pkg_name.upper()}};

namespace Hakoniwa.PluggableAsset.Communication.Pdu.ROS.{{container.pkg_name.upper()}}
{
    class RosTopicPduWriterConverter : IPduWriterConverter
    {
        public IPduCommData ConvertToIoData(IPduWriter src)
        {
            RosTopicPduWriter pdu_writer = src as RosTopicPduWriter;
            return new RosTopicPduCommTypedData(pdu_writer);
        }
        
{% for msg in container.msgs: %}
        static private void ConvertToMessage(IPduReadOperation src, M{{msg.name}} dst)
        {
{%- 	for item in msg.json_data["fields"]: -%}
{%-		if (container.is_primitive(item["type"]) or container.is_primitive_array(item["type"])): %}
			dst.{{item["name"]}} = src.GetData{{container.to_conv(item["type"])}}("{{item["name"]}}");
{%-		else: %}
{%-			if (container.is_array(item["type"])): %}
            foreach (var e in src.Refs("{{item["name"]}}"))
            {
                ConvertToMessage(e.GetPduReadOps(), dst.{{item["name"]}}[Array.IndexOf(src.Refs("{{item["name"]}}"), e)]);
            }
{%-			else: %}
            ConvertToMessage(src.Ref("{{item["name"]}}").GetPduReadOps(), dst.{{item["name"]}});
{%-			endif %}
{%-		endif %}
{%- 	endfor %}
        }
{%- endfor %}
        
        
        static public Message ConvertToMessage(IPduReadOperation src, string type)
        {
{% for topic in container.ros_topics["fields"]: %}
            if (type.Equals("{{topic.topic_type_name}}"))
            {
            	M{{topic.topic_type_name}} ros_topic = new M{{topic.topic_type_name}}();
                ConvertToMessage(src, ros_topic);
                return ros_topic;
            }
{%- endfor %}
            throw new InvalidCastException("Can not find ros message type:" + type);
        }
        
        static public Message ConvertToMessage(RosTopicPduWriter pdu_writer)
        {
            return ConvertToMessage(pdu_writer.GetReadOps(), pdu_writer.GetTypeName());
        }
    }

}
