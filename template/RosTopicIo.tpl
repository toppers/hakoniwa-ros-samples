using Hakoniwa.PluggableAsset.Communication.Pdu.ROS;
using Hakoniwa.PluggableAsset.Communication.Method.ROS;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using System;
{% if container.has_msg == "true" %}
using RosMessageTypes.{{container.pkg_name.capitalize()}};
{% endif %}
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using Hakoniwa.PluggableAsset.Communication.Pdu.ROS.{{container.pkg_name.upper()}};

namespace Hakoniwa.PluggableAsset.Communication.Method.ROS.{{container.pkg_name.upper()}}
{
    public class RosTopicIo : IRosTopicIo
    {
        private ROSConnection ros;
        private Dictionary<string, Message> topic_data_table = new Dictionary<string, Message>();

        public RosTopicIo()
        {
            ros = ROSConnection.instance;

            foreach (var e in AssetConfigLoader.core_config.ros_topics)
            {
                topic_data_table[e.topic_message_name] = null;
            }

{% for msg in container.ros_topics["fields"]: %}
            ros.Subscribe<M{{msg.topic_type_name}}>("{{msg.topic_message_name}}", M{{msg.topic_type_name}}Change);
{%- endfor %}

        }

{% for msg in container.ros_topics["fields"]: %}
        private void M{{msg.topic_type_name}}Change(M{{msg.topic_type_name}} obj)
        {
            this.topic_data_table["{{msg.topic_message_name}}"] = obj;
        }
{%- endfor %}

        public void Publish(IPduCommTypedData data)
        {
            RosTopicPduCommTypedData typed_data = data as RosTopicPduCommTypedData;
            ros.Send(typed_data.GetDataName(), typed_data.GetTopicData());
        }

        public IPduCommTypedData Recv(string topic_name)
        {
            var cfg = AssetConfigLoader.GetRosTopic(topic_name);
            if (cfg == null)
            {
                throw new System.NotImplementedException();
            }
            else if (this.topic_data_table[topic_name] == null)
            {
                return null;
            }
            return new RosTopicPduCommTypedData(topic_name, cfg.topic_type_name, this.topic_data_table[topic_name]);
        }
    }

}
