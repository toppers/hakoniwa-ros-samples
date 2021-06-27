using Hakoniwa.PluggableAsset.Communication.Pdu.ROS;
using Hakoniwa.PluggableAsset.Communication.Method.ROS;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using System;
using RosMessageTypes.RoboticsDemo;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace Hakoniwa.PluggableAsset.Communication.Method.ROS
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


            ros.Subscribe<MSensor>("sensor", MSensorChange);
            ros.Subscribe<MActuator>("actuator", MActuatorChange);

        }


        private void MSensorChange(MSensor obj)
        {
            this.topic_data_table["sensor"] = obj;
        }
        private void MActuatorChange(MActuator obj)
        {
            this.topic_data_table["actuator"] = obj;
        }

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
