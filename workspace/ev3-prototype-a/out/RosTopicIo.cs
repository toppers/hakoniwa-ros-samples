using Hakoniwa.PluggableAsset.Communication.Pdu.ROS;
using Hakoniwa.PluggableAsset.Communication.Method.ROS;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using System;
using RosMessageTypes.Hackev;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Geometry;

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


            ros.Subscribe<MLaserScan>("scan", MLaserScanChange);
            ros.Subscribe<MImu>("imu", MImuChange);
            ros.Subscribe<MTwist>("cmd_vel", MTwistChange);

        }


        private void MLaserScanChange(MLaserScan obj)
        {
            this.topic_data_table["scan"] = obj;
        }
        private void MImuChange(MImu obj)
        {
            this.topic_data_table["imu"] = obj;
        }
        private void MTwistChange(MTwist obj)
        {
            this.topic_data_table["cmd_vel"] = obj;
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
