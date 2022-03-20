//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Ev3
{
    [Serializable]
    public class Ev3PduTouchSensorMsg : Message
    {
        public const string k_RosMessageName = "ev3_msgs/Ev3PduTouchSensor";
        public override string RosMessageName => k_RosMessageName;

        public uint value;

        public Ev3PduTouchSensorMsg()
        {
            this.value = 0;
        }

        public Ev3PduTouchSensorMsg(uint value)
        {
            this.value = value;
        }

        public static Ev3PduTouchSensorMsg Deserialize(MessageDeserializer deserializer) => new Ev3PduTouchSensorMsg(deserializer);

        private Ev3PduTouchSensorMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.value);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.value);
        }

        public override string ToString()
        {
            return "Ev3PduTouchSensorMsg: " +
            "\nvalue: " + value.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}