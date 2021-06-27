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
    class RosTopicPduWriterConverter : IPduWriterConverter
    {
        public IPduCommData ConvertToIoData(IPduWriter src)
        {
            RosTopicPduWriter pdu_writer = src as RosTopicPduWriter;
            return new RosTopicPduCommTypedData(pdu_writer);
        }
        static public Message ConvertToMessage(RosTopicPduWriter pdu_writer)
        {

            if (pdu_writer.GetTypeName().Equals("Actuator"))
            {
                MActuator tmp_topic = new MActuator();

                tmp_topic.led = pdu_writer.GetReadOps().GetDataInt32("led");
                tmp_topic.motor_power_a = pdu_writer.GetReadOps().GetDataInt32("motor_power_a");
                tmp_topic.motor_power_b = pdu_writer.GetReadOps().GetDataInt32("motor_power_b");
                tmp_topic.motor_power_c = pdu_writer.GetReadOps().GetDataInt32("motor_power_c");
                tmp_topic.motor_stop_a = pdu_writer.GetReadOps().GetDataUInt32("motor_stop_a");
                tmp_topic.motor_stop_b = pdu_writer.GetReadOps().GetDataUInt32("motor_stop_b");
                tmp_topic.motor_stop_c = pdu_writer.GetReadOps().GetDataUInt32("motor_stop_c");
                tmp_topic.motor_reset_angle_a = pdu_writer.GetReadOps().GetDataInt32("motor_reset_angle_a");
                tmp_topic.motor_reset_angle_b = pdu_writer.GetReadOps().GetDataInt32("motor_reset_angle_b");
                tmp_topic.motor_reset_angle_c = pdu_writer.GetReadOps().GetDataInt32("motor_reset_angle_c");
                tmp_topic.gyro_reset = pdu_writer.GetReadOps().GetDataInt32("gyro_reset");

                return tmp_topic;
            }
            if (pdu_writer.GetTypeName().Equals("Sensor"))
            {
                MSensor tmp_topic = new MSensor();

                tmp_topic.button = pdu_writer.GetReadOps().GetDataInt8("button");
                tmp_topic.sensor_color0 = pdu_writer.GetReadOps().GetDataUInt32("sensor_color0");
                tmp_topic.sensor_color1 = pdu_writer.GetReadOps().GetDataUInt32("sensor_color1");
                tmp_topic.sensor_color2 = pdu_writer.GetReadOps().GetDataUInt32("sensor_color2");
                tmp_topic.sensor_reflect0 = pdu_writer.GetReadOps().GetDataUInt32("sensor_reflect0");
                tmp_topic.sensor_reflect1 = pdu_writer.GetReadOps().GetDataUInt32("sensor_reflect1");
                tmp_topic.sensor_reflect2 = pdu_writer.GetReadOps().GetDataUInt32("sensor_reflect2");
                tmp_topic.sensor_rgb_r0 = pdu_writer.GetReadOps().GetDataUInt32("sensor_rgb_r0");
                tmp_topic.sensor_rgb_r1 = pdu_writer.GetReadOps().GetDataUInt32("sensor_rgb_r1");
                tmp_topic.sensor_rgb_r2 = pdu_writer.GetReadOps().GetDataUInt32("sensor_rgb_r2");
                tmp_topic.sensor_rgb_g0 = pdu_writer.GetReadOps().GetDataUInt32("sensor_rgb_g0");
                tmp_topic.sensor_rgb_g1 = pdu_writer.GetReadOps().GetDataUInt32("sensor_rgb_g1");
                tmp_topic.sensor_rgb_g2 = pdu_writer.GetReadOps().GetDataUInt32("sensor_rgb_g2");
                tmp_topic.sensor_rgb_b0 = pdu_writer.GetReadOps().GetDataUInt32("sensor_rgb_b0");
                tmp_topic.sensor_rgb_b1 = pdu_writer.GetReadOps().GetDataUInt32("sensor_rgb_b1");
                tmp_topic.sensor_rgb_b2 = pdu_writer.GetReadOps().GetDataUInt32("sensor_rgb_b2");
                tmp_topic.sensor_gyroscope = pdu_writer.GetReadOps().GetDataInt32("sensor_gyroscope");
                tmp_topic.gyro_degree = pdu_writer.GetReadOps().GetDataInt32("gyro_degree");
                tmp_topic.gyro_degree_rate = pdu_writer.GetReadOps().GetDataInt32("gyro_degree_rate");
                tmp_topic.sensor_ultrasonic = pdu_writer.GetReadOps().GetDataUInt32("sensor_ultrasonic");
                tmp_topic.touch_sensor0 = pdu_writer.GetReadOps().GetDataUInt32("touch_sensor0");
                tmp_topic.touch_sensor1 = pdu_writer.GetReadOps().GetDataUInt32("touch_sensor1");
                tmp_topic.motor_angle_a = pdu_writer.GetReadOps().GetDataUInt32("motor_angle_a");
                tmp_topic.motor_angle_b = pdu_writer.GetReadOps().GetDataUInt32("motor_angle_b");
                tmp_topic.motor_angle_c = pdu_writer.GetReadOps().GetDataUInt32("motor_angle_c");
                tmp_topic.gps_lat = pdu_writer.GetReadOps().GetDataFloat64("gps_lat");
                tmp_topic.gps_lon = pdu_writer.GetReadOps().GetDataFloat64("gps_lon");

                return tmp_topic;
            }
            
            throw new InvalidCastException("Can not find ros message type:" + pdu_writer.GetTypeName());

        }
    }

}
