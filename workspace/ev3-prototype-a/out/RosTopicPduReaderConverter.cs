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

            if (ros_pdu_reader.GetTypeName().Equals("Actuator"))
            {
                var ros_topic_data = ros_topic.GetTopicData() as MActuator;

                dst.GetWriteOps().SetData("led", ros_topic_data.led);
                dst.GetWriteOps().SetData("motor_power_a", ros_topic_data.motor_power_a);
                dst.GetWriteOps().SetData("motor_power_b", ros_topic_data.motor_power_b);
                dst.GetWriteOps().SetData("motor_power_c", ros_topic_data.motor_power_c);
                dst.GetWriteOps().SetData("motor_stop_a", ros_topic_data.motor_stop_a);
                dst.GetWriteOps().SetData("motor_stop_b", ros_topic_data.motor_stop_b);
                dst.GetWriteOps().SetData("motor_stop_c", ros_topic_data.motor_stop_c);
                dst.GetWriteOps().SetData("motor_reset_angle_a", ros_topic_data.motor_reset_angle_a);
                dst.GetWriteOps().SetData("motor_reset_angle_b", ros_topic_data.motor_reset_angle_b);
                dst.GetWriteOps().SetData("motor_reset_angle_c", ros_topic_data.motor_reset_angle_c);
                dst.GetWriteOps().SetData("gyro_reset", ros_topic_data.gyro_reset);
                return;
            }
            if (ros_pdu_reader.GetTypeName().Equals("Sensor"))
            {
                var ros_topic_data = ros_topic.GetTopicData() as MSensor;

                dst.GetWriteOps().SetData("button", ros_topic_data.button);
                dst.GetWriteOps().SetData("sensor_color0", ros_topic_data.sensor_color0);
                dst.GetWriteOps().SetData("sensor_color1", ros_topic_data.sensor_color1);
                dst.GetWriteOps().SetData("sensor_color2", ros_topic_data.sensor_color2);
                dst.GetWriteOps().SetData("sensor_reflect0", ros_topic_data.sensor_reflect0);
                dst.GetWriteOps().SetData("sensor_reflect1", ros_topic_data.sensor_reflect1);
                dst.GetWriteOps().SetData("sensor_reflect2", ros_topic_data.sensor_reflect2);
                dst.GetWriteOps().SetData("sensor_rgb_r0", ros_topic_data.sensor_rgb_r0);
                dst.GetWriteOps().SetData("sensor_rgb_r1", ros_topic_data.sensor_rgb_r1);
                dst.GetWriteOps().SetData("sensor_rgb_r2", ros_topic_data.sensor_rgb_r2);
                dst.GetWriteOps().SetData("sensor_rgb_g0", ros_topic_data.sensor_rgb_g0);
                dst.GetWriteOps().SetData("sensor_rgb_g1", ros_topic_data.sensor_rgb_g1);
                dst.GetWriteOps().SetData("sensor_rgb_g2", ros_topic_data.sensor_rgb_g2);
                dst.GetWriteOps().SetData("sensor_rgb_b0", ros_topic_data.sensor_rgb_b0);
                dst.GetWriteOps().SetData("sensor_rgb_b1", ros_topic_data.sensor_rgb_b1);
                dst.GetWriteOps().SetData("sensor_rgb_b2", ros_topic_data.sensor_rgb_b2);
                dst.GetWriteOps().SetData("sensor_gyroscope", ros_topic_data.sensor_gyroscope);
                dst.GetWriteOps().SetData("gyro_degree", ros_topic_data.gyro_degree);
                dst.GetWriteOps().SetData("gyro_degree_rate", ros_topic_data.gyro_degree_rate);
                dst.GetWriteOps().SetData("sensor_ultrasonic", ros_topic_data.sensor_ultrasonic);
                dst.GetWriteOps().SetData("touch_sensor0", ros_topic_data.touch_sensor0);
                dst.GetWriteOps().SetData("touch_sensor1", ros_topic_data.touch_sensor1);
                dst.GetWriteOps().SetData("motor_angle_a", ros_topic_data.motor_angle_a);
                dst.GetWriteOps().SetData("motor_angle_b", ros_topic_data.motor_angle_b);
                dst.GetWriteOps().SetData("motor_angle_c", ros_topic_data.motor_angle_c);
                dst.GetWriteOps().SetData("gps_lat", ros_topic_data.gps_lat);
                dst.GetWriteOps().SetData("gps_lon", ros_topic_data.gps_lon);
                dst.GetWriteOps().SetData("test_array", ros_topic_data.test_array);
                return;
            }
            throw new InvalidCastException("Can not find ros message type:" + ros_pdu_reader.GetTypeName());

        }
        static public Message ConvertToMessage(RosTopicPduReader pdu_reader)
        {

            if (pdu_reader.GetTypeName().Equals("Actuator"))
            {
                MActuator tmp_topic = new MActuator();

                tmp_topic.led = pdu_reader.GetReadOps().GetDataInt32("led");
                tmp_topic.motor_power_a = pdu_reader.GetReadOps().GetDataInt32("motor_power_a");
                tmp_topic.motor_power_b = pdu_reader.GetReadOps().GetDataInt32("motor_power_b");
                tmp_topic.motor_power_c = pdu_reader.GetReadOps().GetDataInt32("motor_power_c");
                tmp_topic.motor_stop_a = pdu_reader.GetReadOps().GetDataUInt32("motor_stop_a");
                tmp_topic.motor_stop_b = pdu_reader.GetReadOps().GetDataUInt32("motor_stop_b");
                tmp_topic.motor_stop_c = pdu_reader.GetReadOps().GetDataUInt32("motor_stop_c");
                tmp_topic.motor_reset_angle_a = pdu_reader.GetReadOps().GetDataInt32("motor_reset_angle_a");
                tmp_topic.motor_reset_angle_b = pdu_reader.GetReadOps().GetDataInt32("motor_reset_angle_b");
                tmp_topic.motor_reset_angle_c = pdu_reader.GetReadOps().GetDataInt32("motor_reset_angle_c");
                tmp_topic.gyro_reset = pdu_reader.GetReadOps().GetDataInt32("gyro_reset");
                return tmp_topic;
            }
            if (pdu_reader.GetTypeName().Equals("Sensor"))
            {
                MSensor tmp_topic = new MSensor();

                tmp_topic.button = pdu_reader.GetReadOps().GetDataInt8("button");
                tmp_topic.sensor_color0 = pdu_reader.GetReadOps().GetDataInt32("sensor_color0");
                tmp_topic.sensor_color1 = pdu_reader.GetReadOps().GetDataInt32("sensor_color1");
                tmp_topic.sensor_color2 = pdu_reader.GetReadOps().GetDataInt32("sensor_color2");
                tmp_topic.sensor_reflect0 = pdu_reader.GetReadOps().GetDataInt32("sensor_reflect0");
                tmp_topic.sensor_reflect1 = pdu_reader.GetReadOps().GetDataInt32("sensor_reflect1");
                tmp_topic.sensor_reflect2 = pdu_reader.GetReadOps().GetDataInt32("sensor_reflect2");
                tmp_topic.sensor_rgb_r0 = pdu_reader.GetReadOps().GetDataInt32("sensor_rgb_r0");
                tmp_topic.sensor_rgb_r1 = pdu_reader.GetReadOps().GetDataInt32("sensor_rgb_r1");
                tmp_topic.sensor_rgb_r2 = pdu_reader.GetReadOps().GetDataInt32("sensor_rgb_r2");
                tmp_topic.sensor_rgb_g0 = pdu_reader.GetReadOps().GetDataInt32("sensor_rgb_g0");
                tmp_topic.sensor_rgb_g1 = pdu_reader.GetReadOps().GetDataInt32("sensor_rgb_g1");
                tmp_topic.sensor_rgb_g2 = pdu_reader.GetReadOps().GetDataInt32("sensor_rgb_g2");
                tmp_topic.sensor_rgb_b0 = pdu_reader.GetReadOps().GetDataInt32("sensor_rgb_b0");
                tmp_topic.sensor_rgb_b1 = pdu_reader.GetReadOps().GetDataInt32("sensor_rgb_b1");
                tmp_topic.sensor_rgb_b2 = pdu_reader.GetReadOps().GetDataInt32("sensor_rgb_b2");
                tmp_topic.sensor_gyroscope = pdu_reader.GetReadOps().GetDataInt32("sensor_gyroscope");
                tmp_topic.gyro_degree = pdu_reader.GetReadOps().GetDataInt32("gyro_degree");
                tmp_topic.gyro_degree_rate = pdu_reader.GetReadOps().GetDataInt32("gyro_degree_rate");
                tmp_topic.sensor_ultrasonic = pdu_reader.GetReadOps().GetDataInt32("sensor_ultrasonic");
                tmp_topic.touch_sensor0 = pdu_reader.GetReadOps().GetDataInt32("touch_sensor0");
                tmp_topic.touch_sensor1 = pdu_reader.GetReadOps().GetDataInt32("touch_sensor1");
                tmp_topic.motor_angle_a = pdu_reader.GetReadOps().GetDataInt32("motor_angle_a");
                tmp_topic.motor_angle_b = pdu_reader.GetReadOps().GetDataInt32("motor_angle_b");
                tmp_topic.motor_angle_c = pdu_reader.GetReadOps().GetDataInt32("motor_angle_c");
                tmp_topic.gps_lat = pdu_reader.GetReadOps().GetDataFloat64("gps_lat");
                tmp_topic.gps_lon = pdu_reader.GetReadOps().GetDataFloat64("gps_lon");
                tmp_topic.test_array = pdu_reader.GetReadOps().GetDataInt32Array("test_array");
                return tmp_topic;
            }
            throw new InvalidCastException("Can not find ros message type:" + pdu_reader.GetTypeName());
        }
    }
}
