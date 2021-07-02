using Hakoniwa.PluggableAsset.Communication.Pdu;
using Hakoniwa.PluggableAsset.Communication.Pdu.ROS;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RosMessageTypes.Std;
using RosMessageTypes.Hackev;
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
        

        private void ConvertToPdu(MActuator src, IPduWriteOperation dst)
        {
            dst.SetData("led", src.led);
            dst.SetData("motor_power_a", src.motor_power_a);
            dst.SetData("motor_power_b", src.motor_power_b);
            dst.SetData("motor_power_c", src.motor_power_c);
            dst.SetData("motor_stop_a", src.motor_stop_a);
            dst.SetData("motor_stop_b", src.motor_stop_b);
            dst.SetData("motor_stop_c", src.motor_stop_c);
            dst.SetData("motor_reset_angle_a", src.motor_reset_angle_a);
            dst.SetData("motor_reset_angle_b", src.motor_reset_angle_b);
            dst.SetData("motor_reset_angle_c", src.motor_reset_angle_c);
            dst.SetData("gyro_reset", src.gyro_reset);
        }
        private void ConvertToPdu(MHeader src, IPduWriteOperation dst)
        {
            dst.SetData("seq", src.seq);
			ConvertToPdu(src, dst.Ref("stamp").GetPduWriteOps());
            dst.SetData("frame_id", src.frame_id);
        }
        private void ConvertToPdu(MLaserScan src, IPduWriteOperation dst)
        {
			ConvertToPdu(src, dst.Ref("header").GetPduWriteOps());
            foreach (var e in dst.Refs("array"))
            {
				ConvertToPdu(src.array[Array.IndexOf(dst.Refs("array"), e)], e.GetPduWriteOps());
            }
            dst.SetData("angle_min", src.angle_min);
            dst.SetData("angle_max", src.angle_max);
            dst.SetData("angle_increment", src.angle_increment);
            dst.SetData("time_increment", src.time_increment);
            dst.SetData("scan_time", src.scan_time);
            dst.SetData("range_min", src.range_min);
            dst.SetData("range_max", src.range_max);
            dst.SetData("ranges", src.ranges);
            dst.SetData("intensities", src.intensities);
        }
        private void ConvertToPdu(MSensor src, IPduWriteOperation dst)
        {
            dst.SetData("button", src.button);
            dst.SetData("sensor_color0", src.sensor_color0);
            dst.SetData("sensor_color1", src.sensor_color1);
            dst.SetData("sensor_color2", src.sensor_color2);
            dst.SetData("sensor_reflect0", src.sensor_reflect0);
            dst.SetData("sensor_reflect1", src.sensor_reflect1);
            dst.SetData("sensor_reflect2", src.sensor_reflect2);
            dst.SetData("sensor_rgb_r0", src.sensor_rgb_r0);
            dst.SetData("sensor_rgb_r1", src.sensor_rgb_r1);
            dst.SetData("sensor_rgb_r2", src.sensor_rgb_r2);
            dst.SetData("sensor_rgb_g0", src.sensor_rgb_g0);
            dst.SetData("sensor_rgb_g1", src.sensor_rgb_g1);
            dst.SetData("sensor_rgb_g2", src.sensor_rgb_g2);
            dst.SetData("sensor_rgb_b0", src.sensor_rgb_b0);
            dst.SetData("sensor_rgb_b1", src.sensor_rgb_b1);
            dst.SetData("sensor_rgb_b2", src.sensor_rgb_b2);
            dst.SetData("sensor_gyroscope", src.sensor_gyroscope);
            dst.SetData("gyro_degree", src.gyro_degree);
            dst.SetData("gyro_degree_rate", src.gyro_degree_rate);
            dst.SetData("sensor_ultrasonic", src.sensor_ultrasonic);
            dst.SetData("touch_sensor0", src.touch_sensor0);
            dst.SetData("touch_sensor1", src.touch_sensor1);
            dst.SetData("motor_angle_a", src.motor_angle_a);
            dst.SetData("motor_angle_b", src.motor_angle_b);
            dst.SetData("motor_angle_c", src.motor_angle_c);
            dst.SetData("gps_lat", src.gps_lat);
            dst.SetData("gps_lon", src.gps_lon);
            dst.SetData("test_array", src.test_array);
        }
        private void ConvertToPdu(MTime src, IPduWriteOperation dst)
        {
            dst.SetData("secs", src.secs);
            dst.SetData("nsecs", src.nsecs);
        }

        public void ConvertToPduData(IPduCommData src, IPduReader dst)
        {
            RosTopicPduCommTypedData ros_topic = src as RosTopicPduCommTypedData;

            RosTopicPduReader ros_pdu_reader = dst as RosTopicPduReader;

            if (ros_pdu_reader.GetTypeName().Equals("Sensor"))
            {
                var ros_topic_data = ros_topic.GetTopicData() as MSensor;
                ConvertToPdu(ros_topic_data, dst.GetWriteOps());
                return;
            }
            if (ros_pdu_reader.GetTypeName().Equals("Actuator"))
            {
                var ros_topic_data = ros_topic.GetTopicData() as MActuator;
                ConvertToPdu(ros_topic_data, dst.GetWriteOps());
                return;
            }
            throw new InvalidCastException("Can not find ros message type:" + ros_pdu_reader.GetTypeName());

        }
        static public Message ConvertToMessage(RosTopicPduReader pdu_reader)
        {
            return RosTopicPduWriterConverter.ConvertToMessage(pdu_reader.GetReadOps(), pdu_reader.GetTypeName());
        }
    }
}
