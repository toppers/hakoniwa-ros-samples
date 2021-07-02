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
    class RosTopicPduWriterConverter : IPduWriterConverter
    {
        public IPduCommData ConvertToIoData(IPduWriter src)
        {
            RosTopicPduWriter pdu_writer = src as RosTopicPduWriter;
            return new RosTopicPduCommTypedData(pdu_writer);
        }
        

        static private void ConvertToMessage(IPduReadOperation src, MActuator dst)
        {
			dst.led = src.GetDataInt32("led");
			dst.motor_power_a = src.GetDataInt32("motor_power_a");
			dst.motor_power_b = src.GetDataInt32("motor_power_b");
			dst.motor_power_c = src.GetDataInt32("motor_power_c");
			dst.motor_stop_a = src.GetDataUInt32("motor_stop_a");
			dst.motor_stop_b = src.GetDataUInt32("motor_stop_b");
			dst.motor_stop_c = src.GetDataUInt32("motor_stop_c");
			dst.motor_reset_angle_a = src.GetDataInt32("motor_reset_angle_a");
			dst.motor_reset_angle_b = src.GetDataInt32("motor_reset_angle_b");
			dst.motor_reset_angle_c = src.GetDataInt32("motor_reset_angle_c");
			dst.gyro_reset = src.GetDataInt32("gyro_reset");
        }
        static private void ConvertToMessage(IPduReadOperation src, MHeader dst)
        {
			dst.seq = src.GetDataUInt32("seq");
            ConvertToMessage(src.Ref("stamp").GetPduReadOps(), dst.stamp);
			dst.frame_id = src.GetDataString("frame_id");
        }
        static private void ConvertToMessage(IPduReadOperation src, MLaserScan dst)
        {
            ConvertToMessage(src.Ref("header").GetPduReadOps(), dst.header);
            foreach (var e in src.Refs("array"))
            {
                ConvertToMessage(e.GetPduReadOps(), dst.array[Array.IndexOf(src.Refs("array"), e)]);
            }
			dst.angle_min = src.GetDataFloat32("angle_min");
			dst.angle_max = src.GetDataFloat32("angle_max");
			dst.angle_increment = src.GetDataFloat32("angle_increment");
			dst.time_increment = src.GetDataFloat32("time_increment");
			dst.scan_time = src.GetDataFloat32("scan_time");
			dst.range_min = src.GetDataFloat32("range_min");
			dst.range_max = src.GetDataFloat32("range_max");
			dst.ranges = src.GetDataFloat32Array("ranges");
			dst.intensities = src.GetDataFloat32Array("intensities");
        }
        static private void ConvertToMessage(IPduReadOperation src, MSensor dst)
        {
			dst.button = src.GetDataInt8("button");
			dst.sensor_color0 = src.GetDataInt32("sensor_color0");
			dst.sensor_color1 = src.GetDataInt32("sensor_color1");
			dst.sensor_color2 = src.GetDataInt32("sensor_color2");
			dst.sensor_reflect0 = src.GetDataInt32("sensor_reflect0");
			dst.sensor_reflect1 = src.GetDataInt32("sensor_reflect1");
			dst.sensor_reflect2 = src.GetDataInt32("sensor_reflect2");
			dst.sensor_rgb_r0 = src.GetDataInt32("sensor_rgb_r0");
			dst.sensor_rgb_r1 = src.GetDataInt32("sensor_rgb_r1");
			dst.sensor_rgb_r2 = src.GetDataInt32("sensor_rgb_r2");
			dst.sensor_rgb_g0 = src.GetDataInt32("sensor_rgb_g0");
			dst.sensor_rgb_g1 = src.GetDataInt32("sensor_rgb_g1");
			dst.sensor_rgb_g2 = src.GetDataInt32("sensor_rgb_g2");
			dst.sensor_rgb_b0 = src.GetDataInt32("sensor_rgb_b0");
			dst.sensor_rgb_b1 = src.GetDataInt32("sensor_rgb_b1");
			dst.sensor_rgb_b2 = src.GetDataInt32("sensor_rgb_b2");
			dst.sensor_gyroscope = src.GetDataInt32("sensor_gyroscope");
			dst.gyro_degree = src.GetDataInt32("gyro_degree");
			dst.gyro_degree_rate = src.GetDataInt32("gyro_degree_rate");
			dst.sensor_ultrasonic = src.GetDataInt32("sensor_ultrasonic");
			dst.touch_sensor0 = src.GetDataInt32("touch_sensor0");
			dst.touch_sensor1 = src.GetDataInt32("touch_sensor1");
			dst.motor_angle_a = src.GetDataInt32("motor_angle_a");
			dst.motor_angle_b = src.GetDataInt32("motor_angle_b");
			dst.motor_angle_c = src.GetDataInt32("motor_angle_c");
			dst.gps_lat = src.GetDataFloat64("gps_lat");
			dst.gps_lon = src.GetDataFloat64("gps_lon");
			dst.test_array = src.GetDataInt32Array("test_array");
        }
        static private void ConvertToMessage(IPduReadOperation src, MTime dst)
        {
			dst.secs = src.GetDataUInt32("secs");
			dst.nsecs = src.GetDataUInt32("nsecs");
        }
        
        
        static public Message ConvertToMessage(IPduReadOperation src, string type)
        {

            if (type.Equals("Sensor"))
            {
            	MSensor ros_topic = new MSensor();
                ConvertToMessage(src, ros_topic);
                return ros_topic;
            }
            if (type.Equals("Actuator"))
            {
            	MActuator ros_topic = new MActuator();
                ConvertToMessage(src, ros_topic);
                return ros_topic;
            }
            throw new InvalidCastException("Can not find ros message type:" + type);
        }
        
        static public Message ConvertToMessage(RosTopicPduWriter pdu_writer)
        {
            return ConvertToMessage(pdu_writer.GetReadOps(), pdu_writer.GetTypeName());
        }
    }

}
