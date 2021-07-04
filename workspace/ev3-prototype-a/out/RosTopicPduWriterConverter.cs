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
using RosMessageTypes.Geometry;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.ROS
{
    class RosTopicPduWriterConverter : IPduWriterConverter
    {
        public IPduCommData ConvertToIoData(IPduWriter src)
        {
            RosTopicPduWriter pdu_writer = src as RosTopicPduWriter;
            return new RosTopicPduCommTypedData(pdu_writer);
        }
        

        static private void ConvertToMessage(IPduReadOperation src, MHeader dst)
        {
			dst.seq = src.GetDataUInt32("seq");
            ConvertToMessage(src.Ref("stamp").GetPduReadOps(), dst.stamp);
			dst.frame_id = src.GetDataString("frame_id");
        }
        static private void ConvertToMessage(IPduReadOperation src, MImu dst)
        {
            ConvertToMessage(src.Ref("header").GetPduReadOps(), dst.header);
            ConvertToMessage(src.Ref("orientation").GetPduReadOps(), dst.orientation);
			dst.orientation_covariance = src.GetDataFloat64Array("orientation_covariance");
            ConvertToMessage(src.Ref("angular_velocity").GetPduReadOps(), dst.angular_velocity);
			dst.angular_velocity_covariance = src.GetDataFloat64Array("angular_velocity_covariance");
            ConvertToMessage(src.Ref("linear_acceleration").GetPduReadOps(), dst.linear_acceleration);
			dst.linear_acceleration_covariance = src.GetDataFloat64Array("linear_acceleration_covariance");
        }
        static private void ConvertToMessage(IPduReadOperation src, MLaserScan dst)
        {
            ConvertToMessage(src.Ref("header").GetPduReadOps(), dst.header);
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
        static private void ConvertToMessage(IPduReadOperation src, MQuaternion dst)
        {
			dst.x = src.GetDataFloat64("x");
			dst.y = src.GetDataFloat64("y");
			dst.z = src.GetDataFloat64("z");
			dst.w = src.GetDataFloat64("w");
        }
        static private void ConvertToMessage(IPduReadOperation src, MTime dst)
        {
			dst.secs = src.GetDataUInt32("secs");
			dst.nsecs = src.GetDataUInt32("nsecs");
        }
        static private void ConvertToMessage(IPduReadOperation src, MTwist dst)
        {
            ConvertToMessage(src.Ref("linear").GetPduReadOps(), dst.linear);
            ConvertToMessage(src.Ref("angular").GetPduReadOps(), dst.angular);
        }
        static private void ConvertToMessage(IPduReadOperation src, MVector3 dst)
        {
			dst.x = src.GetDataFloat64("x");
			dst.y = src.GetDataFloat64("y");
			dst.z = src.GetDataFloat64("z");
        }
        
        
        static public Message ConvertToMessage(IPduReadOperation src, string type)
        {

            if (type.Equals("LaserScan"))
            {
            	MLaserScan ros_topic = new MLaserScan();
                ConvertToMessage(src, ros_topic);
                return ros_topic;
            }
            if (type.Equals("Imu"))
            {
            	MImu ros_topic = new MImu();
                ConvertToMessage(src, ros_topic);
                return ros_topic;
            }
            if (type.Equals("Twist"))
            {
            	MTwist ros_topic = new MTwist();
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
