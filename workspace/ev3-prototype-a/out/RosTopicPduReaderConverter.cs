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
    class RosTopicPduReaderConverter : IPduReaderConverter
    {
        public IPduCommData ConvertToIoData(IPduReader src)
        {
            RosTopicPduReader pdu_reader = src as RosTopicPduReader;
            return new RosTopicPduCommTypedData(pdu_reader);
        }
        

        private void ConvertToPdu(MHeader src, IPduWriteOperation dst)
        {
            dst.SetData("seq", src.seq);
			ConvertToPdu(src.stamp, dst.Ref("stamp").GetPduWriteOps());
            dst.SetData("frame_id", src.frame_id);
        }
        private void ConvertToPdu(MImu src, IPduWriteOperation dst)
        {
			ConvertToPdu(src.header, dst.Ref("header").GetPduWriteOps());
			ConvertToPdu(src.orientation, dst.Ref("orientation").GetPduWriteOps());
            dst.SetData("orientation_covariance", src.orientation_covariance);
			ConvertToPdu(src.angular_velocity, dst.Ref("angular_velocity").GetPduWriteOps());
            dst.SetData("angular_velocity_covariance", src.angular_velocity_covariance);
			ConvertToPdu(src.linear_acceleration, dst.Ref("linear_acceleration").GetPduWriteOps());
            dst.SetData("linear_acceleration_covariance", src.linear_acceleration_covariance);
        }
        private void ConvertToPdu(MLaserScan src, IPduWriteOperation dst)
        {
			ConvertToPdu(src.header, dst.Ref("header").GetPduWriteOps());
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
        private void ConvertToPdu(MQuaternion src, IPduWriteOperation dst)
        {
            dst.SetData("x", src.x);
            dst.SetData("y", src.y);
            dst.SetData("z", src.z);
            dst.SetData("w", src.w);
        }
        private void ConvertToPdu(MTime src, IPduWriteOperation dst)
        {
            dst.SetData("secs", src.secs);
            dst.SetData("nsecs", src.nsecs);
        }
        private void ConvertToPdu(MTwist src, IPduWriteOperation dst)
        {
			ConvertToPdu(src.linear, dst.Ref("linear").GetPduWriteOps());
			ConvertToPdu(src.angular, dst.Ref("angular").GetPduWriteOps());
        }
        private void ConvertToPdu(MVector3 src, IPduWriteOperation dst)
        {
            dst.SetData("x", src.x);
            dst.SetData("y", src.y);
            dst.SetData("z", src.z);
        }

        public void ConvertToPduData(IPduCommData src, IPduReader dst)
        {
            RosTopicPduCommTypedData ros_topic = src as RosTopicPduCommTypedData;

            RosTopicPduReader ros_pdu_reader = dst as RosTopicPduReader;

            if (ros_pdu_reader.GetTypeName().Equals("LaserScan"))
            {
                var ros_topic_data = ros_topic.GetTopicData() as MLaserScan;
                ConvertToPdu(ros_topic_data, dst.GetWriteOps());
                return;
            }
            if (ros_pdu_reader.GetTypeName().Equals("Imu"))
            {
                var ros_topic_data = ros_topic.GetTopicData() as MImu;
                ConvertToPdu(ros_topic_data, dst.GetWriteOps());
                return;
            }
            if (ros_pdu_reader.GetTypeName().Equals("Twist"))
            {
                var ros_topic_data = ros_topic.GetTopicData() as MTwist;
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
