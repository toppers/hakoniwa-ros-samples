using Assets.Scripts.Hakoniwa.PluggableAsset.Assets.Robot;
using Assets.Scripts.Hakoniwa.PluggableAsset.Assets.Robot.TB3;
using Hakoniwa.Core.Utils;
using Hakoniwa.PluggableAsset.Assets.Robot;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Collections.Concurrent;
using System.Runtime.InteropServices;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.TB3
{
    public class CameraSensor : MonoBehaviour, ICameraSensor
    {
        private GameObject sensor;
        public RenderTexture RenderTextureRef;
        //public string saveFilePath = "./SavedScreen.jpeg";
        private Texture2D tex;
        private byte[] raw_bytes;
        private byte[] jpg_bytes;
        private string frame_id = "camera_link";

        public void Initialize(object root)
        {
            this.sensor = (GameObject)root;
        }

        public void UpdateSensorValues()
        {
            tex = new Texture2D(RenderTextureRef.width, RenderTextureRef.height, TextureFormat.RGB24, false);
            RenderTexture.active = RenderTextureRef;
            int width = RenderTextureRef.width;
            int height = RenderTextureRef.height;
            int step = width * 3;
            tex.ReadPixels(new Rect(0, 0, width, height), 0, 0);
            tex.Apply();
            // raw_bytes = tex.GetRawTextureData();

            // Raw Image RGB24=(ROS)rgb8
            byte[] _byte = tex.GetRawTextureData();
            raw_bytes = new byte[_byte.Length];
            for (int i = 0; i < height; i++)
            {
              System.Array.Copy(_byte, i*step, raw_bytes, (height-i-1)*step, step);
            }

            // Encode texture into JPG
            jpg_bytes = tex.EncodeToJPG();
            Object.Destroy(tex);
            //File.WriteAllBytes(saveFilePath, bytes);
        }
        public void UpdateSensorData(Pdu pdu)
        {
            if (pdu.GetName() == "sensor_msgs/Image") {
              TimeStamp.Set(pdu);
              pdu.Ref("header").SetData("frame_id", frame_id);
              pdu.SetData("height", (System.UInt32)RenderTextureRef.height);
              pdu.SetData("width", (System.UInt32)RenderTextureRef.width);
              pdu.SetData("encoding", "rgb8");
              pdu.SetData("step", (System.UInt32)RenderTextureRef.width*3);
              pdu.SetData("data", raw_bytes);
            } else if (pdu.GetName() == "sensor_msgs/CompressedImage") {
              TimeStamp.Set(pdu);
              pdu.Ref("header").SetData("frame_id", frame_id);
              pdu.SetData("format", "jpeg");
              pdu.SetData("data", jpg_bytes);
            } else {
              Debug.Log("MSG Type is Not Found: " + pdu.GetName());
            }
        }
        public string [] topic_type = {
            "sensor_msgs/Image",
            "sensor_msgs/CompressedImage",
            "sensor_msgs/CameraInfo"
        };
        public string [] topic_name = {
            "image",
            "image/compressed",
            "camera_info"
        };
        public int [] update_cycle = {
            100,
            100,
            10
        };
        public RosTopicMessageConfig[] getRosConfig()
        {
            RosTopicMessageConfig[] cfg = new RosTopicMessageConfig[topic_type.Length];
            int i = 0;
            for (i = 0; i < topic_type.Length; i++)
            {
                cfg[i] = new RosTopicMessageConfig();
                cfg[i].topic_message_name = this.topic_name[i];
                cfg[i].topic_type_name = this.topic_type[i];
                cfg[i].sub = false;
                cfg[i].pub_option = new RostopicPublisherOption();
                cfg[i].pub_option.cycle_scale = this.update_cycle[i];
                cfg[i].pub_option.latch = false;
                cfg[i].pub_option.queue_size = 1;
            }

            return cfg;
        }
    }
}

