using System;
using System.Collections.Generic;
using UnityEngine;

using Hakoniwa.PluggableAsset.Communication.Connector;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using Hakoniwa.Core.Utils;
using Hakoniwa.PluggableAsset.Communication.Pdu.Accessor;
using Hakoniwa.PluggableAsset.Assets.Robot.Parts;

namespace Hakoniwa.PluggableAsset.Assets.Robot.TB3
{
    public class UpdateDeviceCycle
    {
        public int count;
        public int cycle;
        public UpdateDeviceCycle(int c)
        {
            this.count = 0;
            this.cycle = c;
        }
    }
    public class RobotController : MonoBehaviour, IInsideAssetController
    {
        private GameObject root;
        private GameObject myObject;
        private ITB3Parts parts;
        private string my_name;
        private PduIoConnector pdu_io;
        private IPduReader pdu_motor_control;
        private MotorController motor_controller;
        private long current_timestamp;
        private ParamScale scale;
        private Dictionary<string, UpdateDeviceCycle> device_update_cycle = new Dictionary<string, UpdateDeviceCycle>();
        private IPduWriter pdu_joint_state;

        private IRobotPartsSensor[] sensors = null;

        public void CopySensingDataToPdu()
        {
            foreach (var child in this.sensors)
            {
                if (child.isAttachedSpecificController())
                {
                    continue;
                }
                child.UpdateSensorValues();
            }
            //TODO


            this.current_timestamp = UtilTime.GetUnixTime();
            //joint states
            this.PublishJointStates();

            //Motor
            this.motor_controller.CopySensingDataToPdu();
        }


        public void DoActuation()
        {
            this.motor_controller.DoActuation();
            //Physics.SyncTransforms();
        }

        public string GetName()
        {
            return this.my_name;
        }

        public void Initialize()
        {
            Debug.Log("TurtleBot3");
            this.scale = AssetConfigLoader.GetScale();

            this.root = GameObject.Find("Robot");
            this.myObject = GameObject.Find("Robot/" + this.transform.name);
            this.parts = myObject.GetComponentInChildren<ITB3Parts>();
            this.parts.Load();
            this.my_name = string.Copy(this.transform.name);
            this.pdu_io = PduIoConnector.Get(this.GetName());
            this.InitActuator();
            this.InitSensor();

        }

        private void InitSensor()
        {
            /*
             * ルートは特定のコントローラに割り当てられていないセンサを初期化する
             * 
             * 割り当てされているものは，当該コントローラ側の責務で初期化する
             */
            this.sensors = this.myObject.GetComponentsInChildren<IRobotPartsSensor>();
            foreach (var child in this.sensors)
            {
                if (child.isAttachedSpecificController())
                {
                    continue;
                }
                child.Initialize(this.myObject);
            }

            //TODO
            this.pdu_joint_state = this.pdu_io.GetWriter(this.GetName() + "_joint_statesPdu");
            if (this.pdu_joint_state == null)
            {
                throw new ArgumentException("can not found joint_states pdu:" + this.GetName() + "_joint_statesPdu");
            }
            this.pdu_joint_state.GetWriteOps().Ref("header").SetData("frame_id", "");
            string[] joint_names = new string[2];
            joint_names[0] = "wheel_left_joint";
            joint_names[1] = "wheel_right_joint";
            this.pdu_joint_state.GetWriteOps().SetData("name", joint_names);

        }

        private void InitActuator()
        {
            motor_controller = new MotorController();
            this.pdu_motor_control = this.pdu_io.GetReader(this.GetName() + "_cmd_velPdu");
            if (this.pdu_motor_control == null)
            {
                throw new ArgumentException("can not found CmdVel pdu:" + this.GetName() + "_cmd_velPdu");
            }
            motor_controller.Initialize(this.root, this.transform, this.parts, this.pdu_motor_control);
        }


        public string[] topic_type = {
            "sensor_msgs/JointState"
        };
        public string[] topic_name = {
            "joint_states"
        };
        public int[] update_cycle = {
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
        private void PublishJointStates()
        {
            //ROS 0: left,  1: right
            TimeStamp.Set(this.current_timestamp, this.pdu_joint_state.GetWriteOps().Ref(null));
            //position
            double[] position = new double[2];
            position[0] = this.motor_controller.GetLeftMotor().GetCurrentAngle() * Mathf.Deg2Rad;
            position[1] = this.motor_controller.GetRightMotor().GetCurrentAngle() * Mathf.Deg2Rad;
            //position[0] = this.motor_controller.GetLeftMotor().GetDegree() * Mathf.Deg2Rad;
            //position[1] = this.motor_controller.GetRightMotor().GetDegree() * Mathf.Deg2Rad;


            //velocity
            double[] velocity = new double[2];
            velocity[0] = this.motor_controller.GetLeftMotor().GetCurrentAngleVelocity() * Mathf.Deg2Rad;
            velocity[1] = this.motor_controller.GetRightMotor().GetCurrentAngleVelocity() * Mathf.Deg2Rad;

            //effort
            double[] effort = new double[2];
            effort[0] = 0.0f;
            effort[1] = 0.0f;

            //Set PDU
            this.pdu_joint_state.GetWriteOps().SetData("position", position);
            this.pdu_joint_state.GetWriteOps().SetData("velocity", velocity);
            this.pdu_joint_state.GetWriteOps().SetData("effort", effort);
        }
    }
}