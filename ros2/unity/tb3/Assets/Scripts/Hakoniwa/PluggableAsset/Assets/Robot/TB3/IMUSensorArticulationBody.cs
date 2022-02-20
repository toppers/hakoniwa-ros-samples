using Hakoniwa.PluggableAsset;
using Hakoniwa.PluggableAsset.Assets.Robot;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Assets.Scripts.Hakoniwa.PluggableAsset.Assets.Robot.TB3
{
    class IMUSensorArticulationBody : MonoBehaviour, IIMUSensor
    {
        private GameObject sensor;
        private float deltaTime;
        private Vector3 prev_velocity = Vector3.zero;
        private ArticulationBody mybody;
        private Vector3 prev_angle = Vector3.zero;
        private Vector3 delta_angle = Vector3.zero;

        public void Initialize(object root)
        {
            this.sensor = (GameObject)root;
            this.mybody = this.sensor.GetComponentInChildren<ArticulationBody>();
            this.deltaTime = Time.fixedDeltaTime;
        }

        private void UpdateOrientation(Pdu pdu)
        {
            pdu.Ref("orientation").SetData("x", (double)this.sensor.transform.rotation.z);
            pdu.Ref("orientation").SetData("y", (double)-this.sensor.transform.rotation.x);
            pdu.Ref("orientation").SetData("z", (double)this.sensor.transform.rotation.y);
            pdu.Ref("orientation").SetData("w", (double)-this.sensor.transform.rotation.w);
        }
        private void UpdateAngularVelocity(Pdu pdu)
        {
            pdu.Ref("angular_velocity").SetData("x", (double)mybody.angularVelocity.z);
            pdu.Ref("angular_velocity").SetData("y", (double)-mybody.angularVelocity.x);
            pdu.Ref("angular_velocity").SetData("z", (double)mybody.angularVelocity.y);
        }

        internal Vector3 GetCurrentEulerAngle()
        {
            return this.sensor.transform.rotation.eulerAngles;
        }
        internal Quaternion GetCurrentAngle()
        {
            return this.sensor.transform.rotation;
        }
        internal Quaternion GetCurrentLocalAngle()
        {
            return this.sensor.transform.localRotation;
        }

        internal Vector3 GetDeltaEulerAngle()
        {
            return this.delta_angle;
        }

        private void UpdateLinearAcceleration(Pdu pdu)
        {
            Vector3 current_velocity = this.sensor.transform.InverseTransformDirection(mybody.velocity);
            Vector3 acceleration = (current_velocity - prev_velocity) / deltaTime;
            this.prev_velocity = current_velocity;
            this.delta_angle = this.GetCurrentEulerAngle() - prev_angle;
            this.prev_angle = this.GetCurrentEulerAngle();
            //gravity element
            acceleration += transform.InverseTransformDirection(Physics.gravity);

            pdu.Ref("linear_acceleration").SetData("x", (double)acceleration.z);
            pdu.Ref("linear_acceleration").SetData("y", (double)-acceleration.x);
            pdu.Ref("linear_acceleration").SetData("z", (double)acceleration.y);
        }

        public void UpdateSensorData(Pdu pdu)
        {
            TimeStamp.Set(pdu);
            pdu.Ref("header").SetData("frame_id", "imu_link");

            //orientation
            UpdateOrientation(pdu);

            //angular_velocity
            UpdateAngularVelocity(pdu);

            //linear_acceleration
            UpdateLinearAcceleration(pdu);
        }


        public void UpdateSensorValues()
        {
            //TODO
        }

        IRobotVector3 IIMUSensor.GetDeltaEulerAngle()
        {
            IRobotVector3 angle = new IRobotVector3();
            angle.x = this.delta_angle.x;
            angle.y = this.delta_angle.y;
            angle.z = this.delta_angle.z;
            return angle;
        }

        public IRobotVector3 GetPosition()
        {
            IRobotVector3 pos = new IRobotVector3();
            pos.x = this.transform.position.x;
            pos.y = this.transform.position.y;
            pos.x = this.transform.position.z;
            return pos;
        }

        public string topic_type = "sensor_msgs/Imu";
        public string topic_name = "imu";
        public int update_cycle = 10;
        public RosTopicMessageConfig[] getRosConfig()
        {
            RosTopicMessageConfig[] cfg = new RosTopicMessageConfig[1];
            cfg[0] = new RosTopicMessageConfig();
            cfg[0].topic_message_name = this.topic_name;
            cfg[0].topic_type_name = this.topic_type;
            cfg[0].sub = false;
            cfg[0].pub_option = new RostopicPublisherOption();
            cfg[0].pub_option.cycle_scale = this.update_cycle;
            cfg[0].pub_option.latch = false;
            cfg[0].pub_option.queue_size = 1;
            return cfg;
        }
    }

}
