using Hakoniwa.Core.Utils;
using Hakoniwa.PluggableAsset;
using Hakoniwa.PluggableAsset.Assets.Robot;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Assets.Scripts.Hakoniwa.PluggableAsset.Assets.Robot.TB3
{
    class IMUSensor : MonoBehaviour, IIMUSensor
    {
        private GameObject sensor;
        private float deltaTime;
        private Vector3 prev_velocity = Vector3.zero;
        private Rigidbody my_rigidbody;
        private Vector3 prev_angle = Vector3.zero;
        private Vector3 delta_angle = Vector3.zero;

        public void Initialize(object root)
        {
            this.sensor = (GameObject)root;
            this.my_rigidbody = this.sensor.GetComponentInChildren<Rigidbody>();
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
            pdu.Ref("angular_velocity").SetData("x", (double)my_rigidbody.angularVelocity.z);
            pdu.Ref("angular_velocity").SetData("y", (double)-my_rigidbody.angularVelocity.x);
            pdu.Ref("angular_velocity").SetData("z", (double)my_rigidbody.angularVelocity.y);
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
            Vector3 current_velocity = this.sensor.transform.InverseTransformDirection(my_rigidbody.velocity);
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

        public RosTopicMessageConfig [] getRosConfig()
        {
            return null;
        }
    }
}
