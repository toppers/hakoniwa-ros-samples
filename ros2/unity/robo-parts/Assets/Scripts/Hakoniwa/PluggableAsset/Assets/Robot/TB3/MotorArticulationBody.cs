using Hakoniwa.PluggableAsset.Assets.Robot;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.TB3
{
    public class MotorArticulationBody : MonoBehaviour, IRobotMotor, IRobotMotorSensor
    {
        private float power_const = 500;
        private float rotation_angle_rate = 0.0f;
        private float motor_radius = 3.3f; //3.3cm

        private GameObject obj;
        private float targetVelocity;
        private int force;
        private bool isStop;
        private float deg;
        private ArticulationBody articulation_body;
        private Quaternion current_angle;
        private Quaternion prev_angle;
        private Quaternion diff_angle;
        private float angle_velocity;
        [SerializeField] ArticulationBody wheel;
        [SerializeField] float forceLimit = 10f;
        [SerializeField] float damping = 2f;

        public float GetRadius()
        {
            return motor_radius;
        }
        public void SetTargetVelicty(float targetVelocity)
        {
            float tmp = power_const * targetVelocity;
            //Debug.Log("MOTOR:targetVelocity=" + tmp);
            this.targetVelocity = tmp;
            Drive(wheel);
        }

        private void Drive(ArticulationBody body)
        {
            if (body == null)
                return;

            ArticulationDrive drive = body.xDrive;
            drive.forceLimit = forceLimit;
            drive.damping = damping;
            drive.targetVelocity = targetVelocity;
            //drive.targetVelocity = 300;
            body.xDrive = drive;
        }
        public void Initialize(System.Object root)
        {
            if (this.obj != null)
            {
                Debug.Log("Motor init");
                this.SetTargetVelicty(0.0f);
                this.deg = 0.0f;
                this.current_angle = obj.transform.localRotation;
                this.prev_angle = obj.transform.localRotation;
                this.angle_velocity = 0.0f;
            }
            else
            {
                this.obj = (GameObject)root;
                this.articulation_body = this.obj.GetComponent<ArticulationBody>();
                this.isStop = false;
            }
        }
        public float GetVelocity()
        {
            return (this.motor_radius * this.rotation_angle_rate);
        }

        public float GetCurrentAngleVelocity()
        {
            return this.angle_velocity;
        }

        public void SetForce(int force)
        {
            this.force = force;
        }
        public void SetStop(bool stop)
        {
            if (stop)
            {
                if (this.isStop == false)
                {
                    //this.articulation_body.drag = 10F;
                    //this.articulation_body.angularDrag = 10F;
                    this.isStop = true;
                    Debug.Log("set stop");
                }
            }
            else
            {
                if (isStop == true)
                {
                    //this.articulation_body.drag = 0F;
                    //this.articulation_body.angularDrag = 0.05F;
                    this.isStop = false;
                    Debug.Log("released stop");
                }
            }
        }

        public void ClearDegree()
        {
            this.deg = 0.0f;
        }
        public float GetDegree()
        {
            return this.deg;
        }
        private float Map360To180(float degree)
        {
            if (degree < 180.0f)
            {
                return degree;
            }

            return degree - 360.0f;
        }

        public static float Degree2Rad(float degree)
        {
            return degree * (Mathf.PI / 180.0f);
        }

        public void UpdateSensorValues()
        {
            //float diff;
            //var diff_rot = this.obj.transform.localRotation * Quaternion.Inverse(this.prevRotation);
            //diff = Map360To180(diff_rot.eulerAngles.y);
            //this.prevRotation = this.obj.transform.localRotation;
            //this.deg += diff;

            this.current_angle = obj.transform.localRotation;
            this.diff_angle = current_angle * Quaternion.Inverse(this.prev_angle);

            //this.diff_angle = (this.current_angle - this.prev_angle);
            this.deg += Map360To180(this.diff_angle.eulerAngles.y);

            this.angle_velocity = this.diff_angle.eulerAngles.y / Time.fixedDeltaTime;
            this.prev_angle = this.current_angle;
        }
        public float GetCurrentAngle()
        {
            return -obj.transform.localRotation.eulerAngles.y;
        }

        public float GetDeltaAngle()
        {
            return Map360To180(diff_angle.eulerAngles.y);
        }
        public Vector3 GetDeltaEulerAngle()
        {
            return diff_angle.eulerAngles;
        }

        public void SetTargetVelicty(int targetVelocity)
        {
            throw new System.NotImplementedException();
        }

        public RosTopicMessageConfig[] getRosConfig()
        {
            return null;
        }
    }
}

