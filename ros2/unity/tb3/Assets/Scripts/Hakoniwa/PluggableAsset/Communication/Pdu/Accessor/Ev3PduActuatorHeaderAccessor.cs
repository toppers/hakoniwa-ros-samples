using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    class Ev3PduActuatorHeaderAccessor
    {
        private Pdu pdu;
        
        public Ev3PduActuatorHeaderAccessor(Pdu pdu)
        {
        	this.pdu = pdu;
        }
        public string name
        {
            set
            {
                pdu.SetData("name", value);
            }
            get
            {
                return pdu.GetDataString("name");
            }
        }
        public UInt32 version
        {
            set
            {
                pdu.SetData("version", value);
            }
            get
            {
                return pdu.GetDataUInt32("version");
            }
        }
        public Int64 asset_time
        {
            set
            {
                pdu.SetData("asset_time", value);
            }
            get
            {
                return pdu.GetDataInt64("asset_time");
            }
        }
        public UInt32 ext_off
        {
            set
            {
                pdu.SetData("ext_off", value);
            }
            get
            {
                return pdu.GetDataUInt32("ext_off");
            }
        }
        public UInt32 ext_size
        {
            set
            {
                pdu.SetData("ext_size", value);
            }
            get
            {
                return pdu.GetDataUInt32("ext_size");
            }
        }
    }
}
