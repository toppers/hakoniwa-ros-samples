using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    public class BoolAccessor
    {
        private Pdu pdu;
        
        public BoolAccessor(Pdu pdu)
        {
        	this.pdu = pdu;
        }
        public bool data
        {
            set
            {
                pdu.SetData("data", value);
            }
            get
            {
                return pdu.GetDataBool("data");
            }
        }
    }
}
