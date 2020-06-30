using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SDK
{
    public class Check
    {
        /****************************************************************************************************************************************************/
        public static UInt16 Lds_Calc_Checksum16(Byte[] Ptr, UInt16 Length)
        {
            UInt16 Checksum = 0, j = 0;
            while (Length > 0)
            {
                Checksum += Ptr[j++];
                Length--;
            }
            return Checksum;
        }
        /****************************************************************************************************************************************************/
        public static UInt16 Lds_Cycle_Calc_Checksum16(Byte[] Ptr, UInt16 Start, UInt16 Length)
        {
            UInt16 Checksum = 0;
            while (Length > 0)
            {
                Checksum += Ptr[Start++];
                Length--;
            }
            return Checksum;
        }
        /****************************************************************************************************************************************************/
    }
}
