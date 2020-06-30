using System;
using System.IO.Ports;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Runtime.InteropServices;
/****************************************************************************************************************************************************/
public enum CMD_ID
{
    Cmd_Meas_Report = 0xAD,
    Cmd_Error_Report = 0xAE,
};

/****************************************************************************************************************************************************/
public struct R_BUFFER_T
{
    public UInt64 WrCnt;
    public UInt64 RdCnt;
    public Byte[] Data;
};
/****************************************************************************************************************************************************/
public struct T_BUFFER_T
{
    public UInt64 WrCnt;
    //public CMD_ID CmdId;
	///*
    public byte Repeat;
    public UInt16 ReTimes;
	//*/
    public Byte[] Data;
};
/****************************************************************************************************************************************************/
public struct PROTOCOL_T
{
    public byte FrameHeader;
    public UInt16 FrameLen;
    public byte Addr;
    public byte FrameType;
    public byte Cmd;
    public UInt16 ParamLen;
    public Byte[] Data;
};

/****************************************************************************************************************************************************/
public struct POINT_T
{
    public double Angle;
    public UInt16 Sig;
    public double Dist;
};
/****************************************************************************************************************************************************/
public struct POINT_LIST
{
    public List<POINT_T> Points;
}
/****************************************************************************************************************************************************/
public struct ENC_BUFF
{
    public POINT_LIST[] ScanInfo;
    public UInt16 RdyFlag;
};
public struct ONE_CIRCLE_POINT
{
    public POINT_LIST ScanInfo;
    public UInt16 RdyFlag;
};
/****************************************************************************************************************************************************/
namespace SDK
{
    public class Lidar
    {      
        public R_BUFFER_T RxBuff;
        public T_BUFFER_T TxBuff;      
        public ENC_BUFF EncBuff;
        public ONE_CIRCLE_POINT OneCirclePoint;
        public double Speed;   
        private UInt16 PrevAngle = 0;
        public SerialPort SerialDevice;
       // private Thread RxThread;
        
        /****************************************************************************************************************************************************/
        public Lidar()
        {        
            RxBuff.Data = new byte[65536];
            TxBuff.Data = new byte[2000];
            EncBuff.ScanInfo = new POINT_LIST[16];
            for (Byte i = 0; i < EncBuff.ScanInfo.Length; i++)
            {
                EncBuff.ScanInfo[i].Points = new List<POINT_T>();
            }

            OneCirclePoint.ScanInfo.Points = new List<POINT_T>();

            SerialDevice = new SerialPort();
            SerialDevice.DataReceived += new SerialDataReceivedEventHandler(SerialDataReceived);

			/*
            RxThread = new Thread(new ThreadStart(RxThreadFunct));
            RxThread.IsBackground = true;
            RxThread.Start();
			*/
        }
        public bool SerialOpen(string PortName,int BaudRate)
        {
            SerialDevice.PortName = PortName;
            SerialDevice.BaudRate = BaudRate;
            SerialDevice.Parity = Parity.None;
            SerialDevice.DataBits = 8;
            SerialDevice.StopBits = StopBits.One;
            try
            {
                SerialDevice.Open();
                return true;
            }
            catch (Exception ex)
            {
                return false;
            }
        }
        public bool SerialClose()
        {
            try
            {
                SerialDevice.Close();
                return true;
            }
			catch(Exception ex)
            {
                return false;
            }


        }
        private void SerialDataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            ulong I_Index = (UInt16)RxBuff.WrCnt;
            try
            {
                int Len = SerialDevice.BytesToRead;
                if (Len < 1)
                    return;
                Byte[] Data = new Byte[Len];
                SerialDevice.Read(Data, 0, Len);

                for (UInt16 i = 0; i < Data.Length; i++)
                {
                    RxBuff.Data[(UInt16)RxBuff.WrCnt] = Data[i];
                    RxBuff.WrCnt++;
                }
            }
			catch(Exception ex)
            {
				RxBuff.RdCnt = RxBuff.WrCnt = 0;
            }
        }
        /****************************************************************************************************************************************************/
        private void SerialSendCommand()
        {
            try
            {
                SerialDevice.Write(TxBuff.Data, 0, (int)TxBuff.WrCnt);
            }
            catch (Exception ex)
            {
                Console.WriteLine("Uart send data error\n");
                return;
            }
        }
      

        /****************************************************************************************************************************************************/
        public void ClearMeasBuffer()
        {
            RxBuff.WrCnt = RxBuff.RdCnt = 0;
        }

        /****************************************************************************************************************************************************/
        private void MeasDataParse()
        {
            UInt32 Temp, i;
            POINT_T Pt;          
            UInt16 Len, OrigAng,EncIndex,ZeroOffset;
            Double AngStep;
			
           // if (WorkMode != WORK_MODE.STD_SCAN_MODE)
            //    return;

            //Len 为实际点数
            Len = RxBuff.Data[(UInt16)(RxBuff.RdCnt + 6)];
            Len = (UInt16)((Len << 8) | RxBuff.Data[(UInt16)(RxBuff.RdCnt + 7)]);
            Len = (UInt16)((Len - 5) / 3);

            Speed = (RxBuff.Data[(UInt16)(RxBuff.RdCnt + 8)]);
            Speed *= 0.05;

            ZeroOffset = RxBuff.Data[(UInt16)(RxBuff.RdCnt + 9)];
            ZeroOffset = (UInt16)((ZeroOffset << 8) | RxBuff.Data[(UInt16)(RxBuff.RdCnt + 10)]);

            OrigAng = RxBuff.Data[(UInt16)(RxBuff.RdCnt + 11)];
            OrigAng = (UInt16)((OrigAng << 8) | RxBuff.Data[(UInt16)(RxBuff.RdCnt + 12)]);
		
            if(PrevAngle> OrigAng&& PrevAngle!=33750&& OrigAng!=0)
                Console.WriteLine("Lidar Lost Packet error!\n");

            PrevAngle = OrigAng;         
            AngStep = (UInt16)((2250.0) / Len); 
            EncIndex = (UInt16)(OrigAng / 2250);

            EncBuff.ScanInfo[EncIndex].Points.Clear();    
            for (i = 0; i < Len; i++)
            {
                Pt.Angle = OrigAng + AngStep * i;
                if (Pt.Angle < 0)
                    Pt.Angle += 36000;
                Pt.Angle = Pt.Angle % 36000;
                Pt.Angle /= 100.0;
                Pt.Sig = RxBuff.Data[(UInt16)(RxBuff.RdCnt + (UInt16)(3*i)+13)];

                Temp = RxBuff.Data[(UInt16)(RxBuff.RdCnt + (UInt16)(i * 3) + 14)];
                Temp = (UInt16)((Temp << 8) | RxBuff.Data[(UInt16)(RxBuff.RdCnt + (UInt16)(i * 3) + 15)]);
                Pt.Dist = Temp * 0.25;
                EncBuff.ScanInfo[EncIndex].Points.Add(Pt);
           
            }      
            EncBuff.RdyFlag |= (UInt16)(1 << EncIndex);
            if (EncBuff.RdyFlag < 0xFFFF)
                return;

            OneCirclePoint.ScanInfo.Points.Clear();
            for (i = 0; i < EncBuff.ScanInfo.Length; i++)
            {
                OneCirclePoint.ScanInfo.Points.AddRange(EncBuff.ScanInfo[i].Points);
            }
            OneCirclePoint.ScanInfo.Points.Sort((a, b) => a.Angle.CompareTo(b.Angle));
            OneCirclePoint.RdyFlag = 1;
            EncBuff.RdyFlag = 0;
        }

        /****************************************************************************************************************************************************/
        private void ErrorReportParse()
        {
            UInt16 i;
            double Speed;

            Speed = RxBuff.Data[(UInt16)(RxBuff.RdCnt + 8)];
            Speed *= 0.05;

            Console.WriteLine("Lidar Speed error: {0} r/s\n", Speed.ToString("#0.00"));


        }
     
        /****************************************************************************************************************************************************/
        private void RxFrameParse(UInt16 DataLen)
        {
            Byte Cmd = RxBuff.Data[(UInt16)(RxBuff.RdCnt + 5)];
            
            switch ((CMD_ID)Cmd)
            {                                      
                case CMD_ID.Cmd_Meas_Report:
					if(DataLen>13)
                    {
                        MeasDataParse();
                    }						 
                    break;
                case CMD_ID.Cmd_Error_Report:
                    ErrorReportParse();
                    break;   
					
                default: break;
            }
            RxBuff.RdCnt = RxBuff.RdCnt + DataLen + 2;
        }
        /****************************************************************************************************************************************************/
        public void RxDataProcess()
        {
            UInt16 Checksum, Temp, DataLen = 0;

            /*
            if (WorkMode != WORK_MODE.STD_SCAN_MODE)
            {
                RxBuff.RdCnt = RxBuff.WrCnt = 0;
                return;
            }
            */

            while (true)
            {

                if (RxBuff.WrCnt < RxBuff.RdCnt)
                {
                    RxBuff.WrCnt = RxBuff.RdCnt = 0;
                    return;
                }
                if (RxBuff.WrCnt < (RxBuff.RdCnt + 8))
                    return;

                if (RxBuff.Data[(UInt16)RxBuff.RdCnt] != 0xAA)
                    RxBuff.RdCnt++;
                else
                {
                    DataLen = RxBuff.Data[(UInt16)(RxBuff.RdCnt + 1)];
                    DataLen = (UInt16)((DataLen << 8) | RxBuff.Data[(UInt16)(RxBuff.RdCnt + 2)]);

                    if (DataLen > 300)
                    {
                        RxBuff.RdCnt++;
                        continue;
                    }

                    if ((RxBuff.WrCnt - RxBuff.RdCnt - 2) < DataLen)
                        return;
                    else
                    {
                        Temp = (UInt16)((RxBuff.Data[(UInt16)(RxBuff.RdCnt + DataLen)] << 8) | RxBuff.Data[(UInt16)(RxBuff.RdCnt + DataLen + 1)]);
                        Checksum = Check.Lds_Cycle_Calc_Checksum16(RxBuff.Data, (UInt16)RxBuff.RdCnt, DataLen);
                        if (Checksum != Temp)
                        {
                            Console.WriteLine("Lidar Check error\n");
                            RxBuff.RdCnt++;
                        }
                        else
                            RxFrameParse(DataLen);
                    }
                }
            }
        }

        public void PrintOneCirclePointInformation()
        {
            //List<POINT_T> Data = new List<POINT_T>;
            if (OneCirclePoint.RdyFlag != 1)
                return;

            //print one circle points of  number
            Console.WriteLine("One circle points of number:{0}\n", OneCirclePoint.ScanInfo.Points.Count);

            //100th point's angle and distance
            Console.WriteLine("The 100th point's angle：{0}° ,distance:{1}mm \n", OneCirclePoint.ScanInfo.Points[99].Angle.ToString("#0.00"),
													 OneCirclePoint.ScanInfo.Points[99].Dist);

            OneCirclePoint.RdyFlag = 0;
            
        }
        /****************************************************************************************************************************************************/
		/*
        public void RxThreadFunct()
        {
            while (true)
            {
                RxDataProcess();

                System.Threading.Thread.Sleep(2);
            }
        }
		*/
    }
}
