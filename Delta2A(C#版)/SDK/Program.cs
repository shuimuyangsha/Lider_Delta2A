using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;

namespace SDK
{
    class Program
    {
        static void Main(string[] args)
        {
            Lidar lidar = new Lidar();
            //打开串口
            bool rtn = lidar.SerialOpen("COM3",230400);
            if(rtn == false)
            {
                Console.WriteLine("串口打开失败，确认COM口\n");
            }
          
            while (true)
            {               
                    //解析串口数据
                    lidar.RxDataProcess();

                    //打印雷达每一圈点信息
                    lidar.PrintOneCirclePointInformation();
                                
                System.Threading.Thread.Sleep(2);
            }
        }
    }
}
