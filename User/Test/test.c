#include "test.h"
#include "LCD/lpc177x_8x_lcd.h"
#include "Globalvalue/GlobalValue.h"
#include "use_disp.h"
#include "EX_SDRAM/EX_SDRAM.H"
#include "stdint.h"
#include "key/key.h"
#include <string.h>
#include "timer.h"
#include "lpc177x_8x_eeprom.h"
#include "math.h"
#include  "usbhost_fat.h"
#include "lpc177x_8x_clkpwr.h"
#include "LCD/logo.h"
#include "lpc177x_8x_rtc.h"
#include "debug_frmwrk.h"
#include "lpc177x_8x_gpio.h"
#include "open.h"
#include "ff.h"
#include <stdlib.h>
#include "lpc177x_8x_gpdma.h"
#include "bsp_bmp.h"

FATFS fs;         /* Work area (file system object) for logical drive */
FIL fsrc;         /* file objects */   
FRESULT res;
UINT br;
const char *PASSWORD="12345689";
const uint8_t USB_dISPVALUE[][9]=
{
	"RH_FAIL ",
	"RL_FAIL ",
	"VH_FAIL ",
	"VL_FAIL ",
	"ALL_FAIL",
	"ALL_PASS"
};
struct MODS_T g_tModS;
extern volatile uint32_t timer1_counter;
extern volatile uint32_t stable_counter;
extern vu8 vflag;
vu8 nodisp_v_flag=0;
vu16 stabletime;
vu32 rwatch;
vu32 vwatch;
vu32 testwatch[500];
vu8 u3sendflag;
vu32 uartresdelay;
vu32 Tick_10ms=0;
vu32 OldTick;
vu8 g_mods_timeout;
vu8 powerontest;
extern vu8 u3sendflag;
extern vu8 negvalm;
uint32_t keynum;
Filter Rfilter,Vfilter;
u16 filtersize[3] = {16,32,60};//快中慢速滤波次数
u16 dispfilter[3] = {1,2,10};	 //快中慢速显示二次滤波次数
vu8 trip_flag=0;//手动触发标志
u8 testtimingflag;
u32 timing;
uint32_t colorbuf[480];
u8 rangenum[5] = {4,5,5,5,5};//不同版本总量程数
double maxv[5] = {30,100,300,600,1000};//不同版本电压上限
double maxvdisp[5] = {300000,100000,300000,600000,100000};//不同版本电压上限
double maxvdot[5] = {4,3,3,3,2};//不同版本电压上限小数点
double x1,y1,x2,y2;
u8 bmpname[30];
u8 vcalstep[6]={1,2,1,3,1,4};
u8 clearfalg;
u16 roffcoief[7]={1,10,100,1,10,100,1000};
u8 rangedot[7] = {4,3,2,4,3,2,1};

const u8 DOT_POS[6]=
{	
	2,
	1,
	3,
	2,
	0
};
//#include "debug_frmwrk.h"
const vu8 Uart_Ordel[]={0x60,0x70,0x71,0x80,0x90,0xa0,0xb0,0xc0,0xe0};
const vu8 READDATA[7]={0xAB,0x01,0x06,0x03,0x08,0xbf,'\0'};
const vu8 Disp_Main_Ord[][3]={
	{1,1,0},
	{1,1,1},
	{1,1,2},
	{1,1,3},//Cp
	
	{0,1,0},
	{0,1,1},
	{0,1,3},//Cs
	{1,0,0},
	{1,0,0},
	{1,0,0},
	{1,0,1},//Lp
	{1,0,3},
	{1,0,4},
	{1,0,0},
	{0,0,0},//Ls
	{0,0,1},
	{0,0,5},
	{0,0,4},
	{0,3,6},//Z
	{0,3,7},
	{0,4,6},//Y
	{0,4,7},
	{0,2,8},//R
	{1,2,1},
	{0,2,1},
	{1,1,0},
	{1,5,9},//GB
	//{1,1,0},
	};

void Swap(uint32_t A[], uint16_t i, uint16_t j)
{
    int temp = A[i];
    A[i] = A[j];
    A[j] = temp;
}

//冒泡排序
void BubbleSort(uint32_t A[], uint16_t n)
{
	int i,j;
    for (j = 0; j < n - 1; j++)         // 每次最大元素就像气泡一样"浮"到数组的最后
    {
        for (i = 0; i < n - 1 - j; i++) // 依次比较相邻的两个元素,使较大的那个向后移
        {
            if (A[i] > A[i + 1])            // 如果条件改成A[i] >= A[i + 1],则变为不稳定的排序算法
            {
                Swap(A, i, i + 1);
            }
        }
    }
}

//已排序递增数组队首进入新数据排序
void HeadSort(uint32_t A[], uint16_t n)
{
	int i;
	for(i = 0;i < n-1;i ++)
	{
		if (A[i] > A[i + 1])
		{
				Swap(A, i, i + 1);
		}else{
			break;
		}
	}
}

//已排序递增数组队尾进入新数据排序
void TailSort(uint32_t A[], uint16_t n)
{
	int i;
	for(i = n-1;i > 0;i --)
	{
		if (A[i] < A[i - 1])
		{
				Swap(A, i, i - 1);
		}else{
			break;
		}
	}
}
//==========================================================
//函数名称：Power_Process
//函数功能：上电处理
//入口参数：无
//出口参数：无
//创建日期：2015.10.26
//修改日期：2015.10.26 08:53
//备注说明：开机长按SET进入校准调试模式
//==========================================================
void MODS_Poll(void)
{
	uint16_t addr;
	static uint16_t crc1;
    static vu32 testi;
	/* 超过3.5个字符时间后执行MODH_RxTimeOut()函数。全局变量 g_rtu_timeout = 1; 通知主程序开始解砿*/
//	if (g_mods_timeout == 0)	
//	{
//		return;								/* 没有超时，继续接收。不要清雿g_tModS.RxCount */
//	}

    testi=g_tModS.RxCount;
    testi=g_tModS.RxCount;
    testi=g_tModS.RxCount;
	if(testi>7)				/* ??????С?4???????? */
	{
		testi=testi;
	}
	testi=g_tModS.RxCount;
    if(testi==8)				/* ??????С?4???????? */
	{
		testi=testi+1;
	}
	//??????ˇ???
	if(OldTick!=Tick_10ms)
  	{  
	  OldTick=Tick_10ms;
	   if(g_mods_timeout>0)
      { 
	    g_mods_timeout--;
      }
	  if(g_mods_timeout==0 && g_tModS.RxCount>0)   //??????
      { 
		// goto err_ret;
	
      }
      else if(g_mods_timeout==0 && g_tModS.RxCount==0) //?????
         return;
      else //????ì???
         return;
	}
	else   //???10msì?????
		return;
	//g_mods_timeout = 0;	 					/* ??? */

	if (g_tModS.RxCount < 4)				/* ??????С?4???????? */
	{
		goto err_ret;
	}

	/* ??CRCУ?? */
// 	crc1 = CRC16(g_tModS.RxBuf, g_tModS.RxCount);
// 	if (crc1 != 0)
// 	{
// 		goto err_ret;
// 	}

// 	/* ??? (1??é */
// 	addr = g_tModS.RxBuf[0];				/* ?1?? ?? */
// 	if (addr != SADDR485)		 			/* ???????????ˇ??? */
// 	{
// 		goto err_ret;
// 	}

	/* 分析应用层协访*/
//    if(g_tModS.RxBuf[2] == 0xA5)
//    {
//        UART_Action();
//    }else{
//        usartocflag = 1;
        u3sendflag = 1;
				uartresdelay = 2;
//        RecHandle();
//        u3sendflag = 0;
//    }
							
	
err_ret:
#if 0										/* 此部分为了串口打印结枿实际运用中可不要 */
	g_tPrint.Rxlen = g_tModS.RxCount;
	memcpy(g_tPrint.RxBuf, g_tModS.RxBuf, g_tModS.RxCount);
#endif
	
// 	g_tModS.RxCount = 0;					/* 必须清零计数器，方便下次帧同歿*/
}

void Power_Process(void)
{
	vu16 i,key;
//	u8 buff[4096];
//	uint32_t  numBlks, blkSize;
//	uint8_t  inquiryResult[INQUIRY_LENGTH];
	Disp_Coordinates_Typedef Debug_Cood;
//	u8 rc;
//
	lcd_Clear(LCD_COLOR_TEST_BACK);
	HW_keyInt();
    Beep_on();
    ReadSavedata();
    Set_Compbcd_float();
//	Save_Res.Set_Data.dispvr=1;//固定只显示电压
	Uart3_init(2/*Save_Res.Sys_Setvalue.buard*/);//波特率默认9600

	Bais_LedOff();
	Lock_LedOff();
	Pass_Led();
	Power_Off_led();
	Turnon_backlight();
	if(Save_Res.open == 1)
	{
		DrawLogo(140,160);
//		lcd_image((uint8_t *)gImage_open);
	}
	
	
	init_timer(0, 20);//定时器初始化   	
	enable_timer(0);
    
//    init_timer(1, 10);//定时器1初始化10ms
//    enable_timer(1);
	Uart_Send_Flag=0;
//	EEPROM_Init();

	Host_Init();               /* Initialize the lpc17xx host controller                                    */
//     Host_EnumDev();       /* Enumerate the device connected                                            */
//    if (rc == OK) {
//		/* Initialize the mass storage and scsi interfaces */
//        rc = MS_Init( &blkSize, &numBlks, inquiryResult );
//        if (rc == OK) {
//            rc = FAT_Init();   /* Initialize the FAT16 file system */    
//		Write_Usbdata ( "电阻		电压		分选\r\n" ,19);			
////            if (rc == OK) {
////                Main_Copy();   /* Call the application                                                      */
////            } 
//        } 
//    } 
	if(Save_Res.open == 1)
	{
		Delay(1800/5);
	}
	Beep_Off();
    i=0;
	powerontest = 1;
	Vfilter.initflag = 1;
	Vfilter.initcount = 0;
//UART_TxCmd(LPC_UART3, ENABLE);
	while(GetSystemStatus()==SYS_STATUS_POWER)
	{
	    
        i++;
        Delay(10/5);
        if(i>10)
            SetSystemStatus(SYS_STATUS_TEST);//待测状态
         key=HW_KeyScsn();
		if(key==0xff)
		{
			keynum=0;
		}
		else
			keynum++;
		if(keynum==KEY_NUM)
		{	
			//Disp_Flag=1;
			Key_Beep();
            switch(key)
			{
				case Key_FAST:
                    lcd_Clear(LCD_COLOR_TEST_BACK);
                    Debug_Cood.xpos=180;
                    Debug_Cood.ypos =120;
                    Debug_Cood.lenth=120;
                    input_password(&Debug_Cood);
                    break;
                default:
                    break;
                }
            }
//		i++;
//		if(i>POWERON_DISP_TIME)//延时20*100mS=2s后自动退出
			

//		key=Key_Read_WithTimeOut(TICKS_PER_SEC_SOFTTIMER/10);//等待按键(100*10ms/10=100ms)
//		switch(key)
//		{
////			case KEY_SET:		//设置键
//			case L_KEY_SET:		//长按设置键
//				//SetSystemStatus(SYS_STATUS_DEBUG);//调试状态
//                
//				Beep_One();//蜂鸣器响一声
//				break;
//	
//			case KEY_UP:		//上键
//			case L_KEY_UP:		//长按上键
//				break;
//	
//			case KEY_DOWN:		//下键
//			case L_KEY_DOWN:	//长按下键
//				break;
//	
//			case KEY_LEFT:		//左键
//			case L_KEY_LEFT:	//长按左键
//				break;

//			case KEY_RIGHT:		//右键
//			case L_KEY_RIGHT:	//长按右键
//				break;

//			case KEY_ENTER:		//确认键
//			case L_KEY_ENTER:	//长按确认键
//				break;
//	
//			case KEY_START:		//启动键
//			case L_KEY_START:	//长按启动键
//				break;
//	
//			case KEY_RESET:		//复位键
//			case L_KEY_RESET:	//长按复位键
//				i=POWERON_DISP_TIME;//显示延时结束
//				break;
//			
//			default:
//				break;
//		}
		
		//Delay_1ms(50);
		//Range_Control(3);
	}
	
}

void string_to_float(char *string, double *data)
{
	unsigned int i=0,j=0,k=0;
	unsigned char flag=0;  			//判断正负号的标志
	unsigned char flag_dot=1; 	//判断小数点的标志
	double num=0;     					//临时存储计算结果的变量
	
	for(i=0;*(string+i)!=0x00;i++) //循环直到字符串结尾
	{
		if(*(string+i)>='0'&&*(string+i)<='9'&&flag_dot==1)  //如果当前字符为数字且在小数点之前
		{
			if(j==0) num = num*pow(10,j)+(double)(*(string+i)-'0');     //运算并存储中间计算结果
			else     num = num*pow(10,1)+(double)(*(string+i)-'0');
			j++;
		}
		else if(*(string+i)>='0'&&*(string+i)<='9'&&flag_dot==0) //如果当前字符为数字且在小数点之后
		{
			num = num+(double)(*(string+i)-'0')*pow(0.1,j+1);     //运算并存储中间计算结果
			j++;
		}
		else if(*(string+i)=='.')                               //读到了小数点则将对应标志位数值改变
		{
			flag_dot=0;
			j=0;
		}
		else if(*(string+i)=='-')                              //读到减号同样改变对应标志位的值
		{
			flag = 1;
		}
		else if(*(string+i)==',')                             //读完一个数据，重置标志位，记录最终计算结果
		{
			*(data+k) = num*pow(-1,flag);
			flag = 0;
			flag_dot=1;
			j=0;
			k++;
			num = 0;
		}
	}
	*(data+k) = num*pow(-1,flag);                            //补上最后一个数
}

void ReadPointTest(void)
{
	uint16_t i;
	for(i=0;i<480;i++)
	{
		colorbuf[i] = LCD_ReadPixel(i,1);
	}
}

//测试程序
void Test_Process(void)
{
	char newfile[30];
	char filename[30];
	static vu16 j;
	Button_Page_Typedef Button_Page;
//	Main_Second_TypeDed Main_Second;//主参数和幅参赛的序号
	vu32 keynum=0;
	float ddd,eee;
	vu8 key,i;

    vu8 timebuff[10];
	
    vu8 send_usbbuff[100];
	vu8 return_flag=0;
//	vu8 Usb_Sendbuff[30]={"       Ω"};
//    vu8 page=1;
	vu8 Disp_Flag=1;
//	vu8 uart_count;
	
	vu8 lock_flag=0;
//	vu8 rc;
	vu8 chosen=ALL_PASS,chosen1=ALL_PASS,comp=ALL_PASS;
	Send_Ord_Typedef Uart;
    vu8 sendtest[5] ={1,2,3,4,5};
    
    uint32_t  numBlks, blkSize;
	uint8_t  inquiryResult[INQUIRY_LENGTH],rc;
//    GPIO_ClearInt(0, 1<<19);
//    NVIC_EnableIRQ(GPIO_IRQn);
	Button_Page.page=0;
	Button_Page.index=0;
	Button_Page.third=0xff;
	lpc1788_DMA_Init();
	ComBuf.pageswflag=0;
//     if(Save_Res.Sys_Setvalue.uart)
//         UART_TxCmd(LPC_UART3, ENABLE);
//     else
//         UART_TxCmd(LPC_UART3, DISABLE);
//     Uart3_init(Save_Res.Sys_Setvalue.buard);
    lcd_Clear(LCD_COLOR_TEST_BACK);
	Disp_Test_Item();
    //Button_Page.index=0;
//	Main_Second.Main_flag=0;
//	Main_Second.Second_falg=0;
	
	Delay_Key();
//	uart_count=0;
	clear_flag=0;
	Rfilter.initflag = 1;//每次重新进入测试页面都初始化滤波队列
//	Send_UartStart();//开始时的串口发送数据
	while(GetSystemStatus()==SYS_STATUS_TEST)
	{
		
//		GPIO_SetValue(0, (1<<22));
        if(Rtc_intflag)
        {
            Rtc_intflag=0;
            Colour.Fword=LCD_COLOR_WHITE;
            Colour.black=LCD_COLOR_TEST_BACK;
            //sprintf((char *)timebuff,"%2d:%2d:%2d",RTC_TIME_DISP.HOUR,RTC_TIME_DISP.MIN,RTC_TIME_DISP.SEC);
            //Hex_Format(RTC_TIME_DISP.HOUR, 0, 2, 1);
            timebuff[0]=RTC_TIME_DISP.HOUR/10+'0';
            timebuff[1]=RTC_TIME_DISP.HOUR%10+'0';
            timebuff[2]=':';
            timebuff[3]=RTC_TIME_DISP.MIN/10+'0';
            timebuff[4]=RTC_TIME_DISP.MIN%10+'0';
            timebuff[5]=':';
            timebuff[6]=RTC_TIME_DISP.SEC/10+'0';
            timebuff[7]=RTC_TIME_DISP.SEC%10+'0';
            timebuff[8]=0;
            
            WriteString_16(LIST1+360, 2, timebuff,  0);
						sprintf((char *)bmpname,"0:20%0.2d%0.2d%0.2d%0.2d%0.2d%0.2d.bmp", 
						RTC_TIME_DISP.YEAR,
						RTC_TIME_DISP.MONTH, 
						RTC_TIME_DISP.DOY,
						RTC_TIME_DISP.HOUR, 
						RTC_TIME_DISP.HOUR,
						RTC_TIME_DISP.SEC);
        }
        if(Disp_Flag==1)
		{
			Disp_Test_value(&Button_Page);
			
			Disp_Flag=0;
		
		}//
		//_printf("CoreClock: %s\n",READDATA); 
		
		Colour.black=LCD_COLOR_TEST_MID;

        if(nodisp_v_flag)
            eee=0;

		if(nodisp_v_flag)
            eee=0;
			//ddd-=Save_Res.clear;
		//分选比较打开
		if(Save_Res.Set_Data.V_comp && Save_Res.Set_Data.dispvr != 2)//电压比较
		{
			chosen=V_Test_Comp(Test_Dispvalue.Vdata);
			
		
		
		}
		if(Save_Res.Set_Data.Res_comp && Save_Res.Set_Data.dispvr != 1)//电阻比较打开
		{
			chosen1=R_Test_Comp(Test_Dispvalue.Rdata);
	//			if(R_Test_Comp(ddd)==ALL_PASS)
			
	//				;
		
		}	
		if(chosen==ALL_PASS&&chosen1==ALL_PASS)
		{
			comp=ALL_PASS;
			//Pass_Led();
		} 
		else if(chosen!=ALL_PASS&&chosen1!=ALL_PASS)
		{
			comp=ALL_FAIL;
			//Fail_led();
		
		}
		else
		{
			if(chosen==VH_FAIL||chosen==VL_FAIL)
				comp=chosen;
			else if(chosen1==RH_FAIL||chosen1==RL_FAIL)
				comp=chosen1;
			//Fail_led();
				
		
		}
		#ifdef OVER_NO_ALARM
		if(nodisp_v_flag==0)
			Comp_prompt(comp);
		else
		{
			No_Comp();
		
		}
			
		#else
		if(Save_Res.Set_Data.V_comp || Save_Res.Set_Data.Res_comp || Save_Res.Set_Data.openbeep==1)
		{
			Comp_prompt(comp);
		}else{
			No_Comp();
		}
		#endif
//		if(return_flag)	
//		{
//            rwatch = (int)ddd;
//            vwatch = (int)eee * 10;
//			testwatch[j] = rwatch;
			
			if(j < 499)
				j++;
			if(j == 5)
			{
				stabletime = stable_counter * 20;
			}
			BCD_Int(ddd);//DOT_POS	
			for(i=0;i<6;i++)
			{
				*(UserBuffer+i)=Test_Dispvalue.Rvaluebuff[i];
				Send_buff2[i]=DispBuf[i];
			}
			if(Test_Dispvalue.Rdataraw.unit)
			{
				*(UserBuffer+6)=' ';
				Send_buff2[6]=' ';
				
			}
			else
			{
				*(UserBuffer+6)='m';
				Send_buff2[6]='m';
				
			}
			*(UserBuffer+7)=0xa6;
			*(UserBuffer+8)=0xb8;
			*(UserBuffer+9)='	';
			Disp_R_X();//显示单位
			if(nodisp_v_flag == 1)
			{
				if(eee > 0 && eee < 200)
				{
					eee = 0;
					vflag = 1;
				}else if(eee <=0 && eee > -200){
					eee = 0;
					vflag = 1;
				}else{
					vflag = 0;
				}
			}else{
				vflag = 0;
			}
			testtimingflag = 1;
			Disp_Testvalue(comp,eee);
			testtimingflag = 0;
//			if(Test_Unit.V_Neg)
//			{
//				*(UserBuffer+9)=' ';
//				Send_buff2[6]=' ';
//				
//			}
//			//WriteString_Big(100,92+55 ," ");
//		else
//		{
//			*(UserBuffer+9)='-';
//			Send_buff2[6]='-';
//			
//		}
			//WriteString_Big(100,92+55 ,"-");
		for(i=0;i<8;i++)
		{
			*(UserBuffer+10+i)=Test_Dispvalue.Vvaluebuff[i];
			Send_buff2[7+i]=DispBuf[i];
			
		}
			Send_buff2[12]=comp;
			Send_buff2[13]=0;
			*(UserBuffer+18)='V';
			*(UserBuffer+19)='	';
			for(i=0;i<8;i++)
			*(UserBuffer+20+i)=USB_dISPVALUE[comp][i];
		
			*(UserBuffer+28)='\r';
			*(UserBuffer+29)='\n';
        *(UserBuffer+30)='\0';
//			return_flag=0;
            
			strcpy((char *)send_usbbuff,(char *)timebuff);
			strcat((char *)send_usbbuff,(char *)"   ");
			strcat((char *)send_usbbuff,(char *)UserBuffer);
			if(Save_Res.Sys_Setvalue.u_flag && usb_oenflag == 1)
			{
				strcpy(filename,"0:/"); 
				strcat(filename,(char*)Save_Res.Sys_Setvalue.textname); 
				strcat(filename,(char*)".TXT");
		 //     
				res = f_open( &fsrc , filename ,  FA_WRITE);
		//		fdw = FILE_Open((uint8_t *)filename, RDWR);
				if (res == 0) 
				{
//					usb_oenflag=1;
					f_lseek(&fsrc,fsrc.fsize);
		//			bytes_written = FILE_Write(fdw, buffer, num);//MAX_BUFFER_SIZE);

					res = f_write(&fsrc, &send_usbbuff, 41, &br);     
					f_close(&fsrc); 
		//			usb_oenflag=1;

		//			bytes_written = FILE_Write(fdw, buffer, num);//MAX_BUFFER_SIZE);

		//			FILE_Close(fdw);
								
				} 
//				else
//					usb_oenflag=0;
//					Write_Usbdata ( send_usbbuff,38);//27
			}
			else
			{
					usb_oenflag=0;
			
			}
// 			if(Save_Res.Sys_Setvalue.uart)
// 				UARTPuts( LPC_UART3, sendtest);
	
//		}
		Colour.Fword=LCD_COLOR_WHITE;
//		if(timer1_counter > 0)
//        {
            Tick_10ms ++;
		if(u3sendflag == 0)
      MODS_Poll();
//            timer1_counter = 0;
//        }
		//	Test_Comp(&Comp_Change);
        if(Button_Page.index==0)
        {
            if(usb_oenflag==1)
						{
								//Write_Usbdata ( UserBuffer,27);
								Disp_Usbflag(1);
								
						}
						else
							Disp_Usbflag(2);
							Colour.Fword=LCD_COLOR_WHITE;
							Colour.black=LCD_COLOR_TEST_BACK;
        }
		
	  key=HW_KeyScsn();
		if(key==0xff)
		{
			keynum=0;
		}
		else
			keynum++;
		if(keynum==KEY_NUM)
		{	
			Disp_Flag=1;
            Key_Beep();
			switch(key)
			{
				case Key_F1:
					
                    switch(Button_Page.index)
					{
						case 0:
							//if(Button_Page.page==0)
								SetSystemStatus(SYS_STATUS_TEST);
							break;
						case 1:
							Save_Res.Set_Data.trip=0;

							
							break;

						default:
							break;
					
					
					}
					Savetoeeprom();
				break;
				case Key_F2:
					switch(Button_Page.index)
					{
						case 0:
							//if(page==1)
								SetSystemStatus(SYS_STATUS_SETUPTEST);
							break;
						case 1:
							Save_Res.Set_Data.trip=1;

							
							break;

						
						default:
							break;
					
					
					}
					Savetoeeprom();
				break;
				case Key_F3:
					switch(Button_Page.index)
					{
						case 0:
							//if(page==1)
								SetSystemStatus(SYS_STATUS_SYSSET);
							break;

						case 6:
							
							break;
						
						default:
							break;
					
					
					}
//					Savetoeeprom();
				break;
				case Key_F4:
					switch(Button_Page.index)
					{
						case 0:
							SetSystemStatus(SYS_STATUS_SYS);
							break;
//						
						default:
							break;
					
					
					}	
//					Savetoeeprom();
				break;
				case Key_F5:
					if(Save_Res.Sys_Setvalue.u_flag)
					{
						Host_Init();
						//rc = Host_EnumDev();       /* Enumerate the device connected                                            */
						do{
							res=Host_EnumDev();
							j++;
			
						}
						while(res!=0&&j<50);
						if(res==0)
							usb_oenflag=1;
						else
							usb_oenflag=0;
						if(usb_oenflag)	
						{
							res=f_mount(0,&fs);
							Host_DelayMS(20);
					//		res=f_mkdir("0:/jk");
					//        #ifdef ZC5520 
							strcpy(newfile,"0:/"); 
							strcat(newfile,(char*)Save_Res.Sys_Setvalue.textname); 
							strcat(newfile,(char*)".TXT");
							res = f_open( &fsrc , newfile , FA_CREATE_NEW | FA_WRITE);//FA_CREATE_NEW | FA_WRITE);	
							if(Save_Res.Sys_Setvalue.lanage )
								res = f_write(&fsrc,  "Times    Resistance		Voltage		Sorting\r\n", sizeof( "Times    Resistance		Voltage		Sorting\r\n"), &br);
							else
								res = f_write(&fsrc,  "时间   电阻  	电压	 分选\r\n", sizeof( "时间   电阻  	电压	 分选\r\n"), &br);							
							
					//        #else
					//            res = f_open( &fsrc , "0:/ZC/ZC2683A.xls" , FA_CREATE_NEW | FA_WRITE);//FA_CREATE_NEW | FA_WRITE);
					//        #endif

							if ( res == FR_OK )
							{ 
					//			f_lseek(&fsrc,fsrc.fsize);
					//			res = f_write(&fsrc, "编号	电压	电阻	时间	分选\t\n", sizeof("编号	电压	电阻	时间	分选\t\n"), &br);     
								f_close(&fsrc); 
							}
							
						}
					}
//                    if(Save_Res.Sys_Setvalue.u_flag)
//                    {
//                        Host_Init();               /* Initialize the lpc17xx host controller                                    */
//                        rc = Host_EnumDev();       /* Enumerate the device connected                                            */
//                        if (rc == OK) {
//                            /* Initialize the mass storage and scsi interfaces */
//                            rc = MS_Init( &blkSize, &numBlks, inquiryResult );
//                            if (rc == OK) {
//                                rc = FAT_Init();   /* Initialize the FAT16 file system */   
//														if(Save_Res.Sys_Setvalue.lanage )
//															Write_Usbdata ( "Times   Resistance		Voltage		Sorting\r\n" ,27);	
//														else
//															Write_Usbdata ( "时间     电阻		电压		分选\r\n" ,27);	
//								if (rc == OK)
//								{
//									usb_oenflag=1; 
//								}								
//                    
//                            } 
//                        } 
//                    }

				break;
				case Key_Disp:
                    //SetSystemStatus(SYS_STATUS_TEST);
				break;
				case Key_SETUP:
                    SetSystemStatus(SYS_STATUS_SETUPTEST);
                    //Savetoeeprom();
				break;
				case Key_FAST:
//					Screen_shot(0,0,480,272,(const char *)bmpname);
				break;
				case Key_LEFT:
					if(Button_Page.index==0)
						Button_Page.index=1;
					else
					//if(Button_Page.index>3)
						Button_Page.index=0;
//					else
//						Button_Page.index+=2;
//					Button_Page.page=0;
				break;
				case Key_RIGHT:
					if(Button_Page.index==0)
						Button_Page.index=1;
					else
					//if(Button_Page.index<=3)
						Button_Page.index=0;
//					else
//						Button_Page.index-=2;
				//	Button_Page.page=0;
						
				break;
				case Key_DOWN:
					if(Button_Page.index>0)
						Button_Page.index=0;
					else
						Button_Page.index=1;
					//Button_Page.page=0;
					
				break;
				case Key_UP:
					if(Button_Page.index==0)
						Button_Page.index=1;
					else
						Button_Page.index=0;
					//Button_Page.page=0;
				break;
				
				case Key_NUM1:
				//break;
				case Key_NUM2:
				//break;
				case Key_NUM3:
				//break;
				case Key_NUM4:
				//break;
				case Key_NUM5:
				//break;
				case Key_NUM6:
				//break;
				case Key_NUM7:
				//break;
				case Key_NUM8:
				//break;
				case Key_NUM9:
				//break;
				case Key_NUM0:
					Uart_Send_Flag=1;
				break;
				case Key_DOT:
//					if(Button_Page.index==2)
//					{ 	Disp_Coordinates_Typedef Coordinates;
//						Coordinates.xpos=LIST1+88;
//						Coordinates.ypos=FIRSTLINE+SPACE1*1;
//						Coordinates.lenth=86;
//						SaveData.Main_Func.Freq=Freq_Set_Num(&Coordinates);
//						Uart.Ordel=Uart_Ordel[3];
//							Uart.name=SaveData.Main_Func.Freq;
//							Uart_Send_Flag=2;
//					
//					}
				break;
				case Key_BACK:
					
				break;
				case Key_LOCK:
					//clear_flag=1;
//					if(lock_flag)
//						lock_flag=0;
//					else
//						lock_flag=1;
//					if(lock_flag)
//					{
//						Lock_Control_On();
//						Lock_LedOn();
//						
//					}
//					else
//					{
//						Lock_LedOff();
//						Lock_Control_Off();
//					
//					
//					}
				break;
				case Key_BIAS:
					clearfalg = 1;
//					Uart_Send_Flag=3;
//					clear_flag=1;
//					Bais_LedOn();
//					Delay(500);
				break;
				case Key_REST:
					Power_Off_led();
				break;
				case Key_TRIG:
					trip_flag=1;
				break;
				default:
				break;
					
			}
		
		
		}
	
//		testtimingflag = 0;
//		GPIO_ClearValue(0, (1<<22));
	}

}
//文件管理程序
void File_Process(void)
{
	 vu32 keynum=0;
	 vu8 key;
	Delay_Key();
  	while(GetSystemStatus()==SYS_STATUS_FILE)
	{
	 key=HW_KeyScsn();
		if(key==0xff)
		{
			keynum=0;
		}
		else
			keynum++;
		if(keynum==2)
		{   
            Key_Beep();
			switch(key)
			{
				case Key_F1:
				break;
				case Key_F2:
				break;
				case Key_F3:
				break;
				case Key_F4:
				break;
				case Key_F5:
				break;
				case Key_Disp:
				break;
				case Key_SETUP:
				break;
				case Key_FAST:
				break;
				case Key_LEFT:
				break;
				case Key_RIGHT:
				break;
				case Key_UP:
				break;
				case Key_DOWN:
				break;
				case Key_NUM1:
				break;
				case Key_NUM2:
				break;
				case Key_NUM3:
				break;
				case Key_NUM4:
				break;
				case Key_NUM5:
				break;
				case Key_NUM6:
				break;
				case Key_NUM7:
				break;
				case Key_NUM8:
				break;
				case Key_NUM9:
				break;
				case Key_NUM0:
				break;
				case Key_DOT:
				break;
				case Key_BACK:
				break;
				case Key_LOCK:
				break;
				case Key_BIAS:
				break;
				case Key_REST:
				break;
				case Key_TRIG:
				break;
				default:
				break;
					
			}
		
		
		}
	
	
	
	}

}
//参数设置程序
void Setup_Process(void)
{

	Button_Page_Typedef Button_Page;
	Disp_Coordinates_Typedef  Coordinates;
	Send_Ord_Typedef Uart;
	
	vu32 keynum=0;
	vu8 key;
//    vu8 page=1;
	vu8 Disp_Flag=1;
//	vu8 index=0;
//	vu32 *pt;
//	pt=(vu32 *)&SaveData.Main_Func;
	Button_Page.index=0;
	Button_Page.page=0;
	
	lcd_Clear(LCD_COLOR_TEST_BACK);
	Disp_Test_Set_Item();
	Delay_Key();
 	while(GetSystemStatus()==SYS_STATUS_SETUPTEST)
	{
	  if(missflag == 1)
		{
			lpc1788_DMA_SetInit();
			missflag = 0;
		}
		if(Save_Res.Set_Data.trip == 1)
		{
			lpc1788_DMA_SetInit();
		}
		if(Disp_Flag==1)
		{
			DispSet_value(&Button_Page);
			Disp_Flag=0;
			Delay_Key();
		
		}

		key=HW_KeyScsn();
		if(key==0xff)
		{
			keynum=0;
		}
		else
			keynum++;
		if(keynum==5)
		{	Disp_Flag=1;
            Key_Beep();
			switch(key)
			{
				case Key_F1:

					switch(Button_Page.index)
					{
						case 0:
							//if(Button_Page.page==0)
								ComBuf.pageswflag=1;
								SetSystemStatus(SYS_STATUS_TEST);//
//							else
//								SetSystemStatus(SYS_STATUS_FILE);
								
							break;
						case 1:
							Save_Res.Set_Data.trip=0;
							
							break;
						case 2:
							Save_Res.Set_Data.speed=0;
							Send_Speed();
//							Uart_Send_Flag=2;
								
							
							break;
						case 3:
							Save_Res.Set_Data.dispvr=0;
							
								
							
							break;
						case 3+1:
							Save_Res.Set_Data.Res_comp=0;
							Uart_Send_Flag=2;
							//SaveData.Main_Func.Level=0;
							break;
						case 4+1:
//							if(Save_Res.Set_Data.Res_low.Num<9000)
//								Save_Res.Set_Data.Res_low.Num+=1000;
//							else
//								Save_Res.Set_Data.Res_low.Num-=9000;
//							
							break;
						case 5+1:
							Save_Res.Set_Data.V_comp=0;
							Uart_Send_Flag=2;
							break;
						case 6+1:
							
//							if(Save_Res.Set_Data.V_low.Num<9000)
//								Save_Res.Set_Data.V_low.Num+=1000;
//							else
//								Save_Res.Set_Data.V_low.Num-=9000;
//							
							break;
						case 7+1:
							Save_Res.Set_Data.Range=0;
							Send_Range();
							break;
						case 8+1:
							Save_Res.Set_Data.beep=0;
							break;
						case 9+1:
							Save_Res.Set_Data.VRange=0;
							Send_Vrange();
							break;
						case 10+1:
//							if(Save_Res.Set_Data.High_Res.Num<9000)
//								Save_Res.Set_Data.High_Res.Num+=1000;
//							else
//								Save_Res.Set_Data.High_Res.Num-=9000;
//							Uart_Send_Flag=2;
							break;
						case 11+1:
//							if(Save_Res.Set_Data.Nominal_V.Num<9000)
//								Save_Res.Set_Data.Nominal_V.Num+=1000;
//							else
//								Save_Res.Set_Data.Nominal_V.Num-=9000;
//							Uart_Send_Flag=2;
							break;
						case 12+1:
//							if(Save_Res.Set_Data.V_high.Num<9000)
//								Save_Res.Set_Data.V_high.Num+=1000;
//							else
//								Save_Res.Set_Data.V_high.Num-=9000;
//							Uart_Send_Flag=2;
							
							break;
						
							
						default:
							break;
					
					
					}

				break;
				case Key_F2:
					

					switch(Button_Page.index)
					{
						case 0:
							//if(Button_Page.page==0)
								SetSystemStatus(SYS_STATUS_SETUPTEST);
//							else
//								SetSystemStatus(SYS_STATUS_SYSSET);
								
							break;
						case 1:
							
							Save_Res.Set_Data.trip=1;
							
							
							break;
						case 2:
							Save_Res.Set_Data.speed=1;
							Send_Speed();
//							Uart_Send_Flag=2;
							
							
							break;
						case 3:
							Save_Res.Set_Data.dispvr=1;
							
								
							
							break;
						case 3+1:
							Save_Res.Set_Data.Res_comp=1;
							Uart_Send_Flag=2;
							break;
						case 4+1:
//							if(Save_Res.Set_Data.Res_low.Num%1000<900)
//								Save_Res.Set_Data.Res_low.Num+=100;
//							else
//							{
//								//Save_Res.Set_Data.Res_low.Num%=100;
//								Save_Res.Set_Data.Res_low.Num-=900;
//								
//							}
							break;
						case 5+1:
							Save_Res.Set_Data.V_comp=1;
							Uart_Send_Flag=2;
							break;
						case 6+1:
//							if(Save_Res.Set_Data.V_low.Num%1000<900)
//								Save_Res.Set_Data.V_low.Num+=100;
//							else
//								Save_Res.Set_Data.V_low.Num-=900;
//							Uart_Send_Flag=2;
							break;
						case 7+1:
							if(Save_Res.Set_Data.Range > 1)
								Save_Res.Set_Data.Range--;
							Send_Range();
							break;
						case 8+1:
							Save_Res.Set_Data.beep=1;
							break;
						case 9+1:
							if(Save_Res.Set_Data.VRange > 1)
								Save_Res.Set_Data.VRange--;
							Send_Vrange();
							break;
//						case 9:
//							if(Save_Res.Set_Data.Nominal_Res.Num%1000<900)
//								Save_Res.Set_Data.Nominal_Res.Num+=100;
//							else
//								Save_Res.Set_Data.Nominal_Res.Num-=900;
//							Uart_Send_Flag=2;
//							break;
//						case 10:
//							if(Save_Res.Set_Data.High_Res.Num%1000<900)
//								Save_Res.Set_Data.High_Res.Num+=100;
//							else
//								Save_Res.Set_Data.High_Res.Num-=900;
//							Uart_Send_Flag=2;
//							break;
//						case 11:
//							if(Save_Res.Set_Data.Nominal_V.Num%1000<900)
//								Save_Res.Set_Data.Nominal_V.Num+=100;
//							else
//								Save_Res.Set_Data.Nominal_V.Num-=900;
//							Uart_Send_Flag=2;
//							break;
//						case 12:
//							if(Save_Res.Set_Data.V_high.Num%1000<900)
//								Save_Res.Set_Data.V_high.Num+=100;
//							else
//								Save_Res.Set_Data.V_high.Num-=900;
//							Uart_Send_Flag=2;
//							
//							

//							break;
						default:
							break;
					
					
					}				
				

				break;
				case Key_F3:
					switch(Button_Page.index)
					{
						case 0:
							//if(Button_Page.page==0)
								SetSystemStatus(SYS_STATUS_SYSSET);
//							else
//								SetSystemStatus(SYS_STATUS_TOOL);
							break;
						case 1:
							break;
						case 2:
							Save_Res.Set_Data.speed=2;
							Send_Speed();
						break;
						case 3:
							Save_Res.Set_Data.dispvr=2;
							
								
							
							break;
						case 3+1:
							break;
						case 4+1:
//							if(Save_Res.Set_Data.Res_low.Num%100<90)
//								Save_Res.Set_Data.Res_low.Num+=10;
//							else
//								Save_Res.Set_Data.Res_low.Num-=90;
//							Uart_Send_Flag=2;
							break;
						case 5+1:
							break;
						case 6+1:
//							if(Save_Res.Set_Data.V_low.Num%100<90)
//								Save_Res.Set_Data.V_low.Num+=10;
//							else
//								Save_Res.Set_Data.V_low.Num-=90;
//							Uart_Send_Flag=2;
							
							break;
						case 7+1:
							if(Save_Res.Set_Data.Range < rangenum[Save_Res.version])
								Save_Res.Set_Data.Range++;
							Send_Range();
							break;
						case 8+1:
							Save_Res.Set_Data.beep=2;
							break;
						case 9+1:
							if(Save_Res.Set_Data.VRange < 3)
								Save_Res.Set_Data.VRange++;
							Send_Vrange();
							break;
//						case 9:
//							if(Save_Res.Set_Data.Nominal_Res.Num%100<90)
//								Save_Res.Set_Data.Nominal_Res.Num+=10;
//							else
//								Save_Res.Set_Data.Nominal_Res.Num-=90;
//							Uart_Send_Flag=2;
//							break;
//						case 10://MAX_R_RANGE
//							if(Save_Res.Set_Data.High_Res.Num%100<90)
//								Save_Res.Set_Data.High_Res.Num+=10;
//							else
//								Save_Res.Set_Data.High_Res.Num-=90;
//							Uart_Send_Flag=2;
//							break;
//						case 11:
//							if(Save_Res.Set_Data.Nominal_V.Num%100<90)
//								Save_Res.Set_Data.Nominal_V.Num+=10;
//							else
//								Save_Res.Set_Data.Nominal_V.Num-=90;
//							Uart_Send_Flag=2;
//							break;
//						case 12:
//							if(Save_Res.Set_Data.V_high.Num%100<90)
//								Save_Res.Set_Data.V_high.Num+=10;
//							else
//								Save_Res.Set_Data.V_high.Num-=90;
//							Uart_Send_Flag=2;
							
//							break;
						
						default:
							break;
					
					
					}	
					
				break;
				case Key_F4:
					switch(Button_Page.index)
					{
						case 0:
//							if(Button_Page.page==0)
								SetSystemStatus(SYS_STATUS_SYS);
							break;
						case 1:
							
									
							break;
						case 2:
							
							break;
						case 3+1:
							break;
						case 4+1:
//							if(Save_Res.Set_Data.Res_low.Num%10<9)
//								Save_Res.Set_Data.Res_low.Num+=1;
//							else
//								Save_Res.Set_Data.Res_low.Num-=9;
//							Uart_Send_Flag=2;
							break;
						case 5+1:
							break;
						case 6+1:
							if(Save_Res.Set_Data.V_low.Num%10<9)
								Save_Res.Set_Data.V_low.Num+=1;
							else
								Save_Res.Set_Data.V_low.Num-=9;
							Uart_Send_Flag=2;
							break;
						case 7+1:
							Save_Res.Set_Data.Range=3;
							Uart_Send_Flag=2;
							break;
						case 8+1:
							if(Save_Res.Set_Data.openbeep==0)
							{
								Save_Res.Set_Data.openbeep=1;
							}else{
								Save_Res.Set_Data.openbeep=0;
							}
							break;
//						case 8:
//							break;
//						case 9:
//							if(Save_Res.Set_Data.Nominal_Res.Num%10<9)
//								Save_Res.Set_Data.Nominal_Res.Num+=1;
//							else
//								Save_Res.Set_Data.Nominal_Res.Num-=9;
//							Uart_Send_Flag=2;
//							break;
//						case 10:
//							if(Save_Res.Set_Data.High_Res.Num%10<9)
//								Save_Res.Set_Data.High_Res.Num+=1;
//							else
//								Save_Res.Set_Data.High_Res.Num-=9;
//							Uart_Send_Flag=2;
//							break;
//						case 11:
//							if(Save_Res.Set_Data.Nominal_V.Num%10<9)
//								Save_Res.Set_Data.Nominal_V.Num+=1;
//							else
//								Save_Res.Set_Data.Nominal_V.Num-=9;
//							Uart_Send_Flag=2;
//							break;
//						case 12:
//							if(Save_Res.Set_Data.V_high.Num%10<9)
//								Save_Res.Set_Data.V_high.Num+=1;
//							else
//								Save_Res.Set_Data.V_high.Num-=9;
//							Uart_Send_Flag=2;
//							break;
						
						default:
							break;					
					}	
				
				break;
				case Key_F5:
					switch(Button_Page.index)
					{
//						case 0:
//						{
//							Save_Res.Set_Data.VRange=1;
//							Send_Vrange();
//						}break;
						case 4+1:
						case 9+1:
							break;
						case 7+1:	
//							Save_Res.Set_Data.Range=4;
//							Uart_Send_Flag=2;
						break;
						
						default:
							break;
					
					
					}
                    
					
				break;
				case Key_Disp:
					ComBuf.pageswflag=1;
					SetSystemStatus(SYS_STATUS_TEST);
				break;
				case Key_SETUP:
                        //SetSystemStatus(SYS_STATUS_SETUPTEST);
				break;
				case Key_FAST:
				break;
				case Key_LEFT:
					if(Button_Page.index==0)
						Button_Page.index=12;
					else
					if(Button_Page.index>7&&Button_Page.index < 11)
					{
						Button_Page.index-=7;
					}else if(Button_Page.index<4){
						Button_Page.index+=7;
					}else if(Button_Page.index >=4 && Button_Page.index <= 7){
						Button_Page.index+=7;
					}else if(Button_Page.index >=11){
						Button_Page.index-=7;
					}
						
				break;
				case Key_RIGHT:
					if(Button_Page.index==0)
						Button_Page.index=1;
					else
					if(Button_Page.index<=3)
						Button_Page.index+=7;
					else if(Button_Page.index>3 && Button_Page.index<=7)
						Button_Page.index+=7;
					else if(Button_Page.index>7 && Button_Page.index<=10)
						Button_Page.index-=7;
					else
						Button_Page.index-=7;
						
				break;
				case Key_DOWN:
					if(Button_Page.index>13)
						Button_Page.index=0;
					else
						Button_Page.index++;
					
					
				break;
				case Key_UP:
					if(Button_Page.index<1)
						Button_Page.index=14;
					else
						Button_Page.index--;
					
				break;
				case Key_DOT:
//					switch(Button_Page.index)
//					{
//						case 4:
//							if(Save_Res.Set_Data.Res_low.Dot>2)
//								Save_Res.Set_Data.Res_low.Dot=0;
//							else
//								Save_Res.Set_Data.Res_low.Dot++;
//							break;
//						case 6:
//							if(Save_Res.Set_Data.V_low.Dot>2)
//								Save_Res.Set_Data.V_low.Dot=0;
//							else
//								Save_Res.Set_Data.V_low.Dot++;
//							break;
//						case 9:
//							if(Save_Res.Set_Data.Nominal_Res.Dot>2)
//								Save_Res.Set_Data.Nominal_Res.Dot=0;
//							else
//								Save_Res.Set_Data.Nominal_Res.Dot++;
//							break;
//						case 10:
//							if(Save_Res.Set_Data.High_Res.Dot>2)
//								Save_Res.Set_Data.High_Res.Dot=0;
//							else
//								Save_Res.Set_Data.High_Res.Dot++;
//							break;
//						case 11:
//							if(Save_Res.Set_Data.Nominal_V.Dot>2)
//								Save_Res.Set_Data.Nominal_V.Dot=0;
//							else
//								Save_Res.Set_Data.Nominal_V.Dot++;
//							break;
//						case 12:
//							if(Save_Res.Set_Data.V_high.Dot>2)
//								Save_Res.Set_Data.V_high.Dot=0;
//							else
//								Save_Res.Set_Data.V_high.Dot++;
//							break;
//					
//					
//					}
					break;
				case Key_NUM1:
				//break;
				case Key_NUM2:
				//break;
				case Key_NUM3:
				//break;
				case Key_NUM4:
				//break;
				case Key_NUM5:
				//break;
				case Key_NUM6:
				//break;
				case Key_NUM7:
				//break;
				case Key_NUM8:
				//break;
				case Key_NUM9:
				//break;
				case Key_NUM0:
				//break;
				switch(Button_Page.index)
				{
					case 4+1:
						Coordinates.xpos=LIST1+88;
						Coordinates.ypos=FIRSTLINE+SPACE1*4;
						Coordinates.lenth=76+8;
						Save_Res.Set_Data.Res_low=Disp_Set_Num(&Coordinates);
						
						break;
					case 6+1:
						Coordinates.xpos=LIST1+88;
						Coordinates.ypos=FIRSTLINE+SPACE1*7;
						Coordinates.lenth=76;
						Save_Res.Set_Data.V_low=Disp_Set_NumV(&Coordinates);
						
						break;
					case 9+2:
						Coordinates.xpos=LIST2+88;
						Coordinates.ypos=FIRSTLINE+SPACE1*3;
						Coordinates.lenth=76+8;
						Save_Res.Set_Data.Nominal_Res=Disp_Set_Num(&Coordinates);
						break;
					case 10+2:
						Coordinates.xpos=LIST2+88;
						Coordinates.ypos=FIRSTLINE+SPACE1*4;
						Coordinates.lenth=76+8;
						Save_Res.Set_Data.High_Res=Disp_Set_Num(&Coordinates);
						break;
					case 11+2:
						Coordinates.xpos=LIST2+88;
						Coordinates.ypos=FIRSTLINE+SPACE1*6;
						Coordinates.lenth=76;
						Save_Res.Set_Data.Nominal_V=Disp_Set_NumV(&Coordinates);
						break;
					case 12+2:
						Coordinates.xpos=LIST2+88;
						Coordinates.ypos=FIRSTLINE+SPACE1*7;
						Coordinates.lenth=76;
						Save_Res.Set_Data.V_high=Disp_Set_NumV(&Coordinates);
						break;
					default:
						break;
				
				
				}
																							
					break;
				case Key_BACK:
				break;
				case Key_LOCK:
				break;
				case Key_BIAS:
				break;
				case Key_REST:
				break;
				case Key_TRIG:
				break;
				default:
				break;
					
			}
		
		
		}
	 	
	
	
	
	}
	Savetoeeprom();
}
//数据保存程序
void Data_StoreProcess(void)
{
	vu32 keynum=0;
	vu8 key;
//	Button_Page_Typedef Button_Page;
//	Button_Page.index=0;
//	Button_Page.page=0;
	Delay_Key();
 	while(GetSystemStatus()==SYS_STATUS_DATASTORE)
	{
	 key=HW_KeyScsn();
		if(key==0xff)
		{
			keynum=0;
		}
		else
			keynum++;
		if(keynum==2)
		{
            Key_Beep();
			switch(key)
			{
				case Key_F1:
				break;
				case Key_F2:
				break;
				case Key_F3:
				break;
				case Key_F4:
				break;
				case Key_F5:
				break;
				case Key_Disp:
				break;
				case Key_SETUP:
				break;
				case Key_FAST:
				break;
				case Key_LEFT:
				break;
				case Key_RIGHT:
				break;
				case Key_UP:
				break;
				case Key_DOWN:
				break;
				case Key_NUM1:
				break;
				case Key_NUM2:
				break;
				case Key_NUM3:
				break;
				case Key_NUM4:
				break;
				case Key_NUM5:
				break;
				case Key_NUM6:
				break;
				case Key_NUM7:
				break;
				case Key_NUM8:
				break;
				case Key_NUM9:
				break;
				case Key_NUM0:
				break;
				case Key_DOT:
				break;
				case Key_BACK:
				break;
				case Key_LOCK:
				break;
				case Key_BIAS:
				break;
				case Key_REST:
				break;
				case Key_TRIG:
				break;
				default:
				break;
					
			}
		
		
		}
	 	
	
	}
}
//档号显示
void Range_Process(void)
{
	
	vu32 keynum=0;
	vu8 key;
	Send_Ord_Typedef Uart;
//    vu8 page=1;
	vu8 Disp_flag=1;
//	vu8 index=0;
	Button_Page_Typedef Button_Page;
	Button_Page.index=0;
	Button_Page.page=0;
	Disp_Range_Item();
	Delay_Key();
 	while(GetSystemStatus()==SYS_STATUS_RANGE)
	{
		if(Disp_flag==1)
		{
			Disp_RangeDispValue(&Button_Page);
			Disp_flag=0;	
		}
		Uart_Process();
		
		if(timer0_counter>0)//请求数据
		{
			switch(Uart_Send_Flag)
			{
				case 0:
					Send_Request();
					break;
				case 1:
					Send_Main_Ord();
					break;
				case 2:
					Send_Freq(&Uart);
					break;
				default:
					Send_Request();
					break;
			
			}
			Uart_Send_Flag=0;
			
			timer0_counter=0;
		
		
		}
		if(SaveData.Limit_Tab.Comp)
			Test_Comp(&Comp_Change);
		Disp_RangeTestvalue();
	 	key=HW_KeyScsn();
		if(key==0xff)
		{
			keynum=0;
		}
		else
			keynum++;
		if(keynum==KEY_NUM)
		{
            Key_Beep();
			Disp_flag=1;
			switch(key)
			{
				case Key_F1:
					if(Button_Page.index==0)
					{
						if(Button_Page.page==0)
							SetSystemStatus(SYS_STATUS_TEST);
					}
					else
						SaveData.Limit_Tab.Comp=0;
				break;
				case Key_F2:
					if(Button_Page.index==0)
					{
						if(Button_Page.page==0)
							SetSystemStatus(SYS_STATUS_RANGE);
					}
					else
						SaveData.Limit_Tab.Comp=1;
				break;
				case Key_F3:
					if(Button_Page.index==0)
					{
						if(Button_Page.page==0)
							SetSystemStatus(SYS_STATUS_RANGECOUNT);
					}
				break;
				case Key_F4:
//					if(Button_Page.index==0)
//					{
//						if(Button_Page.page==0)
//							SetSystemStatus(SYS_STATUS_ITEM);
//					}
				break;
				case Key_F5:
//					if(Button_Page.index==0)
//					{
//						if(Button_Page.page==1)
//							Button_Page.page=0;
//						else
//							Button_Page.page=1;
//						Disp_Button_value1(Button_Page.page);
//					}
				break;
				case Key_Disp:
                    SetSystemStatus(SYS_STATUS_TEST);
				break;
				case Key_SETUP:
                    SetSystemStatus(SYS_STATUS_SETUPTEST);
				break;
				case Key_FAST:
					
//				break;
				case Key_LEFT:
//				break;
				case Key_RIGHT:
//				break;
				case Key_UP:
//				break;
				case Key_DOWN:
					if(Button_Page.index)
						Button_Page.index=0;
					else
						Button_Page.index=1;
				break;
				case Key_NUM1:
				break;
				case Key_NUM2:
				break;
				case Key_NUM3:
				break;
				case Key_NUM4:
				break;
				case Key_NUM5:
				break;
				case Key_NUM6:
				break;
				case Key_NUM7:
				break;
				case Key_NUM8:
				break;
				case Key_NUM9:
				break;
				case Key_NUM0:
				break;
				case Key_DOT:
				break;
				case Key_BACK:
				break;
				case Key_LOCK:
				break;
				case Key_BIAS:
				break;
				case Key_REST:
				break;
				case Key_TRIG:
				break;
				default:
				break;
					
			}
		
		
		}
	
	
	}
}
//档计数显示
void Range_CountProcess(void)
{
   	 vu32 keynum=0;
	vu32 uart_count=0;
	 vu8 key;
	u8 i;
	vu8 Disp_flag=1;
//	 vu8 page=1;
	Button_Page_Typedef Button_Page;
	Button_Page.index=0;
	Button_Page.page=0;
	Disp_Range_Count_Item();
	Delay_Key();
	Set_Compbcd_float();
	for(i=0;i<=10;i++)
		Count_buff[i]=0;
 	while(GetSystemStatus()==SYS_STATUS_RANGECOUNT)
	{
		key=HW_KeyScsn();
		if(Disp_flag==1)
		{
			Disp_Range_ComDispValue(&Button_Page);
			Disp_flag=0;
			
		}
		uart_count=Uart_Process();
		if(timer0_counter>0)//请求数据
		{
			switch(Uart_Send_Flag)
			{
				case 0:
					Send_Request();
					break;
				case 1:
					Send_Main_Ord();
					break;
				case 2:
					//Send_Freq(&Uart);
					break;
				default:
					Send_Request();
					break;
			
			}
			Uart_Send_Flag=0;
			
			timer0_counter=0;
		
		
		}
		if(SaveData.Limit_Tab.Comp&&uart_count==1)
			Test_Comp(&Comp_Change);
		//Disp_RangeTestvalue();
		if(SaveData.Limit_Tab.Param)//附属比较
			;
		Disp_RangeCount();//增加分选结果计数
		if(key==0xff)
		{
			keynum=0;
		}
		else
			keynum++;
		if(keynum==KEY_NUM)
		{	Disp_flag=1;
            Key_Beep();
			switch(key)
			{
				case Key_F1:
                    if(Button_Page.index==0)
					{
						if(Button_Page.page==0)
							SetSystemStatus(SYS_STATUS_TEST);
					}
					else
						SaveData.Limit_Tab.count=0;
				break;
				case Key_F2:
					if(Button_Page.index==0)
					{
                    if(Button_Page.page==0)
                        SetSystemStatus(SYS_STATUS_RANGE);
					}
					else
						SaveData.Limit_Tab.count=1;
				break;
				case Key_F3:
					if(Button_Page.index==0)
                    if(Button_Page.page==0)
                        SetSystemStatus(SYS_STATUS_RANGECOUNT);
				break;
				case Key_F4:
//					if(Button_Page.index==0)
//                    if(Button_Page.page==0)
//                        SetSystemStatus(SYS_STATUS_ITEM);
				break;
				case Key_F5:
//                    if(Button_Page.page==1)
//						Button_Page.page=0;
//					else
//						Button_Page.page=1;
//                    Disp_Button_value1(Button_Page.page);
				break;
				case Key_Disp:
                    SetSystemStatus(SYS_STATUS_TEST);
				break;
				case Key_SETUP:
                    SetSystemStatus(SYS_STATUS_SETUPTEST);
				break;
				case Key_FAST:
				break;
				case Key_LEFT:
				//break;
				case Key_RIGHT:
				//break;
				case Key_UP:
				//break;
				case Key_DOWN:
					if(Button_Page.index==0)
						Button_Page.index=1;
					else
						Button_Page.index=0;
				break;
				case Key_NUM1:
				break;
				case Key_NUM2:
				break;
				case Key_NUM3:
				break;
				case Key_NUM4:
				break;
				case Key_NUM5:
				break;
				case Key_NUM6:
				break;
				case Key_NUM7:
				break;
				case Key_NUM8:
				break;
				case Key_NUM9:
				break;
				case Key_NUM0:
				break;
				case Key_DOT:
				break;
				case Key_BACK:
				break;
				case Key_LOCK:
				break;
				case Key_BIAS:
				break;
				case Key_REST:
				break;
				case Key_TRIG:
				break;
				default:
				break;
					
			}
		
		
		}
	
	
	}
}
// 列表显示
void ItemProcess(void)
{
	
	vu32 keynum;
	vu8 key;
	vu8 Disp_flag=1;
//    vu8 page=1;
	
	Button_Page_Typedef Button_Page;
	SaveData.Limit_ScanValue.num=0;
	Button_Page.index=0;
	Button_Page.page=0;
	Disp_List_Count_Item();
	Delay_Key();
 	while(GetSystemStatus()==SYS_STATUS_ITEM)
	{
		if(Disp_flag)
		{
			Disp_flag=0;
			Disp_LIMIT_ComDispValue(&Button_Page);
		}
		
		Disp_Scan_Compvalue(0);
		key=HW_KeyScsn();
		if(key==0xff)
		{
			keynum=0;
		}
		else
			keynum++;
		if(keynum==2)
		{
			Disp_flag=1;
            Key_Beep();
			switch(key)
			{
				case Key_F1:
                    if(Button_Page.index==0)
					{
						if(Button_Page.page==0)
							SetSystemStatus(SYS_STATUS_TEST);
					}
					else if(Button_Page.index==1)
						SaveData.Limit_ScanValue.fun=0;
					
				break;
				case Key_F2:
					if(Button_Page.index==0)
					{
						if(Button_Page.page==0)
							SetSystemStatus(SYS_STATUS_RANGE);
					
					}else if(Button_Page.index==1)
						SaveData.Limit_ScanValue.fun=1;
					
				break;
				case Key_F3:
					if(Button_Page.index==0)
					{
						if(Button_Page.page==0)
							SetSystemStatus(SYS_STATUS_RANGECOUNT);
					
					}
				break;
				case Key_F4:
					if(Button_Page.index==0)
					{
						if(Button_Page.page==0)
							SetSystemStatus(SYS_STATUS_ITEM);
					}
					else if(Button_Page.index==2)
					{
						if(SaveData.Limit_ScanValue.num<1)
							SaveData.Limit_ScanValue.num=20;
						else
							SaveData.Limit_ScanValue.num--;
					
					}
				break;
				case Key_F5:
					if(Button_Page.index==0)
					{
						if(Button_Page.page==0)
							Button_Page.page=1;
						else
							Button_Page.page=0;
						Disp_Button_value1(Button_Page.page);
					}
					else if(Button_Page.index==2)
					{
						if(SaveData.Limit_ScanValue.num>=20)
							SaveData.Limit_ScanValue.num=0;
						else
							SaveData.Limit_ScanValue.num++;
					
					}
				break;
				case Key_Disp:
                    SetSystemStatus(SYS_STATUS_TEST);
				break;
				case Key_SETUP:
                    SetSystemStatus(SYS_STATUS_SETUPTEST);
				break;
				case Key_FAST:
				break;
				case Key_LEFT:
				break;
				case Key_RIGHT:
				break;
				case Key_UP:
					if(Button_Page.index>0)
						Button_Page.index--;
				break;
				case Key_DOWN:
					if(Button_Page.index<2)
						Button_Page.index++;
				break;
				case Key_NUM1:
				break;
				case Key_NUM2:
				break;
				case Key_NUM3:
				break;
				case Key_NUM4:
				break;
				case Key_NUM5:
				break;
				case Key_NUM6:
				break;
				case Key_NUM7:
				break;
				case Key_NUM8:
				break;
				case Key_NUM9:
				break;
				case Key_NUM0:
				break;
				case Key_DOT:
				break;
				case Key_BACK:
				break;
				case Key_LOCK:
				break;
				case Key_BIAS:
				break;
				case Key_REST:
				break;
				case Key_TRIG:
				break;
				default:
				break;
					
			}
		
		
		}
	
	
	}
}
//列表扫描设置
void Use_ITEMSetProcess(void)
{
	Disp_Coordinates_Typedef  Coordinates;
	vu32 keynum=0;
	vu8 key;
//    vu8 page=0;
	vu8 Disp_flag=1;
	Button_Page_Typedef Button_Page;
	
	Button_Page.index=0;
	Button_Page.page=0;
	Button_Page.third=0;
	Button_Page.force=0;
	SaveData.Limit_ScanValue.num=0;
	Disp_ListScan_Item();
	Delay_Key();
 	while(GetSystemStatus()==SYS_STATUS_ITEMSET)
	{
	  	
		if(Disp_flag)
		{
			Disp_flag=0;
		
			Disp_Scan_SetCompvalue(&Button_Page);
		}
		key=HW_KeyScsn();
		if(key==0xff)
		{
			keynum=0;
		}
		else
			keynum++;
		if(keynum==KEY_NUM)
		{
			Disp_flag=1;
            Key_Beep();
			switch(key)
			{
				case Key_F1:
					if(Button_Page.index==0)
					{
						if(Button_Page.page==0)
							SetSystemStatus(SYS_STATUS_SETUPTEST);
						else
							SetSystemStatus(SYS_STATUS_FILE);
					}
						
				break;
				case Key_F2:
					if(Button_Page.index==0)
					{
						if(Button_Page.page==0)
							SetSystemStatus(SYS_STATUS_USERDEBUG);
						else
							SetSystemStatus(SYS_STATUS_SYSSET);
					}
				break;
				case Key_F3://
					if(Button_Page.index==0)
					{
						if(Button_Page.page==0)
							SetSystemStatus(SYS_STATUS_LIMITSET);
						else
							SetSystemStatus(SYS_STATUS_TOOL);
					}
				break;
				case Key_F4:
					if(Button_Page.index==0)
					{
						if(Button_Page.page==0)
							SetSystemStatus(SYS_STATUS_ITEMSET);
					}else 
					if(Button_Page.index>1)
					{
						if(Button_Page.force>0)
							Button_Page.force--;
					
					}
					
				break;
				case Key_F5:
					if(Button_Page.index==0)
					{
						if(Button_Page.page)
							Button_Page.page=0;
						else
							Button_Page.page=1;
						Disp_Button_TestSet(Button_Page.page);
					}else
					if(Button_Page.force<20)
						Button_Page.force++;
				break;
				case Key_Disp:
                    SetSystemStatus(SYS_STATUS_TEST);
				break;
				case Key_SETUP:
                    SetSystemStatus(SYS_STATUS_SETUPTEST);
				break;
				case Key_FAST:
				break;
				case Key_LEFT:
					if(Button_Page.third>0)
						Button_Page.third--;
						
					
				break;
				case Key_RIGHT:
					if(Button_Page.third<5)
						Button_Page.third++;
					else
						Button_Page.third=5;
				break;
				case Key_UP:
					if(Button_Page.index>0)
						Button_Page.index--;
					else
						Button_Page.page=0;
						
				break;
				case Key_DOWN:
					if(Button_Page.index<12)
						Button_Page.index++;
					else
						Button_Page.index=12;
				break;
				case Key_NUM1:
				//break;
				case Key_NUM2:
				//break;
				case Key_NUM3:
				//break;
				case Key_NUM4:
				//break;
				case Key_NUM5:
				//break;
				case Key_NUM6:
				//break;
				case Key_NUM7:
				//break;
				case Key_NUM8:
				//break;
				case Key_NUM9:
				//break;
				case Key_NUM0:
				//break;
				case Key_DOT:
				//break;
				
					if(Button_Page.index>2)
					{
						if(Button_Page.third==0)
						{
							Coordinates.xpos=40;//FIRSTLINE+SPACE1+3+ (i-2)*16
							Coordinates.ypos=FIRSTLINE+SPACE1+3+(Button_Page.index-2)*16;
							
							SaveData.Limit_ScanValue.freq[Button_Page.force*10+
							(Button_Page.index-3)]=Freq_Set_Num(&Coordinates);
						}else 
						if(Button_Page.third==1)
						{
						
						}else
						if(Button_Page.third==2)
						{
						
						}else
						if(Button_Page.third==3)
						{
						
						}else
						if(Button_Page.third==4)
						{
							Coordinates.xpos=420;
							Coordinates.ypos=FIRSTLINE+SPACE1+3+(Button_Page.index-2)*16;
							Coordinates.lenth=60;
							SaveData.Limit_ScanValue.time[Button_Page.force*10+
							(Button_Page.index-3)]=Disp_Set_Num(&Coordinates);
						
						
						}
					
					}
				case Key_BACK:
				break;
				case Key_LOCK:
				break;
				case Key_BIAS:
				break;
				case Key_REST:
				break;
				case Key_TRIG:
				break;
				default:
				break;
					
			}
		
		
		}
	
	
	}
}
//极限设置 
void Use_LimitSetProcess(void)
{
	
	vu32 keynum=0;
	vu8 key,i;
	Disp_Coordinates_Typedef Coordinates;
//    vu8 page=1;
	vu8 Disp_flag=1;
	Button_Page_Typedef Button_Page;
	Button_Page.index=0;
	Button_Page.page=0;
	Disp_LimitList_Item();
	Delay_Key();
 	while(GetSystemStatus()==SYS_STATUS_LIMITSET)
	{
	  	if(Disp_flag)
		{
			Disp_flag=0;
			Disp_LimitSEt_value(& Button_Page);
		}
		key=HW_KeyScsn();
		if(key==0xff)
		{
			keynum=0;
		}
		else
			keynum++;
		if(keynum==KEY_NUM)
		{
			Disp_flag=1;
            Key_Beep();
			switch(key)
			{
				case Key_F1:
					if(Button_Page.index==0)
					{
						if(Button_Page.page==0)
							SetSystemStatus(SYS_STATUS_SETUPTEST);
						else
							SetSystemStatus(SYS_STATUS_FILE);
					
					}else if(Button_Page.index==1)
					;
					else if(Button_Page.index==2)
						;
					else if(Button_Page.index==3)
					{
						SaveData.Limit_Tab.Mode=0;
					
					}
					else if(Button_Page.index==4)
					{
						SaveData.Limit_Tab.Param=0;
					}
					else if(Button_Page.index==5)
					{
						SaveData.Limit_Tab.Comp=0;
					}else
					{
						if(Button_Page.index>15)
						{
						SaveData.Limit_Tab.Comp_Value[Button_Page.index-16].high.Num=0;
						SaveData.Limit_Tab.Comp_Value[Button_Page.index-16].low.Num=0;	
						}else if(Button_Page.index>5)
						{
						SaveData.Limit_Tab.Comp_Value[Button_Page.index-6].high.Num=0;
						SaveData.Limit_Tab.Comp_Value[Button_Page.index-6].low.Num=0;	
						
						}
					
					
					}
						
				break;
				case Key_F2:
					if(Button_Page.index==0)
					{
						if(Button_Page.page==0)
							SetSystemStatus(SYS_STATUS_USERDEBUG);
						else
							SetSystemStatus(SYS_STATUS_SYSSET);
					}
					else if(Button_Page.index==1)
					;
					else if(Button_Page.index==2)
						;
					else if(Button_Page.index==3)
					{
						SaveData.Limit_Tab.Mode=1;
					
					}
					else if(Button_Page.index==4)
					{
						SaveData.Limit_Tab.Param=1;
					}
					else if(Button_Page.index==5)
					{
						SaveData.Limit_Tab.Comp=1;
					}
				break;
				case Key_F3:
					if(Button_Page.index==0)
					{
						if(Button_Page.page==0)
							SetSystemStatus(SYS_STATUS_LIMITSET);
						else
							SetSystemStatus(SYS_STATUS_TOOL);
					}
				break;
				case Key_F4:
//					if(Button_Page.index==0)
//					{
//						if(Button_Page.page==0)
//							SetSystemStatus(SYS_STATUS_ITEMSET);
//					}
					
				break;
				case Key_F5:
					if(Button_Page.index==0)
					{
//						if(Button_Page.page)
//							Button_Page.page=0;
//						else
//						Button_Page.page=1;
//						Disp_Button_TestSet(Button_Page.page);
					}else if(Button_Page.index>5)
					{
						for(i=0;i<10;i++)
						{
							SaveData.Limit_Tab.Comp_Value[i].low.Num=0;
							SaveData.Limit_Tab.Comp_Value[i].high.Num=0;
							
						}
						Button_Page.index=0;
					
					
					}
				break;
				case Key_Disp:
                    SetSystemStatus(SYS_STATUS_TEST);
				break;
				case Key_SETUP:
                    SetSystemStatus(SYS_STATUS_SETUPTEST);
				break;
				case Key_FAST:
				break;
				case Key_LEFT:
					if(Button_Page.index!=0)
					{
						if(Button_Page.index<=5)
						{
							Button_Page.index--;
						
						}
						else
						{
							if(Button_Page.index>15)
								Button_Page.index-=10;
							else
								Button_Page.index+=9;
						}
					
					
					}
					
				break;
				case Key_RIGHT:
					if(Button_Page.index!=0)
					{
//						if(Button_Page.index<=5)
//						{
//							Button_Page.index++;
//						
//						}
//						else
						//if(Button_Page.index)
						{
							if(Button_Page.index>15)
								Button_Page.index-=9;
							else
								//if(Button_Page.index<)
								Button_Page.index+=10;
						}
					
					
					}
				break;
				case Key_UP:
					if(Button_Page.index>16)
						Button_Page.index--;
					else if(Button_Page.index==16)
						Button_Page.index=5;
					else
						
					if(Button_Page.index>0)
						Button_Page.index--;
				break;
				case Key_DOWN:
					if(Button_Page.index<15)
						Button_Page.index++;
					else if(Button_Page.index==15)
						Button_Page.index=0;
					else
						
					if(Button_Page.index<25)
						Button_Page.index++;
					else
						Button_Page.index=0;
						
				break;
				case Key_NUM1:
//				break;
				case Key_NUM2:
//				break;
				case Key_NUM3:
//				break;
				case Key_NUM4:
//				break;
				case Key_NUM5:
//				break;
				case Key_NUM6:
//				break;
				case Key_NUM7:
//				break;
				case Key_NUM8:
//				break;
				case Key_NUM9:
//				break;
				case Key_NUM0:
//				break;
				case Key_DOT:
					if(Button_Page.index==2)//LIST2-24, FIRSTLINE,
					{
							Coordinates.xpos=LIST2-24;//FIRSTLINE+SPACE1+3+ (i-2)*16
							Coordinates.ypos=FIRSTLINE;
							Coordinates.lenth=66;
							SaveData.Limit_Tab.Nom=Disp_Set_InputNum(&Coordinates);
					}else if(Button_Page.index>5&&Button_Page.index<=15)
						{
							//LIST2-90, 76+(i-6)*15
							Coordinates.xpos=LIST2-90;//FIRSTLINE+SPACE1+3+ (i-2)*16
							Coordinates.ypos=76+(Button_Page.index-6)*15;
							Coordinates.lenth=60;
							SaveData.Limit_Tab.Comp_Value[Button_Page.index-6].low=Disp_Set_InputpreNum(&Coordinates);
							SaveData.Limit_Tab.Comp_Value[Button_Page.index-6].high=SaveData.Limit_Tab.Comp_Value[Button_Page.index-6].low;
							//Sort_TypeDef Disp_Set_InputpreNum(Disp_Coordinates_Typedef *Coordinates)
						}else if(Button_Page.index>15)
						{
							Coordinates.xpos=LIST2+70;//FIRSTLINE+SPACE1+3+ (i-2)*16
							Coordinates.ypos=76+(Button_Page.index-16)*15;
							Coordinates.lenth=60;
							SaveData.Limit_Tab.Comp_Value[Button_Page.index-16].high=Disp_Set_InputpreNum(&Coordinates);
						
						}
						do{
							key=HW_KeyScsn();
						}
						while(key!=0xff);
						
				break;
				case Key_BACK:
				break;
				case Key_LOCK:
				break;
				case Key_BIAS:
				break;
				case Key_REST:
				break;
				case Key_TRIG:
				break;
				default:
				break;
					
			}
		
		
		}
	
	
	}
	Set_Compbcd_float();
	Savetoeeprom();
}
void Sys_Process(void)
{
	vu32 keynum=0;
	vu8 key;
	vu8 Disp_flag=1;
	Button_Page_Typedef Button_Page;
	Button_Page.index=0;
	Button_Page.page=0;
//	Disp_Sys_Screen();
//	Colour.Fword=White;
//	Colour.black=LCD_COLOR_TEST_BAR;
//	WriteString_16(0, 4, All_TopName[8],  0);
//	Disp_Sys_Item();
    lcd_Clear(LCD_COLOR_TEST_BACK);
	Delay_Key();
	while(GetSystemStatus()==SYS_STATUS_SYS)
	{
		if(Disp_flag==1)
		{
			Disp_Sys();
			Disp_flag=0;	
		}
		key=HW_KeyScsn();
		if(key==0xff)
		{
			keynum=0;
		}
		else
			keynum++;
		if(keynum==KEY_NUM)
		{
			Disp_flag=1;
            Key_Beep();
			switch(key)
			{
				case Key_F1:
					switch(Button_Page.index)
					{
						case 0:
							ComBuf.pageswflag=1;
							SetSystemStatus(SYS_STATUS_TEST);
							break;
						
						default:
							break;
					
					}
                    
				break;
				case Key_F2:
					switch(Button_Page.index)
					{
						case 0:
							SetSystemStatus(SYS_STATUS_SETUPTEST);
							break;
						
					
					}
                    
				break;
				case Key_F3:
					switch(Button_Page.index)
					{
						case 0:
							SetSystemStatus(SYS_STATUS_SYSSET);
							break;
						
						default:
						break;
					
					}
						
				break;
				case Key_F4:
					switch(Button_Page.index)
					{
						case 0:
							SetSystemStatus(SYS_STATUS_SYS);
							break;
						
							
						
						default:
						break;
					
					}
				break;
				
				
				case Key_Disp:
					ComBuf.pageswflag=1;
					SetSystemStatus(SYS_STATUS_TEST);
				break;
				case Key_SETUP:
                    SetSystemStatus(SYS_STATUS_SETUPTEST);
				break;
				
				
				default:
				break;
					
			}
		
		
		}
	
	
	
	
	}

}
//软件关机程序，放在工具状态里面
void Soft_Turnon(void)
{
	uint32_t i;
	Set_Compbcd_float();
    
//	ReadSavedata();
//	SetData_High_Low_Comp();
//	Savetoeeprom();
	
	Power_On_led();

	All_LedOff();
	Beep_Off();
    GPIO_ClearInt(0, 1<<19);
	while(GetSystemStatus()==SYS_STATUS_TOOL)
	{
		
        for(i=0xfffff;i>0;i--)
		;
		NVIC_EnableIRQ(GPIO_IRQn);
		
//		if(Save_Res.softswitch)
//		{
//			SetSystemStatus(SYS_STATUS_POWER);
//		
//		
//		}
	
	
	}
    NVIC_DisableIRQ(GPIO_IRQn);
    GPIO_ClearInt(0, 1<<19);
	Power_Off_led();

}


//各个档位相位角修正上限
void RoffsetComp(void)
{
	switch(Test_Dispvalue.Rangedisp)
	{
		case 1:
		case 4:
		{
			if(Save_Res.Roffset[0].Dot < 4)
			{
				Save_Res.Roffset[0].Dot = 4;
				Save_Res.Roffset[0].Num = 30000; 
			}else{
				if(Save_Res.Roffset[0].Num > 30000)
				{
					Save_Res.Roffset[0].Num = 30000; 
				}
			}
		}break;
		case 2:
		case 5:
		{
			if(Save_Res.Roffset[0].Dot < 3)
			{
				Save_Res.Roffset[0].Dot = 3;
				Save_Res.Roffset[0].Num = 30000; 
			}else{
				if(Save_Res.Roffset[0].Num > 30000)
				{
					Save_Res.Roffset[0].Num = 30000; 
				}
			}
		}break;
		case 3:
		case 6:
		{
			if(Save_Res.Roffset[0].Dot < 2)
			{
				Save_Res.Roffset[0].Dot = 2;
				Save_Res.Roffset[0].Num = 30000; 
			}else{
				if(Save_Res.Roffset[0].Num > 30000)
				{
					Save_Res.Roffset[0].Num = 30000; 
				}
			}
		}break;
		case 7:
		{
			if(Save_Res.Roffset[0].Dot < 1)
			{
				Save_Res.Roffset[0].Dot = 1;
				Save_Res.Roffset[0].Num = 30000; 
			}else{
				if(Save_Res.Roffset[0].Num > 30000)
				{
					Save_Res.Roffset[0].Num = 30000; 
				}
			}
		}break;
		default:break;
	}
}

//相位角修正值全部复位
void RoffsetAllReset(void)
{
	u8 i;
	for(i=0;i<7;i++)
	{
		Save_Res.Roffset[i].sign = 0;
		Save_Res.Roffset[i].Num = 0;
	}
}

//相位角修正值单复位
void RoffsetSingleReset(void)
{
	u8 i;
	if(Save_Res.Set_Data.Range == 0)
	{
		Save_Res.Roffset[Test_Dispvalue.Rangedisp-1].Num = 0;
		Save_Res.Roffset[Test_Dispvalue.Rangedisp-1].sign = 0;
	}else{
		Save_Res.Roffset[Save_Res.Set_Data.Range-1].Num = 0;
		Save_Res.Roffset[Save_Res.Set_Data.Range-1].sign = 0;
	}
}

//系统设置
void Use_SysSetProcess(void)
{	
	Disp_Coordinates_Typedef  Coordinates;
	Sort_TypeDef Inputbuf;
	vu32 keynum=0;
	vu8 key,i;
//    uint8_t Disp_buff[12];
    char key_count=0;
	vu8 Disp_flag=1;
	Button_Page_Typedef Button_Page;
	Button_Page.index=0;
	Button_Page.page=0;
    lcd_Clear(LCD_COLOR_TEST_BACK);
	Disp_Sys_Item();
	Delay_Key();
 	while(GetSystemStatus()==SYS_STATUS_SYSSET)
	{
	  	
		if(Disp_flag==1||Rtc_intflag==1)
		{
            Rtc_intflag=0;
            if(key_count==1)
            {
                for(i=1;i<8;i++)
                Save_Res.Sys_Setvalue.textname[i]=0;
            
            }
            //Save_Res.Sys_Setvalue.textname[8]='/0';
			Disp_Sys_value(&Button_Page);
			Disp_flag=0;
            
		}
		key=HW_KeyScsn();
		if(key==0xff)
		{
			keynum=0;
		}
		else
			keynum++;
		if(keynum==KEY_NUM)
		{
			Disp_flag=1;
            Key_Beep();
			switch(key)
			{
				case Key_F1:
					switch(Button_Page.index)
					{
						case 0:
							ComBuf.pageswflag=1;
							SetSystemStatus(SYS_STATUS_TEST);
							break;
						case 1:
							Save_Res.Sys_Setvalue.uart=0;
							break;
						case 2:
//							Save_Res.Sys_Setvalue.buard=0;
							break;
						case 3:
                            Save_Res.Sys_Setvalue.u_flag=0;
							
							break;
						case 4:
							Save_Res.clearsw=0;
							break;
						case 5:
							Save_Res.Sys_Setvalue.lanage=0;
							break;
						case 6:
							if(RTC_TIME_DISP.YEAR<1)
								RTC_TIME_DISP.YEAR=RTC_YEAR_MAX;
							else
                                if(RTC_TIME_DISP.YEAR>=RTC_YEAR_MAX)
                                    RTC_TIME_DISP.YEAR=0;
                                else
								RTC_TIME_DISP.YEAR--;
                            RTC_SetTime (LPC_RTC, RTC_TIMETYPE_YEAR, RTC_TIME_DISP.YEAR);
                            
                            RTC_CalibConfig(LPC_RTC, 0, RTC_CALIB_DIR_FORWARD);
                            RTC_CalibCounterCmd(LPC_RTC, ENABLE);
							break;
						case 7:
							if(RTC_TIME_DISP.MONTH<=1)
								RTC_TIME_DISP.MONTH=RTC_MONTH_MAX;
							else
								RTC_TIME_DISP.MONTH--;
                            RTC_SetTime (LPC_RTC, RTC_TIMETYPE_MONTH, RTC_TIME_DISP.MONTH);
                            
                            RTC_CalibConfig(LPC_RTC, 0, RTC_CALIB_DIR_FORWARD);
                            RTC_CalibCounterCmd(LPC_RTC, ENABLE);
							break;
						case 8:
							if(RTC_TIME_DISP.DOM<=1)
								RTC_TIME_DISP.DOM=RTC_DAYOFMONTH_MAX;
							else
								RTC_TIME_DISP.DOM--;
                            RTC_SetTime (LPC_RTC, RTC_TIMETYPE_DAYOFMONTH, RTC_TIME_DISP.DOM);
                            
                            RTC_CalibConfig(LPC_RTC, 0, RTC_CALIB_DIR_FORWARD);
                            RTC_CalibCounterCmd(LPC_RTC, ENABLE);
							break;
						case 9:
							if(RTC_TIME_DISP.HOUR<1)
								RTC_TIME_DISP.HOUR=RTC_HOUR_MAX;
							else
								RTC_TIME_DISP.HOUR--;
                             RTC_SetTime (LPC_RTC, RTC_TIMETYPE_HOUR, RTC_TIME_DISP.HOUR);
                            
                            RTC_CalibConfig(LPC_RTC, 0, RTC_CALIB_DIR_FORWARD);
                            RTC_CalibCounterCmd(LPC_RTC, ENABLE);
							break;
						case 10:
							if(RTC_TIME_DISP.MIN<1)
								RTC_TIME_DISP.MIN=RTC_MINUTE_MAX;
							else
								RTC_TIME_DISP.MIN--;
                            RTC_SetTime (LPC_RTC, RTC_TIMETYPE_MINUTE, RTC_TIME_DISP.MIN);
                            
                            RTC_CalibConfig(LPC_RTC, 0, RTC_CALIB_DIR_FORWARD);
                            RTC_CalibCounterCmd(LPC_RTC, ENABLE);
							break;
						case 11:
							if(RTC_TIME_DISP.SEC<1)
								RTC_TIME_DISP.SEC=RTC_SECOND_MAX;
							else
								RTC_TIME_DISP.SEC--;
                            RTC_SetTime (LPC_RTC, RTC_TIMETYPE_SECOND, RTC_TIME_DISP.SEC);
                            
                            RTC_CalibConfig(LPC_RTC, 2, RTC_CALIB_DIR_FORWARD);
                            RTC_CalibCounterCmd(LPC_RTC, ENABLE);
							break;
						case 12://
							//SaveData.Sys_Setup.Bias=0;
							break;
						case 18:
							SaveData.Sys_Setup.Bus_Mode=0;
							break;
						case 13:
							if(SaveData.Sys_Setup.GP_Addr<1)
								SaveData.Sys_Setup.GP_Addr=99;
							else
								SaveData.Sys_Setup.GP_Addr--;
							break;
						case 14:
							SaveData.Sys_Setup.Talk_Only=1;
							break;
						case 15:
							if(SaveData.Sys_Setup.Timer_Value.Hour<1)
								SaveData.Sys_Setup.Timer_Value.Hour=12;
							else
								SaveData.Sys_Setup.Timer_Value.Hour--;
							break;
						case 16:
							if(SaveData.Sys_Setup.Timer_Value.Min<1)
								SaveData.Sys_Setup.Timer_Value.Min=59;
							else
								SaveData.Sys_Setup.Timer_Value.Min--;
							break;
						case 17:
							if(SaveData.Sys_Setup.Timer_Value.Sec<1)
								SaveData.Sys_Setup.Timer_Value.Sec=59;
							else
								SaveData.Sys_Setup.Timer_Value.Sec--;
							break;
						default:
							break;
					
					}
                    
				break;
				case Key_F2:
					switch(Button_Page.index)
					{
						case 0:
							SetSystemStatus(SYS_STATUS_SETUPTEST);
							break;
						case 1:
							Save_Res.Sys_Setvalue.uart=1;
							break;
						case 2:
//							Save_Res.Sys_Setvalue.buard=1;
							break;
						case 3:
							Save_Res.Sys_Setvalue.u_flag=1;
							break;
						case 4:
							Save_Res.clearsw=1;
							break;
						case 5:
							Save_Res.Sys_Setvalue.lanage=1;
							break;
						case 6:
							if(RTC_TIME_DISP.YEAR>=RTC_YEAR_MAX)
								RTC_TIME_DISP.YEAR=0;
							else
								RTC_TIME_DISP.YEAR++;
                            RTC_SetTime (LPC_RTC, RTC_TIMETYPE_YEAR, RTC_TIME_DISP.YEAR);
                            
                            RTC_CalibConfig(LPC_RTC, 0, RTC_CALIB_DIR_FORWARD);
                            RTC_CalibCounterCmd(LPC_RTC, ENABLE);
							break;
						case 7:
							if(RTC_TIME_DISP.MONTH>=RTC_MONTH_MAX)
								RTC_TIME_DISP.MONTH=1;
							else
								RTC_TIME_DISP.MONTH++;
                            RTC_SetTime (LPC_RTC, RTC_TIMETYPE_MONTH, RTC_TIME_DISP.MONTH);
                            
                            RTC_CalibConfig(LPC_RTC, 0, RTC_CALIB_DIR_FORWARD);
                            RTC_CalibCounterCmd(LPC_RTC, ENABLE);
							break;
						case 8:
							if(RTC_TIME_DISP.DOM>=RTC_DAYOFMONTH_MAX)
								RTC_TIME_DISP.DOM=1;
							else
								RTC_TIME_DISP.DOM++;
                            RTC_SetTime (LPC_RTC, RTC_TIMETYPE_DAYOFMONTH, RTC_TIME_DISP.DOM);
                            
                            RTC_CalibConfig(LPC_RTC, 0, RTC_CALIB_DIR_FORWARD);
                            RTC_CalibCounterCmd(LPC_RTC, ENABLE);
							break;
						case 9:
							if(RTC_TIME_DISP.HOUR>=RTC_HOUR_MAX)
								RTC_TIME_DISP.HOUR=0;
							else
								RTC_TIME_DISP.HOUR++;
                            RTC_SetTime (LPC_RTC, RTC_TIMETYPE_HOUR, RTC_TIME_DISP.HOUR);
                            
                            RTC_CalibConfig(LPC_RTC, 0, RTC_CALIB_DIR_FORWARD);
                            RTC_CalibCounterCmd(LPC_RTC, ENABLE);
							break;
						case 10:
							if(RTC_TIME_DISP.MIN>=RTC_MINUTE_MAX)
								RTC_TIME_DISP.MIN=0;
							else
								RTC_TIME_DISP.MIN++;
                            RTC_SetTime (LPC_RTC, RTC_TIMETYPE_MINUTE, RTC_TIME_DISP.MIN);
                            
                            RTC_CalibConfig(LPC_RTC, 0, RTC_CALIB_DIR_FORWARD);
                            RTC_CalibCounterCmd(LPC_RTC, ENABLE);
							break;
						case 11:
							if(RTC_TIME_DISP.SEC>=RTC_SECOND_MAX)
								RTC_TIME_DISP.SEC=0;
							else
								RTC_TIME_DISP.SEC++;
                            RTC_SetTime (LPC_RTC, RTC_TIMETYPE_SECOND, RTC_TIME_DISP.SEC);
                            
                            RTC_CalibConfig(LPC_RTC, 2, RTC_CALIB_DIR_FORWARD);
                            RTC_CalibCounterCmd(LPC_RTC, ENABLE);
							break;
						case 18:
							SaveData.Sys_Setup.Bias=1;
							break;
						case 12:
							SaveData.Sys_Setup.Bus_Mode=1;
							break;
						case 13:
							if(SaveData.Sys_Setup.GP_Addr>99)
								SaveData.Sys_Setup.GP_Addr=0;
							else
								SaveData.Sys_Setup.GP_Addr++;
							break;
						case 14:
							SaveData.Sys_Setup.Talk_Only=0;
							break;
						case 15:
							if(SaveData.Sys_Setup.Timer_Value.Hour>12)
								SaveData.Sys_Setup.Timer_Value.Hour=0;
							else
								SaveData.Sys_Setup.Timer_Value.Hour++;
							break;
						case 16:
							if(SaveData.Sys_Setup.Timer_Value.Min>69)
								SaveData.Sys_Setup.Timer_Value.Min=0;
							else
								SaveData.Sys_Setup.Timer_Value.Min++;
							break;
						case 17:
							if(SaveData.Sys_Setup.Timer_Value.Sec>59)
								SaveData.Sys_Setup.Timer_Value.Sec=0;
							else
								SaveData.Sys_Setup.Timer_Value.Sec++;
							break;
						default:
							break;
					
					}
                    
				break;
				case Key_F3:
					switch(Button_Page.index)
					{
						case 0:
							SetSystemStatus(SYS_STATUS_SYSSET);
							break;
						case 2:
//							Save_Res.Sys_Setvalue.buard=2;
							
						break;
						default:
						break;
					
					}
						
				break;
				case Key_F4:
					switch(Button_Page.index)
					{
						case 0:
							SetSystemStatus(SYS_STATUS_SYS);
							break;
						case 2:
							RoffsetSingleReset();
							
						break;
						default:
						break;
					
					}
				break;
				case Key_F5:
					switch(Button_Page.index)
					{
						case 0:
							break;//恢复系统复位
						case 2:
							RoffsetAllReset();
							
						break;
						default:
						break;
					
					}
				break;
				case Key_Disp:
					ComBuf.pageswflag=1;
					SetSystemStatus(SYS_STATUS_TEST);
				break;
				case Key_SETUP:
                    SetSystemStatus(SYS_STATUS_SETUPTEST);
				break;
				case Key_FAST:
                    if(Button_Page.index==12)
                    {
                        //dispflag=0;
//                        for(i=0;i<8;i++)
//                        {
//                            Save_Res.Sys_Setvalue.textname[i]=Disp_buff[i];
//                        
//                        }
                        key_count=0;
                        Button_Page.index=0;
                        Savetoeeprom();
                    }
				break;
				case Key_LEFT:
				break;
				case Key_RIGHT:
				break;
				case Key_DOWN:
					if(Button_Page.index>SYS_MAX-1)
						Button_Page.index=0;
					else
						Button_Page.index++;
						
				break;
				case Key_UP:
					if(Button_Page.index<1)
						Button_Page.index=SYS_MAX;
					else
						Button_Page.index--;
				break;
				case Key_NUM1:
					if(Button_Page.index==12)
					{
						if(key_count<PASSWORD_LENTH-1)
						{
							Save_Res.Sys_Setvalue.textname[key_count]='1';
							key_count++;
										
						}
					}else if(Button_Page.index==2){
						Coordinates.xpos=LIST1+90;
						Coordinates.ypos=FIRSTLINE+SPACE1*1;
						Coordinates.lenth=76+24;
						Inputbuf = Disp_Set_Num(&Coordinates);
						if(Save_Res.Set_Data.Range == 0)
						{
							Save_Res.Roffset[Test_Dispvalue.Rangedisp-1] = Inputbuf;
						}else{
							Save_Res.Roffset[Save_Res.Set_Data.Range-1] = Inputbuf;
						}
						RoffsetComp();
					}
                        //Save_Res.Sys_Setvalue
				break;
				case Key_NUM2:
					if(Button_Page.index==12)
					{
						if(key_count<PASSWORD_LENTH-1)
						{
							Save_Res.Sys_Setvalue.textname[key_count]='2';
							key_count++;
										
						}
					
					}else if(Button_Page.index==2){
						Coordinates.xpos=LIST1+90;
						Coordinates.ypos=FIRSTLINE+SPACE1*1;
						Coordinates.lenth=76+24;
						Inputbuf = Disp_Set_Num(&Coordinates);
						if(Save_Res.Set_Data.Range == 0)
						{
							Save_Res.Roffset[Test_Dispvalue.Rangedisp-1] = Inputbuf;
						}else{
							Save_Res.Roffset[Save_Res.Set_Data.Range-1] = Inputbuf;
						}
						RoffsetComp();
					}
				break;
				case Key_NUM3:
					if(Button_Page.index==12)
					{
							if(key_count<PASSWORD_LENTH-1)
							{
									Save_Res.Sys_Setvalue.textname[key_count]='3';
									key_count++;
											
							}
					
					}else if(Button_Page.index==2){
						Coordinates.xpos=LIST1+90;
						Coordinates.ypos=FIRSTLINE+SPACE1*1;
						Coordinates.lenth=76+24;
						Inputbuf = Disp_Set_Num(&Coordinates);
						if(Save_Res.Set_Data.Range == 0)
						{
							Save_Res.Roffset[Test_Dispvalue.Rangedisp-1] = Inputbuf;
						}else{
							Save_Res.Roffset[Save_Res.Set_Data.Range-1] = Inputbuf;
						}
						RoffsetComp();
					}
				break;
				case Key_NUM4:
					if(Button_Page.index==12)
					{
							if(key_count<PASSWORD_LENTH-1)
							{
									Save_Res.Sys_Setvalue.textname[key_count]='4';
									key_count++;
											
							}
					
					}else if(Button_Page.index==2){
						Coordinates.xpos=LIST1+90;
						Coordinates.ypos=FIRSTLINE+SPACE1*1;
						Coordinates.lenth=76+24;
						Inputbuf = Disp_Set_Num(&Coordinates);
						if(Save_Res.Set_Data.Range == 0)
						{
							Save_Res.Roffset[Test_Dispvalue.Rangedisp-1] = Inputbuf;
						}else{
							Save_Res.Roffset[Save_Res.Set_Data.Range-1] = Inputbuf;
						}
						RoffsetComp();
					}
				break;
				case Key_NUM5:
					if(Button_Page.index==12)
					{
							if(key_count<PASSWORD_LENTH-1)
							{
									Save_Res.Sys_Setvalue.textname[key_count]='5';
									key_count++;
											
							}
					
					}else if(Button_Page.index==2){
						Coordinates.xpos=LIST1+90;
						Coordinates.ypos=FIRSTLINE+SPACE1*1;
						Coordinates.lenth=76+24;
						Inputbuf = Disp_Set_Num(&Coordinates);
						if(Save_Res.Set_Data.Range == 0)
						{
							Save_Res.Roffset[Test_Dispvalue.Rangedisp-1] = Inputbuf;
						}else{
							Save_Res.Roffset[Save_Res.Set_Data.Range-1] = Inputbuf;
						}
						RoffsetComp();
					}
				break;
				case Key_NUM6:
					if(Button_Page.index==12)
					{
							if(key_count<PASSWORD_LENTH-1)
							{
									Save_Res.Sys_Setvalue.textname[key_count]='6';
									key_count++;
											
							}
					
					}else if(Button_Page.index==2){
						Coordinates.xpos=LIST1+90;
						Coordinates.ypos=FIRSTLINE+SPACE1*1;
						Coordinates.lenth=76+24;
						Inputbuf = Disp_Set_Num(&Coordinates);
						if(Save_Res.Set_Data.Range == 0)
						{
							Save_Res.Roffset[Test_Dispvalue.Rangedisp-1] = Inputbuf;
						}else{
							Save_Res.Roffset[Save_Res.Set_Data.Range-1] = Inputbuf;
						}
						RoffsetComp();
					}
				break;
				case Key_NUM7:
					if(Button_Page.index==12)
					{
							if(key_count<PASSWORD_LENTH-1)
							{
									Save_Res.Sys_Setvalue.textname[key_count]='7';
									key_count++;
											
							}
					
					}else if(Button_Page.index==2){
						Coordinates.xpos=LIST1+90;
						Coordinates.ypos=FIRSTLINE+SPACE1*1;
						Coordinates.lenth=76+24;
						Inputbuf = Disp_Set_Num(&Coordinates);
						if(Save_Res.Set_Data.Range == 0)
						{
							Save_Res.Roffset[Test_Dispvalue.Rangedisp-1] = Inputbuf;
						}else{
							Save_Res.Roffset[Save_Res.Set_Data.Range-1] = Inputbuf;
						}
						RoffsetComp();
					}
				break;
				case Key_NUM8:
					if(Button_Page.index==12)
					{
							if(key_count<PASSWORD_LENTH-1)
							{
									Save_Res.Sys_Setvalue.textname[key_count]='8';
									key_count++;
											
							}
					
					}else if(Button_Page.index==2){
						Coordinates.xpos=LIST1+90;
						Coordinates.ypos=FIRSTLINE+SPACE1*1;
						Coordinates.lenth=76+24;
						Inputbuf = Disp_Set_Num(&Coordinates);
						if(Save_Res.Set_Data.Range == 0)
						{
							Save_Res.Roffset[Test_Dispvalue.Rangedisp-1] = Inputbuf;
						}else{
							Save_Res.Roffset[Save_Res.Set_Data.Range-1] = Inputbuf;
						}
						RoffsetComp();
					}
				break;
				case Key_NUM9:
					if(Button_Page.index==12)
					{
							if(key_count<PASSWORD_LENTH-1)
							{
									Save_Res.Sys_Setvalue.textname[key_count]='9';
									key_count++;
											
							}
					
					}else if(Button_Page.index==2){
						Coordinates.xpos=LIST1+90;
						Coordinates.ypos=FIRSTLINE+SPACE1*1;
						Coordinates.lenth=76+24;
						Inputbuf = Disp_Set_Num(&Coordinates);
						if(Save_Res.Set_Data.Range == 0)
						{
							Save_Res.Roffset[Test_Dispvalue.Rangedisp-1] = Inputbuf;
						}else{
							Save_Res.Roffset[Save_Res.Set_Data.Range-1] = Inputbuf;
						}
						RoffsetComp();
					}
				break;
				case Key_NUM0:
					if(Button_Page.index==12)
					{
							if(key_count<PASSWORD_LENTH-1)
							{
									Save_Res.Sys_Setvalue.textname[key_count]='0';
									key_count++;
											
							}
					
					}else if(Button_Page.index==2){
						Coordinates.xpos=LIST1+90;
						Coordinates.ypos=FIRSTLINE+SPACE1*1;
						Coordinates.lenth=76+24;
						Inputbuf = Disp_Set_Num(&Coordinates);
						if(Save_Res.Set_Data.Range == 0)
						{
							Save_Res.Roffset[Test_Dispvalue.Rangedisp-1] = Inputbuf;
						}else{
							Save_Res.Roffset[Save_Res.Set_Data.Range-1] = Inputbuf;
						}
						RoffsetComp();
					}
				break;
				case Key_DOT:
					if(Button_Page.index==2){
						Coordinates.xpos=LIST1+90;
						Coordinates.ypos=FIRSTLINE+SPACE1*1;
						Coordinates.lenth=76+24;
						Inputbuf = Disp_Set_Num(&Coordinates);
						if(Save_Res.Set_Data.Range == 0)
						{
							Save_Res.Roffset[Test_Dispvalue.Rangedisp-1] = Inputbuf;
						}else{
							Save_Res.Roffset[Save_Res.Set_Data.Range-1] = Inputbuf;
						}
						RoffsetComp();
					}
				break;
				case Key_BACK:
				break;
				case Key_LOCK:
				break;
				case Key_BIAS:
				break;
				case Key_REST:
				break;
				case Key_TRIG:
				break;
				default:
				break;
					
			}
		
		
		}
	
	
	}
	Savetoeeprom();
}

//电阻校正计算
void DebugHandleR(u8 range)
{
	float standard;
	standard = ((float)Save_Res.DebugStd[range].Num)/((float)pow(10,Save_Res.DebugStd[range].Dot));
	Save_Res.Debug_Value[range] = standard/Test_Dispvalue.Rraw;
}

//电压校正计算
void DebugHandleV(u8 step)
{
	float standard;
	u16 coff;
	if(Save_Res.DebugStd[step+6].Dot == 5)
		coff = 1;
	else if(Save_Res.DebugStd[step+6].Dot == 4)//10V以上
		coff = 10;
	else if(Save_Res.DebugStd[step+6].Dot == 3)//100V以上
		coff = 100;
	else if(Save_Res.DebugStd[step+6].Dot == 2)//1000V以上
		coff = 1000;
	standard = ((float)Save_Res.DebugStd[step+6].Num)*coff;
	if(step == 1 || step == 3 || step == 5)
	{
		x1 = (double)Test_Dispvalue.Vdataraw.num;
		y1 = standard;
	}else if(step == 2){
		x2 = (double)Test_Dispvalue.Vdataraw.num;
		y2 = standard;
		Save_Res.VDebug_Valuek[0] = (y2 - y1)/(x2 - x1);
		Save_Res.VDebug_Valueb[0] = (double)y2 - Save_Res.VDebug_Valuek[0]*(double)x2;
	}else if(step == 4){
		x2 = (double)Test_Dispvalue.Vdataraw.num;
		y2 = standard;
		Save_Res.VDebug_Valuek[1] = (y2 - y1)/(x2 - x1);
		Save_Res.VDebug_Valueb[1] = (double)y2 - Save_Res.VDebug_Valuek[1]*(double)x2;
	}else if(step == 6){
		x2 = (double)Test_Dispvalue.Vdataraw.num;
		y2 = standard;
		Save_Res.VDebug_Valuek[2] = (y2 - y1)/(x2 - x1);
		Save_Res.VDebug_Valueb[2] = (double)y2 - Save_Res.VDebug_Valuek[2]*(double)x2;
	}
}

void DebugRInit(void)
{
	Save_Res.DebugStd[0].Num=9866;
	Save_Res.DebugStd[0].Dot = 4;
	
	Save_Res.DebugStd[1].Num=100263;
	Save_Res.DebugStd[1].Dot = 4;
	
	Save_Res.DebugStd[2].Num=10001;
	Save_Res.DebugStd[2].Dot = 2;
	
	Save_Res.DebugStd[3].Num=10014;
	Save_Res.DebugStd[3].Dot = 4;
	
	Save_Res.DebugStd[4].Num=100020;
	Save_Res.DebugStd[4].Dot = 4;
	
	Save_Res.DebugStd[5].Num=100009;
	Save_Res.DebugStd[5].Dot = 3;
	
	Save_Res.DebugStd[6].Num=10000;
	Save_Res.DebugStd[6].Dot = 1;
}

//用户校正
void Use_DebugProcess(void)
{
	
	vu32 keynum=0;
	vu8 key;
	float ddd,eee;
    Disp_Coordinates_Typedef Debug_Cood;
	Disp_Coordinates_Typedef Coordinates;
//    vu8 page=1;
	vu8 Disp_flag=1;
	Button_Page_Typedef Button_Page;
	Button_Page.index=0;
	Button_Page.page=0;
	lcd_Clear(LCD_COLOR_TEST_BACK);
	Disp_UserCheck_Item(&Button_Page);
	Delay_Key();
	DebugRInit();
	Test_Dispvalue.CalMode = 0;
 	while(GetSystemStatus()==SYS_STATUS_USERDEBUG)
	{
//		Uart_Process();
		Disp_Data_Debug();
		if(Disp_flag==1)
		{
			Disp_Debug_value(&Button_Page);
			Disp_flag=0;	
		}

		key=HW_KeyScsn();
		if(key==0xff)
		{
			keynum=0;
		}
		else
			keynum++;
		if(keynum==KEY_NUM)  
		{
			Disp_flag=1;
            Key_Beep();
			switch(key)
			{
				case Key_F1:
					Button_Page.page = 0;
					Button_Page.index = 0;
					Test_Dispvalue.CalMode = 0;
//					Send_RangeCALR();
					Disp_UserCheck_Item(&Button_Page);
//                    if(Button_Page.page==0)
//                        SetSystemStatus(SYS_STATUS_SETUPTEST);
//					else
//						SetSystemStatus(SYS_STATUS_FILE);
						
				break;
				case Key_F2:
					Button_Page.page = 1;
					Button_Page.index = 0;
					Test_Dispvalue.CalMode = 1;
//					Send_RangeCALV();
					Disp_UserCheck_Item(&Button_Page);
//                    if(Button_Page.page==0)
//                        SetSystemStatus(SYS_STATUS_USERDEBUG);
//					else
//						SetSystemStatus(SYS_STATUS_SYSSET);
				break;
				case Key_F3:
					Savetoeeprom();
					SetSystemStatus(SYS_STATUS_TEST);
//                    if(Button_Page.page==0)
//                        SetSystemStatus(SYS_STATUS_LIMITSET);
//					else
//						SetSystemStatus(SYS_STATUS_TOOL);
				break;
				case Key_F4:
					if(Save_Res.version < 4)
						Save_Res.version ++;
					else
						Save_Res.version = 0;
//                    if(Button_Page.page==0)
//                        SetSystemStatus(SYS_STATUS_ITEMSET);
				break;
				case Key_F5:
					if(Button_Page.page == 0)
					{
//					if(Button_Page.index < 8)
						DebugHandleR(Button_Page.index-1);
					}else if(Button_Page.page == 1){
						DebugHandleV(Button_Page.index);
					}
//					Savetoeeprom();
//					if(Button_Page.page)
//						Button_Page.page=0;
//					else
//						Button_Page.page=1;
//                    Disp_Button_TestSet(Button_Page.page);
				break;
				case Key_Disp:
					if(Button_Page.index==0)
					{
							Debug_Cood.xpos=70;
							Debug_Cood.ypos =272-70;
							Debug_Cood.lenth=120;
							input_num(&Debug_Cood);
					
					}

				break;
				case Key_SETUP:
                   // SetSystemStatus(SYS_STATUS_SETUPTEST);
				break;
				case Key_FAST:
					
                    
				
				
				break;
				
				
                case Key_LEFT:
				case Key_UP:
					if(Button_Page.page == 0)
					{
						if(Button_Page.index>0)
							Button_Page.index--;
						else
							Button_Page.index=DEBUG_RANGE;
					}else if(Button_Page.page == 1){
						if(Button_Page.index>0)
							Button_Page.index--;
						else
							Button_Page.index=6;
					}
				break;
                        case Key_RIGHT:
				case Key_DOWN:
//					Save_Res.Debug_Value[Button_Page.index].standard=
//					Save_Res.Debug_Value[Button_Page.index].ad_value=
					if(Button_Page.page == 0)
					{
						if(Button_Page.index<DEBUG_RANGE)
							Button_Page.index++;
						else
							Button_Page.index=0;
					}else if(Button_Page.page == 1){
						if(Button_Page.index<6)
							Button_Page.index++;
						else
							Button_Page.index=0;
					}
                    
					
				break;
					case Key_DOT:
						break;
				case Key_NUM1:
				//break;
				case Key_NUM2:
				//break;
				case Key_NUM3:
				//break;
				case Key_NUM4:
				//break;
				case Key_NUM5:
				//break;
				case Key_NUM6:
				//break;
				case Key_NUM7:
				//break;
				case Key_NUM8:
				//break;
				case Key_NUM9:
				//break;
				case Key_NUM0:
				//break;
				if(Button_Page.page == 0)
				{
//					if(Button_Page.index < 8)
//					{
						Coordinates.xpos=LIST1+88;
						Coordinates.ypos=FIRSTLINE+SPACE1*(Button_Page.index-1);
						Coordinates.lenth=60+8;
						Save_Res.DebugStd[Button_Page.index-1]=Disp_Set_Num(&Coordinates);
//					}else{
//						Coordinates.xpos=LIST1+88;
//						Coordinates.ypos=FIRSTLINE+SPACE1*(Button_Page.index-1);
//						Coordinates.lenth=60;
//						Save_Res.DebugStd[Button_Page.index-1]=Disp_Set_NumV(&Coordinates);					
//					}
				}else if(Button_Page.page == 1){
						Coordinates.xpos=LIST1+88;
						Coordinates.ypos=FIRSTLINE+SPACE1*(Button_Page.index-1);
						Coordinates.lenth=60;
						Save_Res.DebugStd[Button_Page.index-1+7]=Disp_Set_NumV(&Coordinates);	
				}
				LCD_DrawRect( 0, 227, 479, 271 , LCD_COLOR_TEST_BACK ) ;
				Disp_UserCheck_Item(&Button_Page);
//				Coordinates.xpos=LIST1+160;
//				Coordinates.ypos=FIRSTLINE+SPACE1*(Button_Page.index);
//				Coordinates.lenth=70;
//				Save_Res.Debug_Value[Button_Page.index-1].standard=Freq_Set_Num(&Coordinates);
				
//					if(Button_Page.index==5)
//					{ 	
				//Disp_Coordinates_Typedef Coordinates;
//						Coordinates.xpos=LIST1+92;
//						Coordinates.ypos=FIRSTLINE+SPACE1*4;
//						Coordinates.lenth=86;
//						
//						SaveData.Main_Func.Freq=Freq_Set_Num(&Coordinates);
//					
//					}
				break;
				case Key_BACK:
				break;
				case Key_LOCK:
				break;
				case Key_BIAS:
				break;
				case Key_REST:
					if(Save_Res.open == 0)
					{
						Save_Res.open = 1;
					}else{
						Save_Res.open = 0;
					}
				break;
				case Key_TRIG:
				break;
				default:
				break;
					
			}
		
		
		}
	
	
	}
}

//公司校正
void Fac_DebugProcess(void)
{
	vu32 keynum=0;
	vu8 key;
 	while(GetSystemStatus()==SYS_STATUS_FACRDEBUG)
	{
		key=HW_KeyScsn();
		if(key==0xff)
		{
			keynum=0;
		}
		else
			keynum++;
		if(keynum==2)
		{
            Key_Beep();
			switch(key)
			{
				case Key_F1:
				break;
				case Key_F2:
				break;
				case Key_F3:
				break;
				case Key_F4:
				break;
				case Key_F5:
				break;
				case Key_Disp:
				break;
				case Key_SETUP:
				break;
				case Key_FAST:
				break;
				case Key_LEFT:
				break;
				case Key_RIGHT:
				break;
				case Key_UP:
				break;
				case Key_DOWN:
				break;
				case Key_NUM1:
				break;
				case Key_NUM2:
				break;
				case Key_NUM3:
				break;
				case Key_NUM4:
				break;
				case Key_NUM5:
				break;
				case Key_NUM6:
				break;
				case Key_NUM7:
				break;
				case Key_NUM8:
				break;
				case Key_NUM9:
				break;
				case Key_NUM0:
				break;
				case Key_DOT:
				break;
				case Key_BACK:
				break;
				case Key_LOCK:
				break;
				case Key_BIAS:
				break;
				case Key_REST:
				break;
				case Key_TRIG:
				break;
				default:
				break;
					
			}
		
		
		}
	
	
	}
}	
void VrangeSW(void)
{
	if(Test_Dispvalue.openflag == 0)
	{
		if(Test_Dispvalue.Vdata >= 0)
		{
			if(Test_Dispvalue.vrange == 0)
			{
				if(Test_Dispvalue.Vdata > 10.5)
					Test_Dispvalue.vrange = 1;
			}else if(Test_Dispvalue.vrange == 1){
				if(Test_Dispvalue.Vdata < 9.5)
					Test_Dispvalue.vrange = 0;
				else if(Test_Dispvalue.Vdata > 105)
					Test_Dispvalue.vrange = 2;
			}else if(Test_Dispvalue.vrange == 2){
				if(Test_Dispvalue.Vdata < 95)
					Test_Dispvalue.vrange = 1;
			}
		}else{
			if(Test_Dispvalue.vrange == 0)
			{
				if(Test_Dispvalue.Vdata < -10.5)
					Test_Dispvalue.vrange = 1;
			}else if(Test_Dispvalue.vrange == 1){
				if(Test_Dispvalue.Vdata > -9.5)
					Test_Dispvalue.vrange = 0;
				else if(Test_Dispvalue.Vdata < -105)
					Test_Dispvalue.vrange = 2;
			}else if(Test_Dispvalue.vrange == 2){
				if(Test_Dispvalue.Vdata > -95)
					Test_Dispvalue.vrange = 1;
			}
		}
	}else{
		Test_Dispvalue.vrange = 0;
	}
}
void VDATAFILTER(void)
{
	u16 i;
	if(Test_Dispvalue.openflag == 1/* || Test_Dispvalue.Vdataraw.coefficient != Vfilter.oldcft*/)//开路或电压小数点位置变化
	{
		Vfilter.initflag = 1;//队列初始化
		Vfilter.initcount = 0;//进队列数据计数清零
//		Rfilter.index = 0;
		Vfilter.recindex = 0;
		Vfilter.initdataflag = 0;
		memset(Vfilter.buffer.data,0,sizeof(Vfilter.buffer.data));
		
	}
		
	if(Vfilter.initflag == 1)//队列初始化操作，结束前不进行滤波
	{
		if(Test_Dispvalue.Vdataraw.index != Vfilter.index)//序号与上次不一样则进入队列，否则跳过
		{
			Vfilter.index = Test_Dispvalue.Vdataraw.index;//记录本次序号
			Vfilter.oldcft = Test_Dispvalue.Vdataraw.coefficient;
			if(Vfilter.initcount < 20-1)
			{
				Vfilter.buffer.indexbuf[Vfilter.initcount] = Test_Dispvalue.Vdataraw.index;
				Vfilter.buffer.data[Vfilter.initcount] = Test_Dispvalue.Vdataraw.num * pow(10,5-Test_Dispvalue.Vdataraw.coefficient);
				Vfilter.initcount++;
				if(Test_Dispvalue.openflag == 0)
				{
					Test_Dispvalue.Test_V = (Test_Dispvalue.Vdataraw.num * 
					pow(10,5-Test_Dispvalue.Vdataraw.coefficient))*Save_Res.VDebug_Valuek[Test_Dispvalue.vrange]+
					Save_Res.VDebug_Valueb[Test_Dispvalue.vrange];
					
					if(Test_Dispvalue.Test_V < 1000000)
					{
						Test_Dispvalue.TestVDot = 5;
					}else if(Test_Dispvalue.Test_V >= 1000000 && Test_Dispvalue.Test_V < 10000000){
						Test_Dispvalue.Test_V/=10;
						Test_Dispvalue.TestVDot = 4;
					}else if(Test_Dispvalue.Test_V >= 10000000 && Test_Dispvalue.Test_V < 100000000){
						Test_Dispvalue.Test_V/=100;
						Test_Dispvalue.TestVDot = 3;
					}
				
//					if(Test_Dispvalue.Vdataraw.coefficient ==5)
//					{
//						Test_Dispvalue.Test_V = (u32)(((float)Test_Dispvalue.Vdataraw.num)*Save_Res.VDebug_Valuek[Test_Dispvalue.vrange]+
//							Save_Res.VDebug_Valueb[Test_Dispvalue.vrange]);
//					}else if(Test_Dispvalue.Vdataraw.coefficient ==4){
//						Test_Dispvalue.Test_V = (u32)(((float)Test_Dispvalue.Vdataraw.num*10)*Save_Res.VDebug_Valuek[Test_Dispvalue.vrange]+
//							Save_Res.VDebug_Valueb[Test_Dispvalue.vrange])/10;
//					}else if(Test_Dispvalue.Vdataraw.coefficient ==3){
//						Test_Dispvalue.Test_V = (u32)(((float)Test_Dispvalue.Vdataraw.num*100)*Save_Res.VDebug_Valuek[Test_Dispvalue.vrange]+
//							Save_Res.VDebug_Valueb[Test_Dispvalue.vrange])/100;
//					}
				}
			}else{
				BubbleSort(Vfilter.buffer.data,20);
				Vfilter.initflag = 0;//队列初始化结束，可以开始滤波
				Vfilter.count = 0;
			}
		}
	}else{
		if(Test_Dispvalue.Vdataraw.index != Vfilter.index)//序号与上次不一样则进入队列，否则跳过
		{
			Vfilter.index = Test_Dispvalue.Vdataraw.index;//记录本次序号
			Vfilter.oldcft = Test_Dispvalue.Vdataraw.coefficient;//记录当前系数
			if(Vfilter.count == 0)
			{
				Vfilter.count ++;
				Vfilter.buffer.data[0] = Test_Dispvalue.Vdataraw.num * pow(10,5-Test_Dispvalue.Vdataraw.coefficient);//第一个新数据进入队首
				HeadSort(Vfilter.buffer.data,20);//新数据单独排序
				

			}else if(Vfilter.count == 1){
				Vfilter.count ++; 
				Vfilter.buffer.data[20-1] = Test_Dispvalue.Vdataraw.num * pow(10,5-Test_Dispvalue.Vdataraw.coefficient);//第二个新数据进入队尾
				TailSort(Vfilter.buffer.data,20);//新数据单独排序
			}
			
			if(Vfilter.count ==2)
			{
				
				Vfilter.count = 0;
				Vfilter.sum = 0;
				
				for(i=5;i<15;i++)
				{
					Vfilter.sum += Vfilter.buffer.data[i];//根据队列长度取中间一半数据求和
				}
				Vfilter.result = Vfilter.sum/10;//求和后平均计算出滤波后数据
				Test_Dispvalue.Test_V = (u32)(((float)Vfilter.result)*Save_Res.VDebug_Valuek[Test_Dispvalue.vrange]+
					Save_Res.VDebug_Valueb[Test_Dispvalue.vrange]);
//				if(Save_Res.clearsw == 1)//清零开关打开执行清零操作
//				{
//					if(Test_Dispvalue.Test_V >= Save_Res.vclear)
//						Test_Dispvalue.Test_V-=Save_Res.vclear;
//					else
//						Test_Dispvalue.Test_V = 0;
//				}
				
				if(Test_Dispvalue.Test_V < 1000000)
				{
					Test_Dispvalue.TestVDot = 5;
				}else if(Test_Dispvalue.Test_V >= 1000000 && Test_Dispvalue.Test_V < 10000000){
					Test_Dispvalue.Test_V/=10;
					Test_Dispvalue.TestVDot = 4;
				}else if(Test_Dispvalue.Test_V >= 10000000 && Test_Dispvalue.Test_V < 100000000){
					Test_Dispvalue.Test_V/=100;
					Test_Dispvalue.TestVDot = 3;
				}else if(Test_Dispvalue.Test_V >= 100000000 && Test_Dispvalue.Test_V < 1000000000){
					Test_Dispvalue.Test_V/=1000;
					Test_Dispvalue.TestVDot = 2;
				}
//				if(Test_Dispvalue.Vdataraw.coefficient ==5)
//				{
//					Test_Dispvalue.Test_V = (u32)(((float)Vfilter.result)*Save_Res.VDebug_Valuek[Test_Dispvalue.vrange]+
//						Save_Res.VDebug_Valueb[Test_Dispvalue.vrange]);
//				}else if(Test_Dispvalue.Vdataraw.coefficient ==4){
//					Test_Dispvalue.Test_V = (u32)(((float)Vfilter.result*10)*Save_Res.VDebug_Valuek[Test_Dispvalue.vrange]+
//						Save_Res.VDebug_Valueb[Test_Dispvalue.vrange])/10;
//				}else if(Test_Dispvalue.Vdataraw.coefficient ==3){
//					Test_Dispvalue.Test_V = (u32)(((float)Vfilter.result*100)*Save_Res.VDebug_Valuek[Test_Dispvalue.vrange]+
//						Save_Res.VDebug_Valueb[Test_Dispvalue.vrange])/100;
//				}
//				Test_Dispvalue.Test_V = (u32)((Test_Dispvalue.Vdata*Save_Res.VDebug_Valuek + Save_Res.VDebug_Valueb)*
//				pow(10,Test_Dispvalue.Vdataraw.coefficient));
			}
		}
	}
}

void RoffsetHandle(void)
{
	static double offsetbuf;
	static u32 offsetval;
	offsetbuf = (double)Save_Res.Roffset[Test_Dispvalue.Rangedisp - 1].Num/pow(10,Save_Res.Roffset[Test_Dispvalue.Rangedisp - 1].Dot);
	offsetval = (u32)(offsetbuf*pow(10,rangedot[Test_Dispvalue.Rangedisp - 1]));
	if(Save_Res.Roffset[Test_Dispvalue.Rangedisp - 1].sign == 0)//相位角修正值为+	
	{
		Test_Dispvalue.Test_R += offsetval;
	}else if(Save_Res.Roffset[Test_Dispvalue.Rangedisp - 1].sign == 1){//相位角修正值为-
		if(offsetval <= Test_Dispvalue.Test_R)
		{
			Test_Dispvalue.Test_R -= offsetval; 
		}else{
			Test_Dispvalue.Test_R = offsetval - Test_Dispvalue.Test_R;
		}
	}
}

void RDATAFILTER_IRR(void)
{
	if(Save_Res.version == 0)//3560电阻量程最大为3Ω
	{
		if(Test_Dispvalue.Rdataraw.num == 0xffff || Test_Dispvalue.Rdataraw.range > 4)//采集到开路或短路数据
		{
			trip_flag = 0;//手动触发标志复位
			Test_Dispvalue.openflag = 1;//开路标志
			Rfilter.initcount = 0;//进队列数据计数清零
			Rfilter.index = 0xff;//序号
		}else{
			if(Test_Dispvalue.Rdataraw.index != Rfilter.index)//序号与上次不一样则进入队列，否则跳过
			{
				Rfilter.index = Test_Dispvalue.Rdataraw.index;//记录本次序号
				if(Rfilter.initcount < 2)//前两个数据舍弃
				{
					Test_Dispvalue.openflag = 1;
					Rfilter.initcount++;
				}else if(Rfilter.initcount == 2){//记录第一个有效数据
					Test_Dispvalue.openflag = 0;
					Rfilter.oldres = Test_Dispvalue.Rdataraw.num;
					Rfilter.result = Test_Dispvalue.Rdataraw.num;
					Rfilter.initcount++;					
				}else{
					Test_Dispvalue.openflag = 0;
					Rfilter.newres = Test_Dispvalue.Rdataraw.num;
					if(Rfilter.newres >= Rfilter.oldres)
					{
						if(Rfilter.newres - Rfilter.oldres <=  100)
						{
							Rfilter.result = (u32)((double)Rfilter.oldres + ((double)Rfilter.newres - (double)Rfilter.oldres)*0.1);
						}else{
							Rfilter.oldres = Test_Dispvalue.Rdataraw.num;
							Rfilter.result = Test_Dispvalue.Rdataraw.num;
						}
					}else{
						if(Rfilter.oldres - Rfilter.newres <=  100)
						{
							Rfilter.result = (u32)((double)Rfilter.oldres + ((double)Rfilter.oldres - (double)Rfilter.newres)*0.1);
						}else{
							Rfilter.oldres = Test_Dispvalue.Rdataraw.num;
							Rfilter.result = Test_Dispvalue.Rdataraw.num;
						}
					}
				}
			}
		}			
	}else{
		if(Test_Dispvalue.Rdataraw.num == 0xffff)//采集到开路或短路数据
		{
			trip_flag = 0;//手动触发标志复位
			Test_Dispvalue.openflag = 1;//开路标志
			Rfilter.initcount = 0;//进队列数据计数清零
			Rfilter.index = 0xff;//序号
		}else{
			if(Test_Dispvalue.Rdataraw.index != Rfilter.index)//序号与上次不一样则进入队列，否则跳过
			{
				Rfilter.index = Test_Dispvalue.Rdataraw.index;//记录本次序号
				if(Rfilter.initcount < 2)//前两个数据舍弃
				{
					Test_Dispvalue.openflag = 1;
					Rfilter.initcount++;
				}else if(Rfilter.initcount == 2){//记录第一个有效数据
					Test_Dispvalue.openflag = 0;
					Rfilter.oldres = Test_Dispvalue.Rdataraw.num;
					Rfilter.result = Test_Dispvalue.Rdataraw.num;
					Rfilter.initcount++;					
				}else{
					Test_Dispvalue.openflag = 0;
					Rfilter.newres = Test_Dispvalue.Rdataraw.num;
					if(Rfilter.newres >= Rfilter.oldres)
					{
						if(Rfilter.newres - Rfilter.oldres <=  100)
						{
							Rfilter.result = (u32)((double)Rfilter.oldres + ((double)Rfilter.newres - (double)Rfilter.oldres)*0.1);
							Rfilter.oldres = Rfilter.result;
						}else{
							Rfilter.oldres = Test_Dispvalue.Rdataraw.num;
							Rfilter.result = Test_Dispvalue.Rdataraw.num;
						}
					}else{
						if(Rfilter.oldres - Rfilter.newres <=  100)
						{
							Rfilter.result = (u32)((double)Rfilter.oldres - ((double)Rfilter.oldres - (double)Rfilter.newres)*0.1);
							Rfilter.oldres = Rfilter.result;
						}else{
							Rfilter.oldres = Test_Dispvalue.Rdataraw.num;
							Rfilter.result = Test_Dispvalue.Rdataraw.num;
						}
					}
				}
			}
		}
	}
}

void RDATAFILTER(void)
{
	u16 i;
	if(Save_Res.version == 0)//3560电阻量程最大为3Ω
	{
		if(Test_Dispvalue.Rdataraw.num == 0xffff || Test_Dispvalue.Rdataraw.range > 4)//采集到开路或短路数据
		{
			trip_flag = 0;//手动触发标志复位
			Rfilter.initflag = 1;//队列初始化
			Rfilter.initcount = 0;//进队列数据计数清零
			Test_Dispvalue.openflag = 1;//开路标志
	//		Rfilter.index = 0;
			Rfilter.recindex = 0;
			Rfilter.initdataflag = 0;
			memset(Rfilter.buffer.data,0,sizeof(Rfilter.buffer.data));
			if(u3sendflag == 1)
			{
				if(uartresdelay == 0)
				{
					RecHandle();
					u3sendflag = 0;
					g_tModS.RxCount = 0;
				}else{
					uartresdelay --;
				}
			}
		}
	}else{
		if(Test_Dispvalue.Rdataraw.num == 0xffff/* || Test_Dispvalue.Rdataraw.num == 0*/)//采集到开路或短路数据
		{
			trip_flag = 0;//手动触发标志复位
			Rfilter.initflag = 1;//队列初始化
			Rfilter.initcount = 0;//进队列数据计数清零
			Test_Dispvalue.openflag = 1;//开路标志
	//		Rfilter.index = 0;
			Rfilter.recindex = 0;
			Rfilter.initdataflag = 0;
			memset(Rfilter.buffer.data,0,sizeof(Rfilter.buffer.data));
			if(u3sendflag == 1)
			{
				if(uartresdelay == 0)
				{
					RecHandle();
					u3sendflag = 0;
					g_tModS.RxCount = 0;
				}else{
					uartresdelay --;
				}
			}
		}
	}
	
	if(Rfilter.initflag == 1)//队列初始化操作，结束前不进行滤波
	{
		if(Test_Dispvalue.Rdataraw.index != Rfilter.index)//序号与上次不一样则进入队列，否则跳过
		{
			if(Rfilter.initdataflag == 0)
			{
				Rfilter.initdataflag = 1;
				Rfilter.inittime = 0;
			}
			Rfilter.recorder[Rfilter.recindex++] = Test_Dispvalue.Rdataraw.num;
			Rfilter.index = Test_Dispvalue.Rdataraw.index;//记录本次序号
			Rfilter.buffer.data[Rfilter.initcount] = Test_Dispvalue.Rdataraw.num;//相应数量的数据进入队列

			if(Rfilter.initflag == 1)
			{
				if(Rfilter.initcount < filtersize[Save_Res.Set_Data.speed]-1)
				{
					Rfilter.buffer.indexbuf[Rfilter.initcount] = Test_Dispvalue.Rdataraw.index;
					Rfilter.initcount++;
				}
				else{
//					for(i = 0;i < 20;i ++)
//					{
//						Vfilter.buffer.data[i] = Test_Dispvalue.Vdataraw.num;
//					}
					BubbleSort(Rfilter.buffer.data,filtersize[Save_Res.Set_Data.speed]);
					Rfilter.initflag = 0;//队列初始化结束，可以开始滤波
	//				Rfilter.oldres = Test_Dispvalue.Rdataraw.num;
	//				Rfilter.result = Rfilter.oldres;
					Rfilter.count = 0;
//					Vfilter.count = 0;
				}
			}
		}
	}else{
		Test_Dispvalue.openflag = 0;
//		if(Test_Dispvalue.Vdataraw.index != Vfilter.index)//序号与上次不一样则进入队列，否则跳过
//		{
//			Vfilter.index = Test_Dispvalue.Vdataraw.index;//记录本次序号
//			
//			if(Vfilter.count == 0)
//			{
//				Vfilter.count ++;
//				Vfilter.buffer.data[0] = Test_Dispvalue.Vdataraw.num;//第一个新数据进入队首
//				HeadSort(Vfilter.buffer.data,10);//新数据单独排序
//				

//			}else if(Vfilter.count == 1){
//				Vfilter.count ++; 
//				Vfilter.buffer.data[10-1] = Test_Dispvalue.Vdataraw.num;//第二个新数据进入队尾
//				TailSort(Vfilter.buffer.data,10);//新数据单独排序
//			}
//			
//			if(Vfilter.count ==2)
//			{
//				
//				Vfilter.count = 0;
//				Vfilter.sum = 0;
//				
//				for(i=5;i<15;i++)
//				{
//					Vfilter.sum += Vfilter.buffer.data[i];//根据队列长度取中间一半数据求和
//				}
//				Vfilter.result = Vfilter.sum/10;//求和后平均计算出滤波后数据
//				if(Test_Dispvalue.Vdataraw.coefficient ==5)
//				{
//					Test_Dispvalue.Test_V = (u32)(((float)Vfilter.result)*Save_Res.VDebug_Valuek+Save_Res.VDebug_Valueb);
//				}else if(Test_Dispvalue.Vdataraw.coefficient ==4){
//					Test_Dispvalue.Test_V = (u32)(((float)Vfilter.result*10)*Save_Res.VDebug_Valuek+Save_Res.VDebug_Valueb)/10;
//				}else if(Test_Dispvalue.Vdataraw.coefficient ==3){
//					Test_Dispvalue.Test_V = (u32)(((float)Vfilter.result*100)*Save_Res.VDebug_Valuek+Save_Res.VDebug_Valueb)/100;
//				}
////				Test_Dispvalue.Test_V = (u32)((Test_Dispvalue.Vdata*Save_Res.VDebug_Valuek + Save_Res.VDebug_Valueb)*
////				pow(10,Test_Dispvalue.Vdataraw.coefficient));
//			}
//		}
		if(Test_Dispvalue.Rdataraw.index != Rfilter.index/* && Rfilter.oldres != Test_Dispvalue.Rdataraw.num*/)//序号与上次不一样则进入队列，否则跳过
		{
			Rfilter.index = Test_Dispvalue.Rdataraw.index;//记录本次序号

//			UARTPutChar(LPC_UART3,Test_Dispvalue.Rdataraw.num%100);//打印到串口
			if(Rfilter.count == 0)
			{
				Rfilter.count ++;
				Rfilter.buffer.data[0] = Test_Dispvalue.Rdataraw.num;//第一个新数据进入队首
				HeadSort(Rfilter.buffer.data,filtersize[Save_Res.Set_Data.speed]);//新数据单独排序
				

			}else if(Rfilter.count == 1){
				Rfilter.count ++; 
				Rfilter.buffer.data[filtersize[Save_Res.Set_Data.speed]-1] = Test_Dispvalue.Rdataraw.num;//第二个新数据进入队尾
				TailSort(Rfilter.buffer.data,filtersize[Save_Res.Set_Data.speed]);//新数据单独排序
			}
			if(Rfilter.count ==2)
			{
				
				Rfilter.count = 0;
				Rfilter.sum = 0;
//				BubbleSort(Rfilter.buffer,RMAF_LENGTH_SLOW);
				for(i=filtersize[Save_Res.Set_Data.speed]/4;i<(filtersize[Save_Res.Set_Data.speed]-filtersize[Save_Res.Set_Data.speed]/4);i++)
				{
					Rfilter.sum += Rfilter.buffer.data[i];//根据队列长度取中间一半数据求和
				}
				Rfilter.result = Rfilter.sum/(filtersize[Save_Res.Set_Data.speed]/2);//求和后平均计算出滤波后数据
				if(Save_Res.Set_Data.speed != 0)//非快速模式，显示数据进行二次平均滤波
				{
					if(Rfilter.dispcount < dispfilter[Save_Res.Set_Data.speed])
					{
						Rfilter.dispbufsum += Rfilter.result;
						Rfilter.dispcount ++;
					}else{
						trip_flag = 0;//手动触发标志复位
						Rfilter.dispresult = Rfilter.dispbufsum/dispfilter[Save_Res.Set_Data.speed];
						Test_Dispvalue.Test_R = (u16)(((float)Rfilter.dispresult)*Save_Res.Debug_Value[Test_Dispvalue.Rangedisp - 1]);						
						Rfilter.dispbufsum =0 ;
						Rfilter.dispcount= 0 ;
						Rfilter.dispbufsum += Rfilter.result;
						Rfilter.dispcount++;
						
					}						
				}else{
					trip_flag = 0;//手动触发标志复位
					Rfilter.dispresult = Rfilter.result;
					Test_Dispvalue.Test_R = (u16)(((float)Rfilter.dispresult)*Save_Res.Debug_Value[Test_Dispvalue.Rangedisp - 1]);
				}
				RoffsetHandle();
				if(clearfalg == 1)//获取清零值
				{
					clearfalg = 0;
					Save_Res.rclear = Test_Dispvalue.Test_R;
				}
				if(Save_Res.clearsw == 1)//清零开关打开执行清零操作
				{
					if(Test_Dispvalue.Test_R >= Save_Res.rclear)
						Test_Dispvalue.Test_R-=Save_Res.rclear;
					else
						Test_Dispvalue.Test_R = 0;
				}
				
				
				if(u3sendflag == 1)
				{
					if(uartresdelay == 0)
					{
						RecHandle();
						u3sendflag = 0;
						g_tModS.RxCount = 0;
					}else{
						uartresdelay --;
					}
				}				
//				UARTPutChar(LPC_UART3,Rfilter.result%100);//打印到串口
			}
		}
	}
}
//==========================================================
//函数名称：Uart_Process
//函数功能：串口处理
//入口参数：无
//出口参数：无
//创建日期：2015.10.26 
//修改日期：2015.10.26 10:02
//备注说明：无
//==========================================================
u8 Uart_Process(void)
{
	vu8 i;
#if HW_UART_SUPPORT
	u8 kind=0xff;
	u8 data=0;
	u8 str[(FRAME_LEN_MAX-FRAME_LEN_MIN)+1];//收发数据缓冲
//	if(SaveData.Sys_Setup.Bus_Mode==0)//串口有效
	{
		if (ComBuf.rec.end)//接收数据结束
		{
			data=1;
			memset(str,'\0',(FRAME_LEN_MAX-FRAME_LEN_MIN+1));//清空缓冲
			{
				//memcpy(str,&ComBuf.rec.buf[PDATASTART-1],ComBuf.send.len-FRAME_LEN_MIN);//数据包
				kind=ComBuf.rec.buf[PFRAMEKIND];//命令字
				switch(kind)
				{
					case FRAME_RANGE_SET:
						
					break;
					case FRAME_SPEED:
						
					break;
					case FRAME_VRANGE_SET:
						
					break;
					case FRAME_CLEAR_OK:
						WriteString_16(380, 92+55+55+8,(uint8_t *)"CLEAR!     ",  0);
					break;
					case FRAME_CLEAR_FAIL:
						WriteString_16(380, 92+55+55+8,(uint8_t *)"CLEAR FAIL!",  0);
					break;
					case FRAME_READ_RESULT:
					{
						Test_Dispvalue.Rdataraw.index = ComBuf.rec.buf[2];
						Test_Dispvalue.Rdataraw.range = (ComBuf.rec.buf[3]&0xE0)>>5;
						Test_Dispvalue.Rdataraw.coefficient = (ComBuf.rec.buf[3]&0x1C)>>2;
						Test_Dispvalue.Rdataraw.unit = (ComBuf.rec.buf[3]&0x03);
						Test_Dispvalue.Rdataraw.num = ((u32)(ComBuf.rec.buf[4])<<8)+(ComBuf.rec.buf[5]);
						
						Test_Dispvalue.Vdataraw.index = (ComBuf.rec.buf[6]&0xF8)>>3;
						Test_Dispvalue.Vdataraw.range = (ComBuf.rec.buf[6]&0x06)>>1;
						Test_Dispvalue.Vdataraw.sign = (ComBuf.rec.buf[6]&0x01);
						Test_Dispvalue.Vdataraw.coefficient = (ComBuf.rec.buf[7]&0xE0)>>5;
						
						Test_Dispvalue.Vdataraw.num = (((u32)(ComBuf.rec.buf[7]&0x1fF)<<16)+((u32)(ComBuf.rec.buf[8])<<8)+(ComBuf.rec.buf[9]))&0XFFFFF;
						if(Save_Res.version == 0)//3560电阻量程最大为3Ω
						{
							if(Test_Dispvalue.Rdataraw.num == 0xffff || Test_Dispvalue.Rdataraw.range > 4)//采集到开路或短路数据
							{
								Test_Dispvalue.openflag = 1;
							}else{
								Test_Dispvalue.openflag = 0;
							}
						}else{
							if(Test_Dispvalue.Rdataraw.num == 0xffff)//采集到开路或短路数据
							{
								Test_Dispvalue.openflag = 1;
							}else{
								Test_Dispvalue.openflag = 0;
							}
						}
						Test_Dispvalue.Rangedisp = Test_Dispvalue.Rdataraw.range;
						Test_Dispvalue.Test_R = (u16)(((float)Test_Dispvalue.Rdataraw.num)*Save_Res.Debug_Value[Test_Dispvalue.Rangedisp - 1]);
//						RDATAFILTER_IRR();
//						RDATAFILTER();
//						VDATAFILTER();
						RoffsetHandle();
						if(Save_Res.version == 0)
						{
							if(Test_Dispvalue.Rangedisp  > 4)
								Test_Dispvalue.Rangedisp = 4;
						}
						Test_Dispvalue.Unit[0] = Test_Dispvalue.Rdataraw.unit;
						
						
//						Data_Format(Test_Dispvalue.Main_valuebuff,Test_Dispvalue.Rdataraw.num,Test_Dispvalue.Rdataraw.coefficient,5,0);
						if(Test_Dispvalue.openflag == 0)
						{
							Data_Format(Test_Dispvalue.Main_valuebuff,Test_Dispvalue.Rdataraw.num,Test_Dispvalue.Rdataraw.coefficient,5,0);
							Data_Format(Test_Dispvalue.Rvaluebuff,Test_Dispvalue.Test_R,Test_Dispvalue.Rdataraw.coefficient,5,0);
							
//							Data_Format(Test_Dispvalue.Rvaluebuff,Rfilter.result,Test_Dispvalue.Rdataraw.coefficient,5,0);
//							Data_Format(Test_Dispvalue.Rvaluebuff,Test_Dispvalue.Rdataraw.num,Test_Dispvalue.Rdataraw.coefficient,5,0);
							if(Save_Res.version == 0 && Test_Dispvalue.Rdataraw.range == 1)
								Test_Dispvalue.Main_valuebuff[5] = ' ';
						}else{
							strcpy(Test_Dispvalue.Main_valuebuff,(char*)"------");
							strcpy(Test_Dispvalue.Rvaluebuff,(char*)"------");
						}
						Test_Unit.V_Neg = !Test_Dispvalue.Vdataraw.sign;
						if(Test_Dispvalue.openflag == 0)
						{
							if(Test_Dispvalue.Vdataraw.sign)
							{
								Test_Dispvalue.Secondvaluebuff[0] = '-';
								Test_Dispvalue.Vvaluebuff[0] = '-';
							}else{
								Test_Dispvalue.Secondvaluebuff[0] = ' ';
								Test_Dispvalue.Vvaluebuff[0] = ' ';
							}
							if(Save_Res.Set_Data.speed != 2)
							{
								if(Test_Dispvalue.Vdataraw.range == 1 && Test_Dispvalue.Vdataraw.coefficient == 4)
								{
									Test_Dispvalue.Test_V = (u32)(((float)Test_Dispvalue.Vdataraw.num*10)*Save_Res.VDebug_Valuek[Test_Dispvalue.Vdataraw.range-1]+
									Save_Res.VDebug_Valueb[Test_Dispvalue.Vdataraw.range-1]);
								}else if(Test_Dispvalue.Vdataraw.range == 2 && Test_Dispvalue.Vdataraw.coefficient == 5){
									Test_Dispvalue.Test_V = (u32)(((float)Test_Dispvalue.Vdataraw.num/10)*Save_Res.VDebug_Valuek[Test_Dispvalue.Vdataraw.range-1]+
									Save_Res.VDebug_Valueb[Test_Dispvalue.Vdataraw.range-1]);
								}else if(Test_Dispvalue.Vdataraw.range == 2 && Test_Dispvalue.Vdataraw.coefficient == 3){
									Test_Dispvalue.Test_V = (u32)(((float)Test_Dispvalue.Vdataraw.num*10)*Save_Res.VDebug_Valuek[Test_Dispvalue.Vdataraw.range-1]+
									Save_Res.VDebug_Valueb[Test_Dispvalue.Vdataraw.range-1]);
								}else if(Test_Dispvalue.Vdataraw.range == 3 && Test_Dispvalue.Vdataraw.coefficient == 4){
									Test_Dispvalue.Test_V = (u32)(((float)Test_Dispvalue.Vdataraw.num/10)*Save_Res.VDebug_Valuek[Test_Dispvalue.Vdataraw.range-1]+
									Save_Res.VDebug_Valueb[Test_Dispvalue.Vdataraw.range-1]);
								}else{
									Test_Dispvalue.Test_V = (u32)(((float)Test_Dispvalue.Vdataraw.num)*Save_Res.VDebug_Valuek[Test_Dispvalue.Vdataraw.range-1]+
										Save_Res.VDebug_Valueb[Test_Dispvalue.Vdataraw.range-1]);
								}
								
								if(Test_Dispvalue.Test_V < 1000000)
								{
									Test_Dispvalue.TestVDot = 5;
								}else if(Test_Dispvalue.Test_V >= 1000000 && Test_Dispvalue.Test_V < 10000000){
									Test_Dispvalue.Test_V/=10;
									Test_Dispvalue.TestVDot = 4;
								}else if(Test_Dispvalue.Test_V >= 10000000 && Test_Dispvalue.Test_V < 100000000){
									Test_Dispvalue.Test_V/=100;
									Test_Dispvalue.TestVDot = 3;
								}
							}else{
								if(Test_Dispvalue.Vdataraw.range == 1 && Test_Dispvalue.Vdataraw.coefficient == 3)
								{
									Test_Dispvalue.Test_V = (u32)(((float)Test_Dispvalue.Vdataraw.num*10)*Save_Res.VDebug_Valuek[Test_Dispvalue.Vdataraw.range-1]+
									Save_Res.VDebug_Valueb[Test_Dispvalue.Vdataraw.range-1]/10);
								}else if(Test_Dispvalue.Vdataraw.range == 2 && Test_Dispvalue.Vdataraw.coefficient == 4){
									Test_Dispvalue.Test_V = (u32)(((float)Test_Dispvalue.Vdataraw.num/10)*Save_Res.VDebug_Valuek[Test_Dispvalue.Vdataraw.range-1]+
									Save_Res.VDebug_Valueb[Test_Dispvalue.Vdataraw.range-1]/10);
								}else if(Test_Dispvalue.Vdataraw.range == 2 && Test_Dispvalue.Vdataraw.coefficient == 2){
									Test_Dispvalue.Test_V = (u32)(((float)Test_Dispvalue.Vdataraw.num*10)*Save_Res.VDebug_Valuek[Test_Dispvalue.Vdataraw.range-1]+
									Save_Res.VDebug_Valueb[Test_Dispvalue.Vdataraw.range-1]);
								}else if(Test_Dispvalue.Vdataraw.range == 3 && Test_Dispvalue.Vdataraw.coefficient == 3){
									Test_Dispvalue.Test_V = (u32)(((float)Test_Dispvalue.Vdataraw.num/10)*Save_Res.VDebug_Valuek[Test_Dispvalue.Vdataraw.range-1]+
									Save_Res.VDebug_Valueb[Test_Dispvalue.Vdataraw.range-1]);
								}else{
									Test_Dispvalue.Test_V = (u32)(((float)Test_Dispvalue.Vdataraw.num)*Save_Res.VDebug_Valuek[Test_Dispvalue.Vdataraw.range-1]+
										Save_Res.VDebug_Valueb[Test_Dispvalue.Vdataraw.range-1]/10);
								}
//								Test_Dispvalue.Test_V = (u32)(((float)Test_Dispvalue.Vdataraw.num)*Save_Res.VDebug_Valuek[Test_Dispvalue.Vdataraw.range-1]+
//									(Save_Res.VDebug_Valueb[Test_Dispvalue.Vdataraw.range-1]/10));
								if(Test_Dispvalue.Test_V < 100000)
								{
									Test_Dispvalue.TestVDot = 4;
								}else if(Test_Dispvalue.Test_V >= 100000 && Test_Dispvalue.Test_V < 1000000){
									Test_Dispvalue.Test_V/=10;
									Test_Dispvalue.TestVDot = 3;
								}else if(Test_Dispvalue.Test_V >= 1000000 && Test_Dispvalue.Test_V < 10000000){
									Test_Dispvalue.Test_V/=100;
									Test_Dispvalue.TestVDot = 2;
								}
							}
//							Data_Format(&Test_Dispvalue.Secondvaluebuff[1],Vfilter.result/pow(10,5-Test_Dispvalue.Vdataraw.coefficient),Test_Dispvalue.Vdataraw.coefficient,6,0);	
							Data_Format(&Test_Dispvalue.Vvaluebuff[1],Test_Dispvalue.Test_V,Test_Dispvalue.TestVDot,6,0);
							
							Data_Format(&Test_Dispvalue.Secondvaluebuff[1],Test_Dispvalue.Vdataraw.num,Test_Dispvalue.Vdataraw.coefficient,6,0);
							if(Save_Res.version == 0)
							{
								Test_Dispvalue.Vvaluebuff[7] = ' ';
							}
						}else{
							strcpy(Test_Dispvalue.Secondvaluebuff,(char*)" 0.00000");
							strcpy(Test_Dispvalue.Vvaluebuff,(char*)" 0.00000");	
						}
						string_to_float((char *)Test_Dispvalue.Rvaluebuff,&Test_Dispvalue.Rdata);
						string_to_float((char *)Test_Dispvalue.Vvaluebuff,&Test_Dispvalue.Vdata);
						if(Test_Dispvalue.Vdata < 0.05 && Test_Dispvalue.Vdata > -0.05)//电压小于0.05显示0
						{
//							strcpy(Test_Dispvalue.Secondvaluebuff,(char*)" 0.00000");
							strcpy(Test_Dispvalue.Vvaluebuff,(char*)" 0.00000");
						}
						string_to_float((char *)Test_Dispvalue.Main_valuebuff,&Test_Dispvalue.Rraw);
						string_to_float((char *)Test_Dispvalue.Secondvaluebuff,&Test_Dispvalue.Vraw);
						if(Test_Dispvalue.Vdata > maxv[Save_Res.version])
						{
							Test_Dispvalue.voverflag = 1;
						}else{
							Test_Dispvalue.voverflag = 0;
						}
						if(u3sendflag == 1)
						{
							if(uartresdelay == 0)
							{
								RecHandle();
								u3sendflag = 0;
								g_tModS.RxCount = 0;
							}else{
								uartresdelay --;
							}
						}
						trip_flag = 0;//手动触发标志复位
//						VrangeSW();
//						Test_Dispvalue.Test_V = (u32)((Test_Dispvalue.Vdata*Save_Res.VDebug_Valuek + Save_Res.VDebug_Valueb)*pow(10,Test_Dispvalue.Vdataraw.coefficient));
						if(GetSystemStatus() == SYS_STATUS_USERDEBUG)
							Data_Format(&Test_Dispvalue.Vvaluebuff[1],Test_Dispvalue.Test_V,Test_Dispvalue.Vdataraw.coefficient,6,0);						
//						RDATAFILTER();
					}break;
					default:
//						for(i=0;i<6;i++)
//						{
////							Test_Dispvalue.Main_valuebuff[i]=ComBuf.rec.buf[1+i];
//		//					Test_Dispvalue.Secondvaluebuff[i]=ComBuf.rec.buf[8+i];
//							Test_Dispvalue.Rvaluebuff[i] = ComBuf.rec.buf[1+i];
//						}
//						
//						
//						for(i=0;i<8;i++)
//						{
////							Test_Dispvalue.Secondvaluebuff[i]=ComBuf.rec.buf[7+i];
//							Test_Dispvalue.Vvaluebuff[i] = ComBuf.rec.buf[7+i];							
//						}
//						Test_Dispvalue.Test_V = VBCDtoInt((int8_t *)Test_Dispvalue.Vvaluebuff);
//						if(ComBuf.rec.`buf[7]=='-')
//							Test_Unit.V_Neg=0;
//						else
//							Test_Unit.V_Neg=1;
//						//Test_Dispvalue.Test_R=
//						if(ComBuf.rec.buf[16])
//							Test_Dispvalue.Unit[0]=1;
//						else
//							Test_Dispvalue.Unit[0]=0;
//						
//						Test_Dispvalue.Rangedisp=ComBuf.rec.buf[15];
//						
//						//滤波
//						if(Test_Dispvalue.rfcount <10)
//						{
//							Test_Dispvalue.rfcount ++;
//							Test_Dispvalue.Test_R += BCDtoInt((int8_t *)Test_Dispvalue.Rvaluebuff);
//						}else{
//							Test_Dispvalue.rfcount = 0;
//							Test_Dispvalue.Test_R /= 10;
//							IntToBCD(Test_Dispvalue.Test_R,Test_Dispvalue.Dot[0],6,Test_Dispvalue.Main_valuebuff);
//							Test_Dispvalue.Test_R = 0;	
//							
//						}
					break;
				}
					

			}
			//准备接收下一帧数据sprintf
			ComBuf.rec.end=0;//接收缓冲可读标志复位
			ComBuf.rec.ptr=0;//接收指针清零
			ComBuf.respondflag = 0;
			ComBuf.commcount++;
		}
	}
	ComBuf.rec.end=0;

	
	return data;
#endif
}

//==========================================================
//函数名称：Uart_Process
//函数功能：串口处理
//入口参数：无
//出口参数：无
//创建日期：2015.10.26 
//修改日期：2015.10.26 10:02
//备注说明：无
//==========================================================
u8 Uart3_Process(void)
{
	vu8 i;
#if HW_UART_SUPPORT
	u8 kind=0xff;
	u8 data=0;
	u8 str[(FRAME_LEN_MAX-FRAME_LEN_MIN)+1];//收发数据缓冲
//	if(SaveData.Sys_Setup.Bus_Mode==0)//串口有效
	{
		if (ComBuf3.rec.end)//接收数据结束
		{data=1;
			memset(str,'\0',(FRAME_LEN_MAX-FRAME_LEN_MIN+1));//清空缓冲
			{
				memcpy(str,&ComBuf.rec.buf[PDATASTART-1],ComBuf.send.len-FRAME_LEN_MIN);//数据包
				kind=ComBuf3.rec.buf[PFRAMEKIND];//命令字
                switch(kind)
                {
                    case 0:// 设置数据
                        
                    break;
                    case 1://单次触发
                        
                    break;
                    case 2://请求发送
                        
                    break;
                    default:
                        
                    break;
                
                }
//				for(i=0;i<6;i++)
//				{
//					Test_Dispvalue.Main_valuebuff[i]=ComBuf.rec.buf[1+i];
//					Test_Dispvalue.Secondvaluebuff[i]=ComBuf.rec.buf[8+i];
//					
//				}
//				//Test_Dispvalue.Test_R=
//				if(ComBuf.rec.buf[7])
//					Test_Dispvalue.Unit[0]=1;
//				else
//					Test_Dispvalue.Unit[0]=0;
//				
//				Test_Dispvalue.Rangedisp=ComBuf.rec.buf[14];
					

			}
			//准备接收下一帧数据sprintf
			ComBuf3.rec.end=0;//接收缓冲可读标志复位
			ComBuf3.rec.ptr=0;//接收指针清零
		}
	}
//	WriteString_Big(0, 150, (uint8_t *)&ComBuf.rec.buf[1]);
	ComBuf3.rec.end=0;

//	switch(kind)
//	{
//		case FRAME_READ_RESULT://读取结果
//			//串口发送测试数据:电压(5)+电阻(6)+时间(4)+分选(1)=16字节
//			switch (GetSystemMessage())//系统信息
//			{
//				case MSG_ABORT:
//					kind=0x9B;//测试中止
//					break;
//				case MSG_PASS:
//					kind=0x91;//测试通过
//					break;
//				case MSG_HIGH:
//					kind=0x92;//上限报警
//					break;
//				case MSG_LOW:
//					kind=0x92;//下限报警
//					break;
//				default:
//					kind=0x90;//正常测试
//					break;
//			}		
//			ComBuf.send.buf[1+5+6+4]=kind;
//			ComBuf.send.begin=0;
//			ComBuf.send.len=PackStandFrame(ComBuf.send.buf , &ComBuf.send.buf[1] , 16  );
////			if(SendDataToCom()==0)//发送成功判别
////			{
////			//	Delay_1ms(100);//延时
////			//	SendDataToCom();//发送
////			}
//			break;
//		
//		case FRAME_START://启动
//			SetSystemStatus(SYS_STATUS_TEST);//系统状态-启动测试
//			break;

//		case FRAME_RESET://复位
//			//SetSystemStatus(SYS_STATUS_IDLE);//系统状态-待机
//			break;

//		case FRAME_WRITE_SN://写序列号
//			break;
//		
//		case FRAME_CLR_BOOT_NUM://清开机次数
//			break;
//		
//		case FRAME_DATA://数据帧
//			break;

//		default:
//			break;
//	}
	return data;
#endif
}

////全局变量
//u8 WaitRecTimeOver;

//==========================================================
//函数名称：PackStandFrame
//函数功能：将数据打包成帧
//入口参数：*framebuf:帧数据缓冲
//			*datbuf:数据缓冲
//			len:数据长度
//出口参数：一帧数据长度
//创建日期：2014.04.11
//修改日期：2014.04.11 10:28
//备注说明：旧的发送协议
//开始(0xAB)，电压(5) 电阻(6) 时间(4)，分选(1)，结束(0xAF)=18字节
//==========================================================
int8_t PackStandFrame(int8_t * framebuf, int8_t * datbuf, int8_t len)
{
	if(len>(SEND_LEN_MAX-2))//数据的最大长度
		len=(SEND_LEN_MAX-2);
	framebuf[0]=UART_SEND_BEGIN;//帧头
	memcpy(&framebuf[1], datbuf, len);//数据
	framebuf[len+1]=UART_SEND_END;//帧尾
	return (len+1);//返回一帧数据长度
}

const u32 UNIT_VALUE[15]=
{
	1,1E3,1E6

};
Sort_TypeDef Input_compvalue(Disp_Coordinates_Typedef *Coordinates)
{
	u8 key;
	u8 page=0;
	u8 disp_flag=1;
//	u8 index=0;
	u8 disp_cound=0;
	u8 disp_pow=0;
//	u8 input_flag=0;
//	u32 unit_c;
	u8 dot=5;//小数点
	u8 dot_num=0;
	vu8 While_flag=1;
	vu32 keynum=0;
	vu8 Disp_buff[10];
	vu8 key_count;
	vu8 dot_num1;
	vu8 del_flag=0;
	Sort_TypeDef   Sort_set;
	
//	u8 unit=0;//单位
	u32 Word;
	
	float conp_value=0;
//	float old_value;
	key_count=0;
//	old_value=SaveData.SetParameter.Nominal_value.comp_value;
//	if(SaveData.SysParameter.language==0)
//		Disp_Inputback("输入数值");//背景色
//	else
//		Disp_Inputback("INPUTNUM");//背景色
	LCD_DrawRect( Coordinates->xpos, Coordinates->ypos,Coordinates->xpos+Coordinates->lenth , 
	Coordinates->ypos+16 , Red );
	while(While_flag)
	{
		key=HW_KeyScsn();
		if(key==0xff)
		{
			keynum=0;
		}
		else
			keynum++;
		if(keynum==KEY_NUM)
		{
			disp_flag=1;
            Key_Beep();
			switch(key)
			{
				case Key_F1:
					//unit_c=UNIT_VALUE[DispBuf[disp_cound]-'0'];
						conp_value*=pow(10,5-(disp_cound-dot_num));
						//Sort_set.Num=	conp_value *unit_c/pow(10,dot);
						Sort_set.Num=conp_value;
						Sort_set.Dot=dot;
						if(page==0)
							Sort_set.Unit=0;
						else
							Sort_set.Unit=4;
							
						While_flag=0;
					break;
				case Key_F2:
						conp_value*=pow(10,5-(disp_cound-dot_num));
						//Sort_set.Num=	conp_value *unit_c/pow(10,dot);
						Sort_set.Num=conp_value;
						Sort_set.Dot=dot;
						if(page==0)
							Sort_set.Unit=1;
						else
							Sort_set.Unit=5;
						While_flag=0;
					break;
				case Key_F3:
					conp_value*=pow(10,5-(disp_cound-dot_num));
						//Sort_set.Num=	conp_value *unit_c/pow(10,dot);
						Sort_set.Num=conp_value;
						Sort_set.Dot=dot;
						if(page==0)
							Sort_set.Unit=2;
						else
							Sort_set.Unit=6;
						While_flag=0;
					break;
				case Key_F4:
						if(page==0)
						{
						conp_value*=pow(10,5-(disp_cound-dot_num));
						//Sort_set.Num=	conp_value *unit_c/pow(10,dot);
						Sort_set.Num=conp_value;
						Sort_set.Dot=dot;
						Sort_set.Unit=3;
						While_flag=0;
						}
					break;
				case Key_F5:
					if(page)
						page=0;
					else
						page=1;
				
					break;
				case Key_Disp:
					SetSystemStatus(SYS_STATUS_TEST);
					While_flag=0;
					Sort_set.Updata_flag=0;
				break;
				case Key_SETUP:
					While_flag=0;
					Sort_set.Updata_flag=0;
				SetSystemStatus(SYS_STATUS_SETUPTEST);
				break;
				case Key_FAST:
				break;
				case Key_LEFT:
				break;
				case Key_RIGHT:
				break;
				case Key_UP:
				break;
				case Key_DOWN:
				break;
				case Key_NUM1:
					if(key_count<NUM_LENTH)
					{
						Disp_buff[key_count]='1';
						key_count++;
						Word=Disp_buff[disp_cound]-'0';
						conp_value=conp_value*10+Word;
						disp_pow++;
						disp_cound++;
						if(dot_num==0)
							dot--;
							
					}
				break;
				case Key_NUM2:
					if(key_count<NUM_LENTH)
					{
						Disp_buff[key_count]='2';
						key_count++;
						Word=Disp_buff[disp_cound]-'0';
						conp_value=conp_value*10+Word;
						disp_pow++;
						disp_cound++;
						if(dot_num==0)
							dot--;
					}
				break;
				case Key_NUM3:
					if(key_count<NUM_LENTH)
					{
						Disp_buff[key_count]='3';
						key_count++;
						Word=Disp_buff[disp_cound]-'0';
						conp_value=conp_value*10+Word;
						disp_pow++;
						disp_cound++;
						if(dot_num==0)
							dot--;
					}
				break;
				case Key_NUM4:
					if(key_count<NUM_LENTH)
					{
						Disp_buff[key_count]='4';
						key_count++;
						Word=Disp_buff[disp_cound]-'0';
						conp_value=conp_value*10+Word;
						disp_pow++;
						disp_cound++;
						if(dot_num==0)
							dot--;
					}
					
				break;
				case Key_NUM5:
					if(key_count<NUM_LENTH)
					{
						Disp_buff[key_count]='5';
						key_count++;
						Word=Disp_buff[disp_cound]-'0';
						conp_value=conp_value*10+Word;
						disp_pow++;
						disp_cound++;
						if(dot_num==0)
							dot--;
					}
				break;
				case Key_NUM6:
					if(key_count<NUM_LENTH)
					{
						Disp_buff[key_count]='6';
						key_count++;
						Word=Disp_buff[disp_cound]-'0';
						conp_value=conp_value*10+Word;
						disp_pow++;
						disp_cound++;
						if(dot_num==0)
							dot--;
					}
				break;
				case Key_NUM7:
					if(key_count<NUM_LENTH)
					{
						Disp_buff[key_count]='7';
						
						Word=Disp_buff[disp_cound]-'0';
						key_count++;
						conp_value=conp_value*10+Word;
						disp_pow++;
						disp_cound++;
						if(dot_num==0)
							dot--;
					}
				break;
				case Key_NUM8:
					if(key_count<NUM_LENTH)
					{
						Disp_buff[key_count]='8';
						key_count++;
						Word=Disp_buff[disp_cound]-'0';
						conp_value=conp_value*10+Word;
						disp_pow++;
						disp_cound++;
						if(dot_num==0)
							dot--;
					}
				break;
				case Key_NUM9:
					if(key_count<NUM_LENTH)
					{
						Disp_buff[key_count]='9';
						key_count++;
						Word=Disp_buff[disp_cound]-'0';
						conp_value=conp_value*10+Word;
						disp_pow++;
						disp_cound++;
						if(dot_num==0)
							dot--;
					}
				break;
				case Key_NUM0:
					if(key_count<NUM_LENTH)
					{
						Disp_buff[key_count]='0';
						key_count++;
						Word=Disp_buff[disp_cound]-'0';
						conp_value=conp_value*10+Word;
						disp_pow++;
						disp_cound++;
						if(dot_num==0)
							dot--;
					}
				break;
				case Key_DOT:
					if(dot_num==0)
					{
						dot_num1=key_count;
						Disp_buff[key_count]='.';
						dot_num=1;
						key_count++;
						disp_cound++;
					}
					break;
				case Key_BACK:
					if(key_count>0)
					{	key_count--;
						Disp_buff[key_count]=' ';
						if(dot_num1==key_count)
						{
							dot_num=0;
							dot_num1=0;
							
						}
						del_flag=1;
					
					}
//					else
//					{
//						if(Disp_buff[key_count]==0)
//								Disp_buff[key_count]='-';
//							else if(Disp_buff[key_count]=='-')
//								Disp_buff[key_count]='+';
//							else
//								Disp_buff[key_count]='-';
//							key_count++;
//					}
				break;
				case Key_LOCK:
				break;
				case Key_BIAS:
				break;
				case Key_REST:
				break;
				case Key_TRIG:
				break;
				default:
				break;
					
			}
			if(disp_flag)
			{
				disp_flag=0;
				Disp_button_Num_Input(page);
				Colour.Fword=White;
				Colour.black=Red;
				if(del_flag)
				{
					PutChar( Coordinates->xpos+(key_count+1)*10, Coordinates->ypos, 
					Disp_buff[key_count], Colour.Fword, Colour.black ); 
					del_flag=0;
					
				}
				else if(key_count>0)				
					PutChar( Coordinates->xpos+key_count*10, Coordinates->ypos, 
					Disp_buff[key_count-1], Colour.Fword, Colour.black ); 
			
			}
				
				
			
			
		}
			
	}
		
		
	return Sort_set;
}
	
	

//数字键输入显示
Sort_TypeDef Disp_NumKeyboard_Set(Disp_Coordinates_Typedef *Coordinates )
{
	vu8 While_flag=1;
	vu8 Disp_buff[10];
	vu8 key,i;
	vu8 dispflag=1;
	vu8 dot_num=0,dot_num1=0;
//	vu8 page=0;
	vu32 keynum=0;
	vu8 key_count=0;
//	vu32 Num[6]={1,10,100,1e3,1e4,1e5};
	Sort_TypeDef   Sort_set;
	Sort_set.Dot=0;
	Sort_set.Num=0;
	Sort_set.Unit=0;
	Sort_set.Num=0;
	for(i=0;i<6;i++)
	Disp_buff[i]=' ';
	Disp_buff[7]=0;
	
	while(While_flag)
	{
		key=HW_KeyScsn();
		if(key==0xff)
		{
			keynum=0;
		}
		else
			keynum++;
		if(keynum==KEY_NUM)
		{
			dispflag=1;
            Key_Beep();
			switch(key)
			{
				case Key_F1:
					if(GetSystemStatus() == SYS_STATUS_SYSSET)
					{
						Sort_set.sign = 0;
						if(Test_Dispvalue.Rangedisp < 4)
						{
							Sort_set.Unit=0;
						}else{
							Sort_set.Unit=1;
						}
					}else{
						Sort_set.Unit=0;
					}
					
					While_flag=0;
					if(key_count<NUM_LENTH)
					{
						if(dot_num==0)
						{
							if(key_count>0)
							{
								Disp_buff[key_count]='.';
								dot_num1=key_count;
								key_count++;
							
							
							}
							dot_num++;
						}
					
					}
						
					
					Sort_set.Updata_flag=1;
					
				break;
				case Key_F2:
					if(GetSystemStatus() == SYS_STATUS_SYSSET)
					{
						Sort_set.sign = 1;
						if(Test_Dispvalue.Rangedisp < 4)
						{
							Sort_set.Unit=0;
						}else{
							Sort_set.Unit=1;
						}
					}else{
						Sort_set.Unit=1;
					}
					While_flag=0;
					if(key_count<NUM_LENTH)
					{
						if(dot_num==0)
						{
							if(key_count>0)
							{
								Disp_buff[key_count]='.';
								dot_num1=key_count;
								key_count++;
							
							
							}
							dot_num++;
						}
					
					}
						
					
					Sort_set.Updata_flag=1;
					
				break;
				case Key_F3:
					
				break;
				case Key_F4:
					
				break;
				case Key_F5:
					
				break;
				case Key_Disp:
					SetSystemStatus(SYS_STATUS_TEST);
					While_flag=0;
					Sort_set.Updata_flag=0;
				break;
				case Key_SETUP:
					While_flag=0;
					Sort_set.Updata_flag=0;
				SetSystemStatus(SYS_STATUS_SETUPTEST);
				break;
				case Key_FAST:
				break;
				case Key_LEFT:
				break;
				case Key_RIGHT:
				break;
				case Key_UP:
				break;
				case Key_DOWN:
				break;
				case Key_NUM1:
					if(key_count<NUM_LENTH)
					{
						Disp_buff[key_count]='1';
						
						
							
						
						key_count++;
							
					}
				break;
				case Key_NUM2:
					if(key_count<NUM_LENTH)
					{
						Disp_buff[key_count]='2';
						key_count++;
					}
				break;
				case Key_NUM3:
					if(key_count<NUM_LENTH)
					{
						Disp_buff[key_count]='3';
						key_count++;
					}
				break;
				case Key_NUM4:
					if(key_count<NUM_LENTH)
					{
						Disp_buff[key_count]='4';
						key_count++;
					}
					
				break;
				case Key_NUM5:
					if(key_count<NUM_LENTH)
					{
						Disp_buff[key_count]='5';
						key_count++;
					}
				break;
				case Key_NUM6:
					if(key_count<NUM_LENTH)
					{
						Disp_buff[key_count]='6';
						key_count++;
					}
				break;
				case Key_NUM7:
					if(key_count<NUM_LENTH)
					{
						Disp_buff[key_count]='7';
						key_count++;
					}
				break;
				case Key_NUM8:
					if(key_count<NUM_LENTH)
					{
						Disp_buff[key_count]='8';
						key_count++;
					}
				break;
				case Key_NUM9:
					if(key_count<NUM_LENTH)
					{
						Disp_buff[key_count]='9';
						key_count++;
					}
				break;
				case Key_NUM0:
					if(key_count<NUM_LENTH)
					{
						Disp_buff[key_count]='0';
						key_count++;
					}
				break;
				case Key_DOT:
					
					if(key_count<NUM_LENTH&&key_count>0)
					{
						if(dot_num==0)
						{
							if(key_count>0)
							{
								Disp_buff[key_count]='.';
								dot_num1=key_count;
								key_count++;
							
							
							}
							dot_num++;
						}
					}
//					else 
//					{
//						if(Disp_buff[key_count]==0)
//							Disp_buff[key_count]='-';
//						else if(Disp_buff[key_count]=='-')
//							Disp_buff[key_count]='+';
//						else
//							Disp_buff[key_count]='-';
//						key_count++;
//							
//					
//					
//					
//					
//					}
				break;
				case Key_BACK:
					if(key_count>0)
					{	key_count--;
						Disp_buff[key_count]=' ';
						if(dot_num1==key_count)
						{
							dot_num=0;
							dot_num1=0;
							
						}
					
					}
					else
					{
						if(Disp_buff[key_count]==0)
								Disp_buff[key_count]='-';
							else if(Disp_buff[key_count]=='-')
								Disp_buff[key_count]='+';
							else
								Disp_buff[key_count]='-';
							key_count++;
					}
				break;
				case Key_LOCK:
				break;
				case Key_BIAS:
				break;
				case Key_REST:
				break;
				case Key_TRIG:
				break;
				default:
				break;
					
			}
		
		
		}
		if(dispflag)
		{
			dispflag=0;
			LCD_DrawRect( Coordinates->xpos, Coordinates->ypos,Coordinates->xpos+Coordinates->lenth , Coordinates->ypos+16 , Red );
			Colour.Fword=White;
			Colour.black=Red;
			WriteString_16(Coordinates->xpos, Coordinates->ypos, Disp_buff,  0);
			//dispflag=0;
		}
	
	}
	for(i=key_count;i<NUM_LENTH;i++)
		Disp_buff[i]='0';
	for(i=0;i<NUM_LENTH;i++)
	{
		if(Disp_buff[0]>='0'&&(Disp_buff[0]<='9'))
		{
			if(Disp_buff[i]>='0'&&(Disp_buff[i]<='9'))
			{
			
				if(dot_num1>i)
				{
					Sort_set.Num*=10;
					Sort_set.Num+=Disp_buff[i]-'0';
				
				}
				else
				{
					Sort_set.Num*=10;
					Sort_set.Num+=Disp_buff[i]-'0';
				
				
				}
			}
			
			
			//Sort_set.Num+=(Disp_buff[key_count-1]-'0');
		
		
		}
	
	
	
	}
	Sort_set.Dot=dot_num1;
//	if(Disp_buff[0]>='0'&&(Disp_buff[0]<'9'))
//		{
//			if(Disp_buff[key_count-1]!='.')		
//			{
//				Sort_set.Num*=Num[key_count-dot_num-1];
//				Sort_set.Num+=(Disp_buff[key_count-1]-'0');//*Num[key_count-dot_num-1];
//				
//			}				
//			//*(Disp_buff[key_count-1]-'0'))+=Num[key_count-dot_num-1];
//			else
//			{
//			
//			}
//		
//		
//		}
//			
//		else 
//			;//(Disp_buff[key_count-1]-'0')*Sort_set.Num+=Num[key_count-dot_num-2];
	return Sort_set;

}


//电阻设置
Sort_TypeDef Disp_Set_Num(Disp_Coordinates_Typedef *Coordinates)
{
	double val;
	Sort_TypeDef Sort_num,Sort_num1;
	Disp_button_Num_time();
	Sort_num=Disp_NumKeyboard_Set(Coordinates);
	Sort_num.Dot = 5- Sort_num.Dot;
	val = (((double)Sort_num.Num)/((double)pow(10,Sort_num.Dot)))
		*1000*((double)pow(1000,Sort_num.Unit));
	if(Save_Res.version == 0)
	{
		if(val > 3000000){
			Sort_num.Num = 30000;
			Sort_num.Dot = 4;
			Sort_num.Unit = 1;
		}
	}else{
		if(val > 3000000000){
			Sort_num.Num = 30000;
			Sort_num.Dot = 1;
			Sort_num.Unit = 1;
		}
	}
//	Sort_num1=Time_Set_Cov(&Sort_num);
//	if(Sort_num1.Updata_flag==0)
//	{
//		Sort_num1.Dot=0;
//		Sort_num1.Num=0;
//		Sort_num1.Unit=0;
//	
//	}
		
	return Sort_num;

}




Sort_TypeDef Disp_NumKeyboard_SetV(Disp_Coordinates_Typedef *Coordinates )
{
	vu8 While_flag=1;
	vu8 Disp_buff[10];
	vu8 key,i;
	vu8 dispflag=1;
	vu8 dot_num=0,dot_num1=0;
//	vu8 page=0;
	vu32 keynum=0;
	vu8 key_count=0;
//	vu32 Num[6]={1,10,100,1e3,1e4,1e5};
	Sort_TypeDef   Sort_set;
	Sort_set.Dot=0;
	Sort_set.Num=0;
	Sort_set.Unit=0;
	Sort_set.Num=0;
	for(i=0;i<7;i++)
	Disp_buff[i]=' ';
	Disp_buff[7]=0;
	
	while(While_flag)
	{
		key=HW_KeyScsn();
		if(key==0xff)
		{
			keynum=0;
		}
		else
			keynum++;
		if(keynum==KEY_NUM)
		{
			dispflag=1;
            Key_Beep();
			switch(key)
			{
				case Key_F1:
					
					Sort_set.Unit=0;
					
					While_flag=0;
					if(key_count<VNUM_LENTH)
					{
						if(dot_num==0)
						{
							if(key_count>0)
							{
								Disp_buff[key_count]='.';
								dot_num1=key_count;
								key_count++;
							
							
							}
							dot_num++;
						}
					
					}
						
					
					Sort_set.Updata_flag=1;
					
				break;
				case Key_F2:
					Sort_set.Unit=1;
					
					While_flag=0;
					if(key_count<VNUM_LENTH)
					{
						if(dot_num==0)
						{
							if(key_count>0)
							{
								Disp_buff[key_count]='.';
								dot_num1=key_count;
								key_count++;
							
							
							}
							dot_num++;
						}
					
					}
						
					
					Sort_set.Updata_flag=1;
					
				break;
				case Key_F3:
					
				break;
				case Key_F4:
					
				break;
				case Key_F5:
					
				break;
				case Key_Disp:
					SetSystemStatus(SYS_STATUS_TEST);
					While_flag=0;
					Sort_set.Updata_flag=0;
				break;
				case Key_SETUP:
					While_flag=0;
					Sort_set.Updata_flag=0;
				SetSystemStatus(SYS_STATUS_SETUPTEST);
				break;
				case Key_FAST:
				break;
				case Key_LEFT:
				break;
				case Key_RIGHT:
				break;
				case Key_UP:
				break;
				case Key_DOWN:
				break;
				case Key_NUM1:
					if(key_count<VNUM_LENTH)
					{
						Disp_buff[key_count]='1';
						
						
							
						
						key_count++;
							
					}
				break;
				case Key_NUM2:
					if(key_count<VNUM_LENTH)
					{
						Disp_buff[key_count]='2';
						key_count++;
					}
				break;
				case Key_NUM3:
					if(key_count<VNUM_LENTH)
					{
						Disp_buff[key_count]='3';
						key_count++;
					}
				break;
				case Key_NUM4:
					if(key_count<VNUM_LENTH)
					{
						Disp_buff[key_count]='4';
						key_count++;
					}
					
				break;
				case Key_NUM5:
					if(key_count<VNUM_LENTH)
					{
						Disp_buff[key_count]='5';
						key_count++;
					}
				break;
				case Key_NUM6:
					if(key_count<VNUM_LENTH)
					{
						Disp_buff[key_count]='6';
						key_count++;
					}
				break;
				case Key_NUM7:
					if(key_count<VNUM_LENTH)
					{
						Disp_buff[key_count]='7';
						key_count++;
					}
				break;
				case Key_NUM8:
					if(key_count<VNUM_LENTH)
					{
						Disp_buff[key_count]='8';
						key_count++;
					}
				break;
				case Key_NUM9:
					if(key_count<VNUM_LENTH)
					{
						Disp_buff[key_count]='9';
						key_count++;
					}
				break;
				case Key_NUM0:
					if(key_count<VNUM_LENTH)
					{
						Disp_buff[key_count]='0';
						key_count++;
					}
				break;
				case Key_DOT:
					
					if(key_count<VNUM_LENTH&&key_count>0)
					{
						if(dot_num==0)
						{
							if(key_count>0)
							{
								Disp_buff[key_count]='.';
								dot_num1=key_count;
								key_count++;
							
							
							}
							dot_num++;
						}
					}
//					else 
//					{
//						if(Disp_buff[key_count]==0)
//							Disp_buff[key_count]='-';
//						else if(Disp_buff[key_count]=='-')
//							Disp_buff[key_count]='+';
//						else
//							Disp_buff[key_count]='-';
//						key_count++;
//							
//					
//					
//					
//					
//					}
				break;
				case Key_BACK:
					if(key_count>0)
					{	key_count--;
						Disp_buff[key_count]=' ';
						if(dot_num1==key_count)
						{
							dot_num=0;
							dot_num1=0;
							
						}
					
					}
					else
					{
						if(Disp_buff[key_count]==0)
								Disp_buff[key_count]='-';
							else if(Disp_buff[key_count]=='-')
								Disp_buff[key_count]='+';
							else
								Disp_buff[key_count]='-';
							key_count++;
					}
				break;
				case Key_LOCK:
				break;
				case Key_BIAS:
				break;
				case Key_REST:
				break;
				case Key_TRIG:
				break;
				default:
				break;
					
			}
		
		
		}
		if(dispflag)
		{
			dispflag=0;
			LCD_DrawRect( Coordinates->xpos, Coordinates->ypos,Coordinates->xpos+Coordinates->lenth , Coordinates->ypos+16 , Red );
			Colour.Fword=White;
			Colour.black=Red;
			WriteString_16(Coordinates->xpos, Coordinates->ypos, Disp_buff,  0);
			//dispflag=0;
		}
	
	}
	for(i=key_count;i<VNUM_LENTH;i++)
		Disp_buff[i]='0';
	for(i=0;i<VNUM_LENTH;i++)
	{
		if(Disp_buff[0]>='0'&&(Disp_buff[0]<='9'))
		{
			if(Disp_buff[i]>='0'&&(Disp_buff[i]<='9'))
			{
			
				if(dot_num1>i)
				{
					Sort_set.Num*=10;
					Sort_set.Num+=Disp_buff[i]-'0';
				
				}
				else
				{
					Sort_set.Num*=10;
					Sort_set.Num+=Disp_buff[i]-'0';
				
				
				}
			}
			
			
			//Sort_set.Num+=(Disp_buff[key_count-1]-'0');
		
		
		}
	
	
	
	}
	Sort_set.Dot=dot_num1;
//	if(Disp_buff[0]>='0'&&(Disp_buff[0]<'9'))
//		{
//			if(Disp_buff[key_count-1]!='.')		
//			{
//				Sort_set.Num*=Num[key_count-dot_num-1];
//				Sort_set.Num+=(Disp_buff[key_count-1]-'0');//*Num[key_count-dot_num-1];
//				
//			}				
//			//*(Disp_buff[key_count-1]-'0'))+=Num[key_count-dot_num-1];
//			else
//			{
//			
//			}
//		
//		
//		}
//			
//		else 
//			;//(Disp_buff[key_count-1]-'0')*Sort_set.Num+=Num[key_count-dot_num-2];
	return Sort_set;

}

//电压设置
Sort_TypeDef Disp_Set_NumV(Disp_Coordinates_Typedef *Coordinates)
{
	double val;
	Sort_TypeDef Sort_num,Sort_num1;
	Disp_button_Num_Freq();
	Sort_num=Disp_NumKeyboard_SetV(Coordinates);
	Sort_num.Dot = 6- Sort_num.Dot;
	val = ((double)Sort_num.Num)/((double)pow(10,Sort_num.Dot));
	if(val > maxv[Save_Res.version]){
		Sort_num.Num = maxvdisp[Save_Res.version];
		Sort_num.Dot = maxvdot[Save_Res.version];
		Sort_num.Unit = 0;
	}
//	Sort_num1=Time_Set_Cov(&Sort_num);
//	if(Sort_num1.Updata_flag==0)
//	{
//		Sort_num1.Dot=0;
//		Sort_num1.Num=0;
//		Sort_num1.Unit=0;
//	
//	}
		
	return Sort_num;

}

//电压设置
Sort_TypeDef Disp_Set_CompNum(Disp_Coordinates_Typedef *Coordinates)
{
	Sort_TypeDef Sort_num,Sort_num1;
	Disp_button_Num_Freq();
	Sort_num=Disp_NumKeyboard_SetV(Coordinates);
	Sort_num1=Input_Set_Cov(&Sort_num);
	if(Sort_num1.Updata_flag==0)
	{
		Sort_num1.Dot=0;
		Sort_num1.Num=0;
		Sort_num1.Unit=0;
	
	}
		
	return Sort_num1;

}
Sort_TypeDef Disp_Set_InputNum(Disp_Coordinates_Typedef *Coordinates)
{
	Sort_TypeDef Sort_num1;
	Disp_button_Num_Input(0);
	Sort_num1=Input_compvalue(Coordinates);
	//if(SaveData.Limit_Tab.Mode==0)
	Sort_num1=Input_Set_Cov(&Sort_num1);
	//else
	//Sort_num1=Input_Set_CovPre(&Sort_num1);
	if(Sort_num1.Updata_flag==0)
	{
		Sort_num1.Dot=0;
		Sort_num1.Num=0;
		Sort_num1.Unit=0;
	
	}
		
	return Sort_num1;
	
	

}
Sort_TypeDef Disp_Set_InputpreNum(Disp_Coordinates_Typedef *Coordinates)
{
	Sort_TypeDef Sort_num1;
	Disp_button_Num_Input(0);
	Sort_num1=Input_compvalue(Coordinates);//Input_Set_Cov
	if(SaveData.Limit_Tab.Mode==0)
		Sort_num1=Input_Set_Cov(&Sort_num1);
	else
		Sort_num1=Input_Set_CovPre(&Sort_num1);
	if(Sort_num1.Updata_flag==0)
	{
		Sort_num1.Dot=0;
		Sort_num1.Num=0;
		Sort_num1.Unit=0;
	
	}
		
	return Sort_num1;
	
	

}


vu16 Freq_Set_Num(Disp_Coordinates_Typedef *Coordinates)//频率设置
{
	Sort_TypeDef Sort_num;
//	vu8 i;
	vu16 num;
	Disp_button_Num_Freq();
	Sort_num=Disp_NumKeyboard_Set(Coordinates);
	num= Debug_Value(&Sort_num);
	//Sort_num1=Freq_Set_Cov(&Sort_num);
	
	
		
	return num;
	
}
vu8 Avg_Set_Num(Disp_Coordinates_Typedef *Coordinates)//平均数设置
{
	Sort_TypeDef Sort_num,Sort_num1;
	vu8 i;
	vu8 num;
	Disp_button_Num_Avg();
	Sort_num=Disp_NumKeyboard_Set(Coordinates);
	Sort_num1=Freq_Set_Cov(&Sort_num);
	if(Sort_num1.Updata_flag==0)
	{
		Sort_num1.Dot=0;
		Sort_num1.Num=0;
		Sort_num1.Unit=0;
	
	}
	if(Sort_num1.Dot==0)
	{
		if(Sort_num1.Num>32)
			Sort_num1.Num=32;
	
	} else
	if(Sort_num1.Dot==1)
	{
		for(i=0;i<5;i++)
		{
			if(Sort_num1.Num>0&&Sort_num1.Num<=9)
				break;
			else
				Sort_num1.Num/=10;
		
		
		}
	
	}else if(Sort_num1.Dot==2)
	{
		for(i=0;i<5;i++)
		{
			if(Sort_num1.Num>10&&Sort_num1.Num<=99)
				break;
			else
				Sort_num1.Num/=10;
		
		
		}
		if(Sort_num1.Num>32)
			Sort_num1.Num=32;
	
	
	
	}else
	{
		Sort_num1.Num=32;
	
	
	}
	num=Sort_num1.Num;
	if(num==0)
		num=1;
		
	return num;
	
}
void Set_daot(vu8 *buff,vu8 dot)
{
	vu8 i;
	for(i=0;i<dot;i++)
	{
		buff[5-i]=buff[5-i-1];
	
	
	}
	if(dot==0)
		buff[5]=' ';
	else
	buff[5-dot]='.';
	buff[6]=0;


}
int8_t V_Test_Comp(double value)
{
	vu8 res;	
	static double data,upper,lower;
	if(Test_Dispvalue.openflag == 0)
		data = value;
	else
		data = 0;
	upper = ((double)Save_Res.Set_Data.V_high.Num)/((double)pow(10,Save_Res.Set_Data.V_high.Dot));
	lower = ((double)Save_Res.Set_Data.V_low.Num)/((double)pow(10,Save_Res.Set_Data.V_low.Dot));
	
	if(data>upper)
		res=VH_FAIL;
	else if(data<lower)
		res=VL_FAIL;
	else
		res=ALL_PASS;
	return res;
//	if(value>Save_Res.Set_Data.V_high.mid_data)
//		data=VH_FAIL;
//	else if(value<Save_Res.Set_Data.V_low.mid_data || negvalm == 1)
//		data=VL_FAIL;
//	else
//		data=ALL_PASS;
//	return data;
		


}
int8_t R_Test_Comp(double value)
{
	vu8 res;	
	static double data,upper,lower;
	data = value*1000*((double)pow(1000,Test_Dispvalue.Unit[0]));
	upper = (((double)Save_Res.Set_Data.High_Res.Num)/((double)pow(10,Save_Res.Set_Data.High_Res.Dot)))
		*1000*((double)pow(1000,Save_Res.Set_Data.High_Res.Unit));
	lower = (((double)Save_Res.Set_Data.Res_low.Num)/((double)pow(10,Save_Res.Set_Data.Res_low.Dot)))
		*1000*((double)pow(1000,Save_Res.Set_Data.Res_low.Unit));
	
	
	if(data>upper)
		res=RH_FAIL;
	else if(data<lower)
		res=RL_FAIL;
	else
		res=ALL_PASS;
	return res;

}
void Comp_prompt(int8_t value)
{	
	if(Test_Dispvalue.openflag == 0)
	{
		if(value==ALL_PASS)
		{
			Pass_Led();	
		}else{
			Fail_led();
		}
		switch(Save_Res.Set_Data.beep)
		{
			case 0://蜂鸣器关闭
				Beep_Off();
				break;
			case 1://合格讯响
				if(value==ALL_PASS)
					Beep_on();
				else
					Beep_Off();
				break;
			case 2://不合格讯响
				if(Save_Res.Set_Data.openbeep==1)
				{
					if(value==ALL_PASS)
						Beep_Off();
					else
						Beep_on();
				}else{
					if(nodisp_v_flag == 1)
					{
						Beep_Off();
					}else{
						if(value==ALL_PASS)
							Beep_Off();
						else
							Beep_on();
					}
				}
					
				
				break;
			default:
				Beep_Off();
				break;
		
		}
	}else if(Test_Dispvalue.openflag == 1){
		if(Save_Res.Set_Data.openbeep==1/* || nodisp_v_flag == 0*/)
		{
			Fail_led();
			if(Save_Res.Set_Data.beep==2)//不合格讯响
			{
				Beep_on();
			}else{
				Beep_Off();
			}				
		}else{
			No_Comp();
		}
	}
	
//	if(value==ALL_PASS && Test_Dispvalue.openflag == 0)
//	{
//		Pass_Led();	
//	}
//	else if(Test_Dispvalue.openflag == 1)
//	{
//		if(Save_Res.Set_Data.openbeep==1/* || nodisp_v_flag == 0*/)
//		{
//			Fail_led();	
//		}else{
//			No_Comp();
//		}
//	}
	
	



}
void Test_Comp(All_Compvalue_Typedef *pt)
{
	float value;
	float data;
	vu8 i;
	for(i=0;i<5;i++)
	{
		data=pt->all[0].buff[i];
		value+=(pow(10,4-i)*data);//从下位机接收来的数据
	}
	value=value*pow(1000,pt->all[0].Unit);
	value/=(pow(10,pt->all[0].Dot));
	
	if(SaveData.Limit_Tab.Mode)//百分比比较
	{
		for(i=0;i<9;i++)
		{
			if(SaveData.Limit_Tab.Comp_Value[i].low.Num!=0)
			{
				if(value>Comp_Testvalue.comp_highvalue[i])//大于上限
					Count_buff[10]++;
				else if(value<Comp_Testvalue.comp_lowvalue[i])//小于下限 加合格标志位
									//计数
					Count_buff[10]++;
				else Count_buff[i]++;
			
			
			}
		
		
		}
	
	
	
	}
//	else
//	{
//	
//	
//	}//ABS比较

}//Comp_Testvalue  Comp_Testvalue
float Inttofloat(Sort_TypeDef *pt)//int转换为float  INT包含小数点和单位
{
	float value;
//	vu8 i;
	value=pt->Num;
	value=value*pow(1000,pt->Unit);
	value/=pow(10,pt->Dot);
	return value;
}
void Set_Compbcd_float(void)//把设置比较数据转换为float数据  把这个数据与标称值进行运算，得到
	//可以直接比较的值  这个分为2种，一种是ABS一种是%
{
	vu8 i;
	float value;
	for(i=0;i<9;i++)
	{
		Comp_Testvalue.comp_highvalue[i]=Inttofloat( &SaveData.Limit_Tab.Comp_Value[i].high);
		Comp_Testvalue.comp_lowvalue[i]=Inttofloat( &SaveData.Limit_Tab.Comp_Value[i].low);
	
	
	}
	Comp_Testvalue.comp_highvalue[9]=Inttofloat( &SaveData.Limit_Tab.Comp_Value_2nd.high);
	Comp_Testvalue.comp_lowvalue[9]=Inttofloat( &SaveData.Limit_Tab.Comp_Value_2nd.low);
	value=Inttofloat(&SaveData.Limit_Tab.Nom);
	if(SaveData.Limit_Tab.Mode)//百分比比较
	{
		for(i=0;i<9;i++)
		{
			Comp_Testvalue.comp_highvalue[i]=(100000+Comp_Testvalue.comp_highvalue[i])*value;
			Comp_Testvalue.comp_highvalue[i]/=100000;
			Comp_Testvalue.comp_lowvalue[i]=(100000-Comp_Testvalue.comp_lowvalue[i])*value;
			Comp_Testvalue.comp_lowvalue[i]/=100000;
			
		}
	
	
	
	}
	else
	{
		
		for(i=0;i<9;i++)
		{
			Comp_Testvalue.comp_highvalue[i]+=value;
			Comp_Testvalue.comp_lowvalue[i]-=value;
			
		}
		//Comp_Testvalue.comp_highvalue[9]=
		
	
	
	}//ABS比较
	if(Save_Res.Set_Data.dispvr > 2)
		Save_Res.Set_Data.dispvr=0;



}
int32_t Input_int(int8_t *pt)
{
	u32 value=0;
	u8 i,j=0;
//	u8 dot=0;
	for(i=0;i<5;i++)
	{
		if(*(pt+i+1)>='0')
		{
			value*=10;
			value+=*(pt+i+1)-'0';
			j++;
			
		}
//		else
//			dot=4-i;
	
	}
	return value;


}
//电阻BCD转换为INT
int32_t BCDtoInt(int8_t *pt)
{
	u32 value=0;
	u8 i,j=0;
	u8 dot=0;
	if(*(pt)=='-')
	{
		value=0xfffffff;
		return value;
	}
	for(i=0;i<6;i++)
	{
		if(*(pt+i)>='0')
		{
			value*=10;
			value+=*(pt+i)-'0';
			j++;
			
		}
		else
			dot=5-i;
	
	}
	Test_Dispvalue.Dot[0] = dot;
//	if(Test_Dispvalue.Unit[0])
//		value*=1e6;
//	else
//		value*=1e3;
//	value/=pow(10,dot);
	
	return value;
}

//电阻BCD转换为INT
void IntToBCD(u32 r,u8 dot,u8 len,char *pt)
{
	u8 i=len;
	do
	{
		i--;
		if(i != dot-1)
		{
			*(pt+i) = r%10+0x30;
			r/=10;
		}else{
			*(pt+i) = '.';
		}				
	}while(i!=0);
//	if(Test_Dispvalue.Unit[0])
//		value*=1e6;
//	else
//		value*=1e3;
//	value/=pow(10,dot);
}

//电压BCD转换为INT
int32_t VBCDtoInt(int8_t *pt)
{
	u32 value=0;
	
	u8 i,j=0;
	u8 dot=0;
	for(i=1;i<8;i++)
	{
		if(*(pt+1+i)>='0')
		{
			value*=10;
			value+=*(pt+1+i)-'0';
			j++;
			
		}
		else
			dot=6-i;
		
	
	}
	Test_Unit.V_dot=dot;
	value*=1000;
	value/=pow(10,dot);
	if(*pt=='-')
		Test_Unit.V_Neg=0;
	else
		Test_Unit.V_Neg=1;
	
	return value;
}
void V_BCD_Int(int32_t data)
{
	u8 i;
	vu16 word;
	if(data>60e3)
	{
		for(i=0;i<5;i++)
		DispBuf[i]='-';
	}
	else 
		if(data>6e3)
		{
			word=data/10;
			Hex_Format((word),2,4,TRUE);
		
		
		}
		else
			Hex_Format((data),3,4,TRUE);
			


}
void BCD_Int(int32_t data)
{
	u8 i;
	vu16 word;
	if(data>30e6)
	{
		Test_Unit.Res_dot=1;
		for(i=0;i<5;i++)
		DispBuf[i]='-';
	
	}else
		if(data>=10e6)//xx.xx
		{
			Test_Unit.Res_dot=1;
			word=data/10e3;
			Hex_Format((word),2,4,TRUE);
				
		}else
			if(data>=10e5)//x.xxx
			{
				Test_Unit.Res_dot=1;
				word=data/10e2;
				Hex_Format((word),3,4,TRUE);
			}
			else
				if(data>=10e4)//xxx.xm
				{
					Test_Unit.Res_dot=0;
					word=data/10e1;
					Hex_Format((word),1,4,TRUE);
				}else
				if(data>=10e3)
				{
					Test_Unit.Res_dot=0;
					word=data/10;
					Hex_Format((word),2,4,TRUE);
				}
				else
				{
					Test_Unit.Res_dot=0;
					word=data/10;
					Hex_Format((word),2,4,TRUE);
				
				
				}


}
float Debug_Res(float a,float b,float c)
{
	//float d;
	return a*b/c;
}
void  Write_Usbdata ( uint8_t  *buffer,uint32_t num)
{
    int32_t  fdw;
//    int32_t  fdr;
//    uint32_t  tot_bytes_written;
    uint32_t  bytes_written;
    char filename[30];
//    strcpy(filename,(char*)"JK2520B");
 //       filename=(char*)"JK2520B";
        strcpy(filename,(char*)Save_Res.Sys_Setvalue.textname); 
        strcat(filename,(char*)".TXT");
 //       
        fdw = FILE_Open((uint8_t *)filename, RDWR);
        if (fdw > 0) 
		{
			usb_oenflag=1;

			bytes_written = FILE_Write(fdw, buffer, num);//MAX_BUFFER_SIZE);

			FILE_Close(fdw);
            
        } 
		else
			usb_oenflag=0;

    } 
void input_password(Disp_Coordinates_Typedef *Coordinates )
{
  	char While_flag=1;
	char Disp_buff[10];
	u8 key,i;
	char dispflag=1;
	char dot_num=0,dot_num1=0;
    char password_flag=0;
//	vu8 page=0;
	uint32_t keynum=0;
	char  key_count=0;
	for(i=0;i<9;i++)
	Disp_buff[i]=0;
	//Disp_buff[8]=0;
	Disp_Button_value1(2);
	while(While_flag)
	{
		key=HW_KeyScsn();
		if(key==0xff)
		{
			keynum=0;
		}
		else
			keynum++;
		if(keynum==KEY_NUM)
		{
            //Key_Beep()
            password_flag=0;
            Key_Beep();
			dispflag=1;
			switch(key)
			{
				case Key_F1://退出
					 While_flag=0;
                       SetSystemStatus(SYS_STATUS_TEST);

					
				break;
				case Key_F3://取消
                    
					
				break;
				case Key_F2://确认
                   if(strcmp(PASSWORD,Disp_buff))//比较函数  当相等时  返回0
                   {//密码错误
                       key_count=0;
                       for(i=0;i<8;i++)
                        Disp_buff[i]=' ';
                       password_flag=1;
                       
                   
                   
                   }
                   else //密码正确
                   {
                        While_flag=0;
                       SetSystemStatus(SYS_STATUS_USERDEBUG);
                   
                   }
                       
					
				break;
				case Key_F4:
					
				break;
				case Key_F5:
					
				break;
				case Key_Disp:
//					SetSystemStatus(SYS_STATUS_TEST);
//					While_flag=0;
//					Sort_set.Updata_flag=0;
				break;
				case Key_SETUP:
//					While_flag=0;
//					Sort_set.Updata_flag=0;
//				SetSystemStatus(SYS_STATUS_SETUPTEST);
				break;
				case Key_FAST:
				break;
				case Key_LEFT:
				break;
				case Key_RIGHT:
				break;
				case Key_UP:
				break;
				case Key_DOWN:
				break;
				case Key_NUM1:
					if(key_count<PASSWORD_LENTH)
					{
						Disp_buff[key_count]='1';
						
						
							
						
						key_count++;
							
					}
				break;
				case Key_NUM2:
					if(key_count<PASSWORD_LENTH)
					{
						Disp_buff[key_count]='2';
						key_count++;
					}
				break;
				case Key_NUM3:
					if(key_count<PASSWORD_LENTH)
					{
						Disp_buff[key_count]='3';
						key_count++;
					}
				break;
				case Key_NUM4:
					if(key_count<PASSWORD_LENTH)
					{
						Disp_buff[key_count]='4';
						key_count++;
					}
					
				break;
				case Key_NUM5:
					if(key_count<PASSWORD_LENTH)
					{
						Disp_buff[key_count]='5';
						key_count++;
					}
				break;
				case Key_NUM6:
					if(key_count<PASSWORD_LENTH)
					{
						Disp_buff[key_count]='6';
						key_count++;
					}
				break;
				case Key_NUM7:
					if(key_count<PASSWORD_LENTH)
					{
						Disp_buff[key_count]='7';
						key_count++;
					}
				break;
				case Key_NUM8:
					if(key_count<PASSWORD_LENTH)
					{
						Disp_buff[key_count]='8';
						key_count++;
					}
				break;
				case Key_NUM9:
					if(key_count<PASSWORD_LENTH)
					{
						Disp_buff[key_count]='9';
						key_count++;
					}
				break;
				case Key_NUM0:
					if(key_count<PASSWORD_LENTH)
					{
						Disp_buff[key_count]='0';
						key_count++;
					}
				break;
				case Key_DOT:
					
					if(key_count<PASSWORD_LENTH&&key_count>0)
					{
						if(dot_num==0)
						{
							if(key_count>0)
							{
								Disp_buff[key_count]='.';
								dot_num1=key_count;
								key_count++;
							
							
							}
							dot_num++;
						}
					}
//					else 
//					{
//						if(Disp_buff[key_count]==0)
//							Disp_buff[key_count]='-';
//						else if(Disp_buff[key_count]=='-')
//							Disp_buff[key_count]='+';
//						else
//							Disp_buff[key_count]='-';
//						key_count++;
//							
//					
//					
//					
//					
//					}
				break;
				case Key_BACK:
					if(key_count>0)
					{	key_count--;
						Disp_buff[key_count]=' ';
						if(dot_num1==key_count)
						{
							dot_num=0;
							dot_num1=0;
							
						}
					
					}
					else
					{
						if(Disp_buff[key_count]==0)
								Disp_buff[key_count]='-';
							else if(Disp_buff[key_count]=='-')
								Disp_buff[key_count]='+';
							else
								Disp_buff[key_count]='-';
							key_count++;
					}
				break;
				case Key_LOCK:
				break;
				case Key_BIAS:
				break;
				case Key_REST:
				break;
				case Key_TRIG:
				break;
				default:
				break;
					
			}
		
		
		}
		if(dispflag)
		{
			dispflag=0;
            LCD_DrawLine( Coordinates->xpos+13, Coordinates->ypos-2, Coordinates->xpos+13+80, Coordinates->ypos-2 , Red );
            LCD_DrawLine( Coordinates->xpos+13, Coordinates->ypos+18, Coordinates->xpos+13+80, Coordinates->ypos+18 , Red );
            for(i=0;i<9;i++)
            LCD_DrawLine( Coordinates->xpos+13+i*10, Coordinates->ypos-2, Coordinates->xpos+13+i*10, Coordinates->ypos+18 , Red );
            
			//LCD_DrawRect( Coordinates->xpos, Coordinates->ypos,Coordinates->xpos+Coordinates->lenth , Coordinates->ypos+16 , Red );
			Colour.Fword=White;
			Colour.black=LCD_COLOR_TEST_BACK;
				if(Save_Res.Sys_Setvalue.lanage )
						WriteString_16(Coordinates->xpos-38, Coordinates->ypos, "PassWord:",  0);
				else
            WriteString_16(Coordinates->xpos-38, Coordinates->ypos, "密码:",  0);
				WriteString_16(Coordinates->xpos+15, Coordinates->ypos, ( uint8_t *)Disp_buff,  0);
            if(password_flag)
								if(Save_Res.Sys_Setvalue.lanage )
									WriteString_16(Coordinates->xpos, Coordinates->ypos+20, "PassWord Error!",  0);
								else
									WriteString_16(Coordinates->xpos, Coordinates->ypos+20, "密码错误",  0);
            else
               LCD_DrawRect( Coordinates->xpos, Coordinates->ypos+20,Coordinates->xpos+100 , Coordinates->ypos+40, LCD_COLOR_TEST_BACK );
               // WriteString_16(Coordinates->xpos, Coordinates->ypos, "         ",  0);
							if(Save_Res.Sys_Setvalue.lanage )
								WriteString_16(Coordinates->xpos, Coordinates->ypos-20, "Please input 8-bits PassWord",  0);
							else
								WriteString_16(Coordinates->xpos, Coordinates->ypos-20, "请输入8位密码",  0);
			//dispflag=0;
		}
	
	}

}
void input_num(Disp_Coordinates_Typedef *Coordinates )
{
  	char While_flag=1;
	uint8_t Disp_buff[12];
	uint8_t  key,i;
	char dispflag=1;
//	char dot_num=0,dot_num1=0;
//    char password_flag=0;
	char keynum=0;
	char key_count=0;
//	Sort_TypeDef   Sort_set;
	
	for(i=0;i<8;i++)
	Disp_buff[i]=' ';
	Disp_buff[8]=0;
	
	while(While_flag)
	{
		key=HW_KeyScsn();
		if(key==0xff)
		{
			keynum=0;
		}
		else
			keynum++;
		if(keynum==KEY_NUM)
		{
 //           password_flag=0;
            Key_Beep();
			dispflag=1;
			switch(key)
			{
				case Key_F1://退出
					 
                       //SetSystemStatus(SYS_STATUS_USERDEBUG);

					
				break;
				case Key_F2://取消
                    
					
				break;
				case Key_F3://确认保存机号和日期
//                   if(strcmp(PASSWORD,Disp_buff))//比较函数  当相等时  返回0
//                   {//密码错误
//                       key_count=0;
//                       for(i=0;i<8;i++)
//                        Disp_buff[i]=' ';
//                       password_flag=1;
//                       
//                   
//                   
//                   }
//                   else //密码正确
//                   {
//                        While_flag=0;
//                       SetSystemStatus(SYS_STATUS_USERDEBUG);
//                   
//                   }//复制到内存
                       
					
				break;
				case Key_F4:
					
				break;
				case Key_F5:
                   //return  &Disp_buff[0];
					
				break;
				case Key_Disp:
//					SetSystemStatus(SYS_STATUS_TEST);
//					While_flag=0;
//					Sort_set.Updata_flag=0;
				break;
				case Key_SETUP:
//					While_flag=0;
//					Sort_set.Updata_flag=0;
//				SetSystemStatus(SYS_STATUS_SETUPTEST);
				break;
				case Key_FAST:
                    While_flag=0;//保存
                    dispflag=0;
                    for(i=0;i<8;i++)
                    {
                        Save_Res.fac_num[i]=Disp_buff[i];
                    
                    }
                    Savetoeeprom();
                     LCD_DrawRect( 0, Coordinates->ypos-20,Coordinates->xpos+200 , Coordinates->ypos+20, LCD_COLOR_TEST_BACK );
				break;
				case Key_LEFT:
				break;
				case Key_RIGHT:
				break;
				case Key_UP:
				break;
				case Key_DOWN:
				break;
				case Key_NUM1:
					if(key_count<PASSWORD_LENTH)
					{
						Disp_buff[key_count]='1';
						
						
							
						
						key_count++;
							
					}
				break;
				case Key_NUM2:
					if(key_count<PASSWORD_LENTH)
					{
						Disp_buff[key_count]='2';
						key_count++;
					}
				break;
				case Key_NUM3:
					if(key_count<PASSWORD_LENTH)
					{
						Disp_buff[key_count]='3';
						key_count++;
					}
				break;
				case Key_NUM4:
					if(key_count<PASSWORD_LENTH)
					{
						Disp_buff[key_count]='4';
						key_count++;
					}
					
				break;
				case Key_NUM5:
					if(key_count<PASSWORD_LENTH)
					{
						Disp_buff[key_count]='5';
						key_count++;
					}
				break;
				case Key_NUM6:
					if(key_count<PASSWORD_LENTH)
					{
						Disp_buff[key_count]='6';
						key_count++;
					}
				break;
				case Key_NUM7:
					if(key_count<PASSWORD_LENTH)
					{
						Disp_buff[key_count]='7';
						key_count++;
					}
				break;
				case Key_NUM8:
					if(key_count<PASSWORD_LENTH)
					{
						Disp_buff[key_count]='8';
						key_count++;
					}
				break;
				case Key_NUM9:
					if(key_count<PASSWORD_LENTH)
					{
						Disp_buff[key_count]='9';
						key_count++;
					}
				break;
				case Key_NUM0:
					if(key_count<PASSWORD_LENTH)
					{
						Disp_buff[key_count]='0';
						key_count++;
					}
				break;
				case Key_DOT:
                    if(key_count<PASSWORD_LENTH)
					{
						Disp_buff[key_count]='/';
						key_count++;
					}
					
//					if(key_count<NUM_LENTH&&key_count>0)
//					{
//						if(dot_num==0)
//						{
//							if(key_count>0)
//							{
//								Disp_buff[key_count]='.';
//								dot_num1=key_count;
//								key_count++;
//							
//							
//							}
//							dot_num++;
//						}
//					}
//					else 
//					{
//						if(Disp_buff[key_count]==0)
//							Disp_buff[key_count]='-';
//						else if(Disp_buff[key_count]=='-')
//							Disp_buff[key_count]='+';
//						else
//							Disp_buff[key_count]='-';
//						key_count++;
//							
//					
//					
//					
//					
//					}
				break;
				case Key_BACK:
					if(key_count)
					{	key_count--;
						Disp_buff[key_count]=' ';
//						if(dot_num1==key_count)
//						{
//							dot_num=0;
//							dot_num1=0;
//							
//						}
					
					}
//					else
//					{
//						if(Disp_buff[key_count]==0)
//								Disp_buff[key_count]='-';
//							else if(Disp_buff[key_count]=='-')
//								Disp_buff[key_count]='+';
//							else
//								Disp_buff[key_count]='-';
//							key_count++;
//					}
				break;
				case Key_LOCK:
				break;
				case Key_BIAS:
				break;
				case Key_REST:
				break;
				case Key_TRIG:
				break;
				default:
				break;
					
			}
		
		
		}
		if(dispflag)
		{
			dispflag=0;
            LCD_DrawLine( Coordinates->xpos+13, Coordinates->ypos-2, Coordinates->xpos+13+80, Coordinates->ypos-2 , Red );
            LCD_DrawLine( Coordinates->xpos+13, Coordinates->ypos+18, Coordinates->xpos+13+80, Coordinates->ypos+18 , Red );
            for(i=0;i<9;i++)
            LCD_DrawLine( Coordinates->xpos+13+i*10, Coordinates->ypos-2, Coordinates->xpos+13+i*10, Coordinates->ypos+18 , Red );
            
			//LCD_DrawRect( Coordinates->xpos, Coordinates->ypos,Coordinates->xpos+Coordinates->lenth , Coordinates->ypos+16 , Red );
			Colour.Fword=White;
			Colour.black=LCD_COLOR_TEST_BACK;
					if(Save_Res.Sys_Setvalue.lanage )
						 WriteString_16(Coordinates->xpos-70, Coordinates->ypos, "Factory Num:",  0);
					else
            WriteString_16(Coordinates->xpos-70, Coordinates->ypos, "出厂编号:",  0);
					WriteString_16(Coordinates->xpos+15, Coordinates->ypos, ( uint8_t *)Disp_buff,  0);
//            if(password_flag)
//                WriteString_16(Coordinates->xpos, Coordinates->ypos+20, "密码错误",  0);
//            else
//               LCD_DrawRect( Coordinates->xpos, Coordinates->ypos+20,Coordinates->xpos+100 , Coordinates->ypos+40, LCD_COLOR_TEST_BACK );
               // WriteString_16(Coordinates->xpos, Coordinates->ypos, "         ",  0);
					if(Save_Res.Sys_Setvalue.lanage )
						WriteString_16(Coordinates->xpos, Coordinates->ypos-20, "Please input 8-bits Factory Num",  0);
					else
            WriteString_16(Coordinates->xpos, Coordinates->ypos-20, "请输入8位出厂编号",  0);
		}
	
	}

}

