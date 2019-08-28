/*---------------------------------------------------------------------
功能 4-20ma电流检测，输出电流单位uA   量程0-25ma

默认地址：	0x01
默认波特率：9600  (1:2400  2:4800  3:9600  4:19200 5:38400 6:115200)

1、	设置命令 	0x01    应答后重启（按照新配置运行）
地址 0x01 命令 0x01 数值 X(2字节) 
	应答 			0x02
地址 X    ACK 0x02 CRC 2字节			   (例如：05 02 05 03 )

2、读取配置命令	0x03  地址为0xff 也返回配置信息 
地址 X 命令 0x03 数值 0 0(2字节) 
	应答 			0x02
地址 X ACK 0x04 数值  X X(2字节) 

3、获取电流采集结果 0x05
地址 X 命令 0x05 数值 0(2字节) 
	应答 			0x06
地址 X ACK 0x06 数值 X (2字节) 

4、获取电压采集结果 0x06
地址 X 命令 0x07 数值 0(2字节)
	应答 			0x06
地址 X ACK 0x08 数值 X (2字节) 
---------------------------------------------------------------------*/
#include "io430.h"
#include "string.h"

#define CMD_SET 0x01
#define ACK_SET 0x02
#define CMD_GET 0x03
#define ACK_GET 0x04
#define CMD_DAT1 0x05
#define ACK_DAT1 0x06
#define CMD_DAT2 0x07
#define ACK_DAT2 0x08

#define DRE_out     P3DIR |= BIT3     //连接485芯片的DE，RE端口的IO设置为输出状态
#define DE          P3OUT |= BIT3     //设置485芯片处于发送状态 
#define RE          P3OUT &= ~BIT3    //设置485芯片处于接收状态
//typedef unsigned char uchar;
//typedef unsigned int  uint;

#define BUFLEN 20
uchar NBOK=0;
uint k;
uint average;//AD转换的平均值
uchar adchl=2;//AD通道
unsigned long int average0,average1;//计算后的AD值
uint result0[32];//存储通道0数据
uint result1[32];//存储通道1数据
uchar s1Buffer[BUFLEN]={0x01,0x03,0x02,0x85,0x03,0x9A,0xD5};//定义数据传输数组
uchar s1BufIdx = 0;
uchar Band=3;//波特率模式
uchar addr=0x01;
uchar RecOK=0;//串口接收，接收完毕为1
unsigned char crcH = 0;//crc高8位
unsigned char crcL = 0;//crc低8位
uint count=0;//定时器计数
#define RSTTIME 10//定时时间10*0.5s=5s

void SendChar(unsigned char sendchar);
void InitUART();
void SendBuffer();
void Init_ADC(void);
void CRC16_INT(unsigned char *puchMsg , unsigned short usDataLen) ;
void Flash_Init();
void FlashWritten();
char FlashRead(char index);
void delay();
void WDTInit();
void ExterOSC();
void TimerInit();

int main( void )
{
  
  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;
  /*下面六行程序关闭所有的IO口*/
    P1DIR = 0XFF;P1OUT = 0XFF;
    P2DIR = 0XFF;P2OUT = 0Xff;
    //P3DIR = 0XFF;P3OUT = 0XFF;
    P4DIR = 0XFF;P4OUT = 0XFF;
    P5DIR=0xff;
    P6DIR = 0XFF;P6OUT = 0XFF;
    ExterOSC();
  TimerInit();
  //Flash_Init();//Flash初始化
  
 //Flash测试代码 
 /* s1Buffer[2]=0x01;
  s1Buffer[3]=0x03;
  FlashWritten();*/
  
 /* if(FlashRead(0)!=0xff)//判断Flash是否第一次使用
  {
    addr=FlashRead(0);//读取地址
    Band=FlashRead(1);//读取波特率
  }*/
  
 /* for(k=0;k<200;k++)
  {     
      delay();
  }*/
  
  InitUART();//串口中断初始化
 // DRE_out;//485引脚输出方向
  //RE;//485接收状态
  
  //WDTInit();
  __enable_interrupt();//开启全局中断
  Init_ADC();//AD初始化
  P5OUT=0xf0;
 

  
  while(1)
  {
    //SendBuffer();
    P2OUT=~BIT3;
    if(RecOK==1) //接收字节数据后，先进行crc计算判定数据是否正确，然后进行指令解析
    {  
          
          P2OUT^=BIT3;
          s1BufIdx = 0;
          RecOK=0;
          
          CRC16_INT(s1Buffer,4);//crc计算
         //if(crcH == s1Buffer[4] && s1Buffer[5] == crcL)          
         // {
                 SendBuffer();//发送接收到的数据
                  //地址判断
                  if(s1Buffer[0]==addr)
                  {
                        //命令判断
                        if(s1Buffer[1]==0x03)//读取第0路AD命令
                        {
                          adchl=0;
                          ADC12CTL0 |= ENC;                         // 使能转换
                          ADC12CTL0 |= ADC12SC;                     // 开始转换
                        }
                        else if(s1Buffer[1]==CMD_DAT2)//读取第1路AD命令
                        {
                          adchl=1;
                          ADC12CTL0 |= ENC;                         // 使能转换
                          ADC12CTL0 |= ADC12SC;                     // 开始转换
                        }
                        
                       /* //设置地址波特率命令
                        else if(s1Buffer[1]==CMD_SET)
                        {
                          Band=s1Buffer[3];
                          addr=s1Buffer[2];                         
                          FlashWritten();//把地址和波特率信息写入Flash
                          //生成应答数据
                          s1Buffer[0]=addr;//地址
                          s1Buffer[1]=ACK_SET;//应答                      
                          s1Buffer[2]=addr;                        
                          s1Buffer[3]=Band;
                          CRC16_INT(s1Buffer,4);
                          s1Buffer[4]=crcH;//crc
                          s1Buffer[5]=crcL;//crc
                          SendBuffer();//发送应答数据
                          InitUART();//更改串口波特率
                          
                        }
                        else if(s1Buffer[1]==CMD_GET)
                        {
                          //生成应答数据
                          s1Buffer[0]=addr;//地址
                          s1Buffer[1]=ACK_GET;//应答                      
                          s1Buffer[2]=addr;                        
                          s1Buffer[3]=Band;
                          CRC16_INT(s1Buffer,4);
                          s1Buffer[4]=crcH;//crc
                          s1Buffer[5]=crcL;//crc
                          SendBuffer();
                        }*/
                  }
    }
  }
  
}

void delay()
{
  uint i;
  for(i=0;i<10000;i++);
}

/*AD初始化*/
void Init_ADC(void)
{
    P6SEL |= BIT0+BIT1;                            // 使能ADC通道
    ADC12CTL0 &=~ ENC;                         // 失能转换
    ADC12CTL0 = ADC12ON+SHT0_15+MSC;          // 打开ADC，设置采样时间1024clk，多通道采样开启
    ADC12CTL1 = SHP+CONSEQ_1;                 // 使用采样定时器的时钟作为采样保持器的时钟源，多通道单次转化
    ADC12IE = BIT0+BIT1;                       // 使能ADC中断通道0+通道1
    ADC12MCTL0|=0x00;                   //通道0
    ADC12MCTL1|=0x81;                   //通道1+转换结束
    ADC12CTL0 |= ENC;                         // 使能转换
    ADC12CTL0 |= ADC12SC;                     // 开始转换
   
}

/*AD转换中断*/
#pragma vector=ADC_VECTOR
__interrupt void ADC12ISR (void)
{
    static uchar index = 0;
    uchar SendData;
              
    
    
    if(index<32)
    {
         // SendChar(0xff);//发送接收到的数据
          //while(ADC12BUSY&ADC12CTL1);//等待转换完毕

          average0 += ADC12MEM0;//求和
          average1 += ADC12MEM1;
          index++;
          if(index<32)//提前一次不使能转换模块，否则会一直在ADC中断里
          {
            ADC12CTL0 |= ENC;                         // 使能转换
            ADC12CTL0 |= ADC12SC;                     // 开始新一次转换
          }
          
    } 
    
    if(index == 32)//控制32次转换
    {
           index = 0;
          //ADC12CTL0 &= ~ENC;                         // 失能转换 
           //ADC12CTL0 &= ~ADC12SC;
           average0 >>= 5;                            //除以32得出平均值
           average1 >>= 5;                            //除以32
           // P2OUT^=BIT4;    
           
          //ADC12IFG=0x00;
          if(adchl==0)
          {  
        
              
              //生成发送数据数组
                  s1Buffer[0]=addr;//地址
                  s1Buffer[1]=0x03;//应答
                  s1Buffer[2]=0x02;
                  SendData=average0>>8;
                  s1Buffer[3]=SendData;//转换数值高8位
                  SendData=(average0&0xff);
                  s1Buffer[4]=SendData;//转换数值低4位
                  CRC16_INT(s1Buffer,5);
                  s1Buffer[5]=crcH;//crc
                  s1Buffer[6]=crcL;//crc
                  
                 //ADC12IE = 0x00;                       // 失能ADC中断
                   average0 = 0;                           //清零
                   average1 = 0;                                         
                 SendBuffer();    
             
          }
          else if(adchl==1)
          {
            //生成发送数据数组
                  s1Buffer[0]=addr;//地址
                  s1Buffer[1]=ACK_DAT2;//应答
                  SendData=average1>>4;
                  s1Buffer[2]=SendData;//转换数值高8位
                  SendData=(average1&0x0f);
                  s1Buffer[3]=SendData;//转换数值低4位
                  CRC16_INT(s1Buffer,4);
                  s1Buffer[4]=0x01;//crcH;//crc
                  s1Buffer[5]=0x02;//crcL;//crc
                  
                 //ADC12IE = 0x00;                       // 失能ADC中断
                  
                 average0 = 0;                           //清零
                 average1 = 0;                                   
                 SendBuffer(); 
          }
                
       
    }
    
    
   
    
}

/*串口初始化*/
void InitUART()
{
    P3SEL |= 0x30;                            // P3.4,5 引脚使用串口复位功能
    UCTL0 |= SWRST;                          // 关闭复位
    ME1 |= URXE0 + UTXE0;                     // 开启发送接收模块
    UCTL0 |= CHAR;                            // 8位数据传输，无奇偶校验，1停止位，不使用回环模式
    
    
    if(Band==3)//9600
    {
        UTCTL0 = SSEL0;                          // 选择ACLK为时钟
       UBR00 = 0x03;                             // 32768hz/9600 - 3.41，取时钟/波特率的整数部分赋值给UBR00
       UBR10 = 0x00;                            //UBR10为UBR寄存器的高8位
       UMCTL0 = 0x4A;                            // 0.41*8=3.28,01001010,尽量分开分布
    }
    else if(Band==1)//2400
    {
        UTCTL0 = SSEL0;                          // 选择ACLK为时钟
        UBR00 = 0x0D;                             // 32768hz/2400 - 13.65，取时钟/波特率的整数部分赋值给UBR00
        UBR10 = 0x00;                            //UBR10为UBR寄存器的高8位
        UMCTL0 = 0xAB;                            // 0.41*8=5.2,10101011,尽量分开分布
    }
    else if(Band==2)//4800
    {
        UTCTL0 = SSEL0;                          // 选择ACLK为时钟
       UBR00 = 0x06;                             // 32768hz/4800- 6.82，取时钟/波特率的整数部分赋值给UBR00
       UBR10 = 0x00;                            //UBR10为UBR寄存器的高8位
       UMCTL0 = 0xB7;                            // 0.41*8=6.56,10110111,尽量分开分布
    }
    else if(Band==4)//19200需要更改时钟源SMCLK
    {
       UTCTL0 = SSEL1+SSEL0; 
       /*UBR00 = 0x36;                             
       UBR10 = 0x00;                            
       UMCTL0 = 0x6B;*/    
       UBR00 = 0x34;                             
       UBR10 = 0x00;                            
       UMCTL0 = 0x00;  
    }
    else if(Band==5)//38400需要更改时钟源SMCLK
    {
       UTCTL0 = SSEL1+SSEL0;                          
       UBR00 = 0x1A;                             
       UBR10 = 0x00;                            
       UMCTL0 = 0x00;                            
    }
    else if(Band==6)//115200需要更改时钟源SMCLK
    {
       UTCTL0 = SSEL1+SSEL0;                          
     
      UBR00 = 0x08;                             
       UBR10 = 0x00;                            
       UMCTL0 = 0xAE;
    }
    else//9600
    {
       UTCTL0 = SSEL0;                          // 选择ACLK为时钟
       UBR00 = 0x03;                             // 32768hz/9600 - 3.41，取时钟/波特率的整数部分赋值给UBR00
       UBR10 = 0x00;                            //UBR10为UBR寄存器的高8位
       UMCTL0 = 0x4A;                            // 0.41*8=3.28,01001010,尽量分开分布
    }
                     
    UCTL0 &= ~SWRST;                          // 关闭复位
    IE1|=URXIE0;                              //串口中断使能，串口中断配置必须在SWEST置0时才能生效
}

/*串口发送1个字符*/
void SendChar(uchar sendchar)
{
      
      while (!(IFG1 & UTXIFG0));    //等待发送寄存器为空，当发送寄存器发送完成时，IFG=1，while（！1），跳出循环         
      TXBUF0 = sendchar; 
      //while(!(TXEPT&UTCTL0));
      
}

/*发送s1Buffer*/
void SendBuffer()
{
   DE;//485发送状态
   //P2OUT^=BIT5;
   //delay();//动作时间

  for(s1BufIdx = 0;s1BufIdx<7;s1BufIdx++)
  {
    SendChar(s1Buffer[s1BufIdx]);
  }
         
  s1BufIdx = 0;
 // delay();//等待发送完再置位
   RE; //485接收状态  
 
}

void TimerInit()
{
    TACTL|=TASSEL0+ID0+ID1+TACLR;//ACLK，8分频，清空定时器
    TACCTL0=OUTMOD0+CCIE+OUT;//复位模式，开中断,端口初始化为高电平
    CCR0=400;//0.1s   
    //TACTL|=MC0;//使能定时器
}

#pragma vector=TIMERA0_VECTOR
__interrupt void Timer()
{
  
  //count++;
 // WDTCTL=WDT_ARST_1000+WDTCNTCL;//喂狗
 /*if(count==RSTTIME)//20*0.5=10s
  {
    P2OUT=~BIT3;//复位
    delay();
    P2OUT=BIT3;//置位
    count=0;
  }*/
  RecOK=1;
  TACTL&=~(MC0+MC1);//停止定时器
  CCR0=200;//0.1s  200 0.05s
}

/*串口接收中断*/
#pragma vector=USART0RX_VECTOR
__interrupt void uart()
{
  if(s1BufIdx==0)
  {
    TACTL|=MC0;//使能定时器
  }
  CCR0=200;//0.1s  200 0.05s
  s1Buffer[s1BufIdx] = RXBUF0;
  s1BufIdx++;
  NBOK=1; 
    
    //P2OUT|=BIT6;
     
}

void Flash_Init()
{
   FCTL2 = FWKEY + FSSEL0 + FN0;//写寄存器，时钟选择SMCLK/2
}

/*读FLASH内容，index表示读第几个字节数据*/
char FlashRead(char index)
{
    char *Flash_ptr;//Flash地址指针
    Flash_ptr = ((char *) 0x1080)+index; //segA存储器的首地址
    return *Flash_ptr;
}

/*把地址和波特率写入0x1080开始的地址，此段128个字节*/
void FlashWritten()
{
  char *Flash_ptr;//Flash地址指针
  Flash_ptr = (char *) 0x1080; //segA存储器的首地址
  FCTL1 = FWKEY + ERASE;//擦除主存储空间段
  FCTL3 = FWKEY; //解锁读写擦除功能
  *Flash_ptr = 0;//从0x1080开始擦除段
  
  FCTL1 = FWKEY + WRT;//写模式开启
  
  //写入数据（地址和波特率设置）
  
  *Flash_ptr =s1Buffer[2];                   // 写入地址
   Flash_ptr++;
  *Flash_ptr =s1Buffer[3];                   // 写入波特率
    
  FCTL1 = FWKEY;                            // 关闭写模式
  FCTL3 = FWKEY + LOCK;                     // 锁定Flash
  
}

/////////////标准CRC16校验查询表 ，不要问我怎么得来的，就是这么用//// 
////////////////////////////// CRC 高位字节值表 *///////////////////////////////////////////
unsigned char  const auchCRCHi[] = { 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,  
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,  
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,  
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,  
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,  
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,  
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,  
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,  
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,  
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,  
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,  
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40};  
////////////////////////////////// CRC 低位字节值表 ///////////////////////////////////// 
 unsigned char  const auchCRCLo[] = { 
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,  
    0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 
    0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,  
    0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,  
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 
    0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,  
    0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,  
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,  
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,  
    0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,  
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,  
    0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,  
    0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,  
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40};
 
////////////////////////////////////////////////////////////////////////////////////////////////////// 
///CRC16校验函数，使用函数需输入待校验数组指针（数组名字咯）与数组字节数，返回值为ushort型16位CRC校验码///  
void CRC16_INT(unsigned char *puchMsg , unsigned short usDataLen)    
{ 
	unsigned char uchCRCHi = 0xFF;                          /* 高CRC字节初始化 */         
	unsigned char uchCRCLo = 0xFF;                         /* 低CRC字节初始化 */         
	unsigned int uIndex ;                              /* CRC循环中的索引 */         
	while (usDataLen--)                           /* 传输消息缓冲区 */       
	{ 
		uIndex = uchCRCHi ^ (*puchMsg++);        /*          */                 
		uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex]; /* 计算CRC */                 
		uchCRCLo = auchCRCLo[uIndex];         /*      */      
	} 
	crcH = uchCRCHi;
	crcL = uchCRCLo;
	//return (uchCRCHi << 8 | uchCRCLo);     
}////////////////////////////////////////////////////////////////////////////////////////////////////// 

void WDTInit()
{
    WDTCTL=WDT_ARST_1000;
    
}

void ExterOSC()
{
    uchar i;
    BCSCTL1&=~XT2OFF;//开启外部时钟  XT2 
    do
    {
      IFG1&=~OFIFG;//清除振荡器失效标志
      for(i=0xff;i>0;i--);
    }
    while(IFG1&OFIFG);
    BCSCTL2|=SELS+DIVS0+DIVS1;//选择XT2为SMCLk，SMCLK8分频
    //BCSCTL2|=SELM1+DIVM0+DIVS1;//选择XT2为Mclk，8分频
    
    
}