/*---------------------------------------------------------------------
���� 4-20ma������⣬���������λuA   ����0-25ma

Ĭ�ϵ�ַ��	0x01
Ĭ�ϲ����ʣ�9600  (1:2400  2:4800  3:9600  4:19200 5:38400 6:115200)

1��	�������� 	0x01    Ӧ����������������������У�
��ַ 0x01 ���� 0x01 ��ֵ X(2�ֽ�) 
	Ӧ�� 			0x02
��ַ X    ACK 0x02 CRC 2�ֽ�			   (���磺05 02 05 03 )

2����ȡ��������	0x03  ��ַΪ0xff Ҳ����������Ϣ 
��ַ X ���� 0x03 ��ֵ 0 0(2�ֽ�) 
	Ӧ�� 			0x02
��ַ X ACK 0x04 ��ֵ  X X(2�ֽ�) 

3����ȡ�����ɼ���� 0x05
��ַ X ���� 0x05 ��ֵ 0(2�ֽ�) 
	Ӧ�� 			0x06
��ַ X ACK 0x06 ��ֵ X (2�ֽ�) 

4����ȡ��ѹ�ɼ���� 0x06
��ַ X ���� 0x07 ��ֵ 0(2�ֽ�)
	Ӧ�� 			0x06
��ַ X ACK 0x08 ��ֵ X (2�ֽ�) 
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

#define DRE_out     P3DIR |= BIT3     //����485оƬ��DE��RE�˿ڵ�IO����Ϊ���״̬
#define DE          P3OUT |= BIT3     //����485оƬ���ڷ���״̬ 
#define RE          P3OUT &= ~BIT3    //����485оƬ���ڽ���״̬
//typedef unsigned char uchar;
//typedef unsigned int  uint;

#define BUFLEN 20
uchar NBOK=0;
uint k;
uint average;//ADת����ƽ��ֵ
uchar adchl=2;//ADͨ��
unsigned long int average0,average1;//������ADֵ
uint result0[32];//�洢ͨ��0����
uint result1[32];//�洢ͨ��1����
uchar s1Buffer[BUFLEN]={0x01,0x03,0x02,0x85,0x03,0x9A,0xD5};//�������ݴ�������
uchar s1BufIdx = 0;
uchar Band=3;//������ģʽ
uchar addr=0x01;
uchar RecOK=0;//���ڽ��գ��������Ϊ1
unsigned char crcH = 0;//crc��8λ
unsigned char crcL = 0;//crc��8λ
uint count=0;//��ʱ������
#define RSTTIME 10//��ʱʱ��10*0.5s=5s

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
  /*�������г���ر����е�IO��*/
    P1DIR = 0XFF;P1OUT = 0XFF;
    P2DIR = 0XFF;P2OUT = 0Xff;
    //P3DIR = 0XFF;P3OUT = 0XFF;
    P4DIR = 0XFF;P4OUT = 0XFF;
    P5DIR=0xff;
    P6DIR = 0XFF;P6OUT = 0XFF;
    ExterOSC();
  TimerInit();
  //Flash_Init();//Flash��ʼ��
  
 //Flash���Դ��� 
 /* s1Buffer[2]=0x01;
  s1Buffer[3]=0x03;
  FlashWritten();*/
  
 /* if(FlashRead(0)!=0xff)//�ж�Flash�Ƿ��һ��ʹ��
  {
    addr=FlashRead(0);//��ȡ��ַ
    Band=FlashRead(1);//��ȡ������
  }*/
  
 /* for(k=0;k<200;k++)
  {     
      delay();
  }*/
  
  InitUART();//�����жϳ�ʼ��
 // DRE_out;//485�����������
  //RE;//485����״̬
  
  //WDTInit();
  __enable_interrupt();//����ȫ���ж�
  Init_ADC();//AD��ʼ��
  P5OUT=0xf0;
 

  
  while(1)
  {
    //SendBuffer();
    P2OUT=~BIT3;
    if(RecOK==1) //�����ֽ����ݺ��Ƚ���crc�����ж������Ƿ���ȷ��Ȼ�����ָ�����
    {  
          
          P2OUT^=BIT3;
          s1BufIdx = 0;
          RecOK=0;
          
          CRC16_INT(s1Buffer,4);//crc����
         //if(crcH == s1Buffer[4] && s1Buffer[5] == crcL)          
         // {
                 SendBuffer();//���ͽ��յ�������
                  //��ַ�ж�
                  if(s1Buffer[0]==addr)
                  {
                        //�����ж�
                        if(s1Buffer[1]==0x03)//��ȡ��0·AD����
                        {
                          adchl=0;
                          ADC12CTL0 |= ENC;                         // ʹ��ת��
                          ADC12CTL0 |= ADC12SC;                     // ��ʼת��
                        }
                        else if(s1Buffer[1]==CMD_DAT2)//��ȡ��1·AD����
                        {
                          adchl=1;
                          ADC12CTL0 |= ENC;                         // ʹ��ת��
                          ADC12CTL0 |= ADC12SC;                     // ��ʼת��
                        }
                        
                       /* //���õ�ַ����������
                        else if(s1Buffer[1]==CMD_SET)
                        {
                          Band=s1Buffer[3];
                          addr=s1Buffer[2];                         
                          FlashWritten();//�ѵ�ַ�Ͳ�������Ϣд��Flash
                          //����Ӧ������
                          s1Buffer[0]=addr;//��ַ
                          s1Buffer[1]=ACK_SET;//Ӧ��                      
                          s1Buffer[2]=addr;                        
                          s1Buffer[3]=Band;
                          CRC16_INT(s1Buffer,4);
                          s1Buffer[4]=crcH;//crc
                          s1Buffer[5]=crcL;//crc
                          SendBuffer();//����Ӧ������
                          InitUART();//���Ĵ��ڲ�����
                          
                        }
                        else if(s1Buffer[1]==CMD_GET)
                        {
                          //����Ӧ������
                          s1Buffer[0]=addr;//��ַ
                          s1Buffer[1]=ACK_GET;//Ӧ��                      
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

/*AD��ʼ��*/
void Init_ADC(void)
{
    P6SEL |= BIT0+BIT1;                            // ʹ��ADCͨ��
    ADC12CTL0 &=~ ENC;                         // ʧ��ת��
    ADC12CTL0 = ADC12ON+SHT0_15+MSC;          // ��ADC�����ò���ʱ��1024clk����ͨ����������
    ADC12CTL1 = SHP+CONSEQ_1;                 // ʹ�ò�����ʱ����ʱ����Ϊ������������ʱ��Դ����ͨ������ת��
    ADC12IE = BIT0+BIT1;                       // ʹ��ADC�ж�ͨ��0+ͨ��1
    ADC12MCTL0|=0x00;                   //ͨ��0
    ADC12MCTL1|=0x81;                   //ͨ��1+ת������
    ADC12CTL0 |= ENC;                         // ʹ��ת��
    ADC12CTL0 |= ADC12SC;                     // ��ʼת��
   
}

/*ADת���ж�*/
#pragma vector=ADC_VECTOR
__interrupt void ADC12ISR (void)
{
    static uchar index = 0;
    uchar SendData;
              
    
    
    if(index<32)
    {
         // SendChar(0xff);//���ͽ��յ�������
          //while(ADC12BUSY&ADC12CTL1);//�ȴ�ת�����

          average0 += ADC12MEM0;//���
          average1 += ADC12MEM1;
          index++;
          if(index<32)//��ǰһ�β�ʹ��ת��ģ�飬�����һֱ��ADC�ж���
          {
            ADC12CTL0 |= ENC;                         // ʹ��ת��
            ADC12CTL0 |= ADC12SC;                     // ��ʼ��һ��ת��
          }
          
    } 
    
    if(index == 32)//����32��ת��
    {
           index = 0;
          //ADC12CTL0 &= ~ENC;                         // ʧ��ת�� 
           //ADC12CTL0 &= ~ADC12SC;
           average0 >>= 5;                            //����32�ó�ƽ��ֵ
           average1 >>= 5;                            //����32
           // P2OUT^=BIT4;    
           
          //ADC12IFG=0x00;
          if(adchl==0)
          {  
        
              
              //���ɷ�����������
                  s1Buffer[0]=addr;//��ַ
                  s1Buffer[1]=0x03;//Ӧ��
                  s1Buffer[2]=0x02;
                  SendData=average0>>8;
                  s1Buffer[3]=SendData;//ת����ֵ��8λ
                  SendData=(average0&0xff);
                  s1Buffer[4]=SendData;//ת����ֵ��4λ
                  CRC16_INT(s1Buffer,5);
                  s1Buffer[5]=crcH;//crc
                  s1Buffer[6]=crcL;//crc
                  
                 //ADC12IE = 0x00;                       // ʧ��ADC�ж�
                   average0 = 0;                           //����
                   average1 = 0;                                         
                 SendBuffer();    
             
          }
          else if(adchl==1)
          {
            //���ɷ�����������
                  s1Buffer[0]=addr;//��ַ
                  s1Buffer[1]=ACK_DAT2;//Ӧ��
                  SendData=average1>>4;
                  s1Buffer[2]=SendData;//ת����ֵ��8λ
                  SendData=(average1&0x0f);
                  s1Buffer[3]=SendData;//ת����ֵ��4λ
                  CRC16_INT(s1Buffer,4);
                  s1Buffer[4]=0x01;//crcH;//crc
                  s1Buffer[5]=0x02;//crcL;//crc
                  
                 //ADC12IE = 0x00;                       // ʧ��ADC�ж�
                  
                 average0 = 0;                           //����
                 average1 = 0;                                   
                 SendBuffer(); 
          }
                
       
    }
    
    
   
    
}

/*���ڳ�ʼ��*/
void InitUART()
{
    P3SEL |= 0x30;                            // P3.4,5 ����ʹ�ô��ڸ�λ����
    UCTL0 |= SWRST;                          // �رո�λ
    ME1 |= URXE0 + UTXE0;                     // �������ͽ���ģ��
    UCTL0 |= CHAR;                            // 8λ���ݴ��䣬����żУ�飬1ֹͣλ����ʹ�ûػ�ģʽ
    
    
    if(Band==3)//9600
    {
        UTCTL0 = SSEL0;                          // ѡ��ACLKΪʱ��
       UBR00 = 0x03;                             // 32768hz/9600 - 3.41��ȡʱ��/�����ʵ��������ָ�ֵ��UBR00
       UBR10 = 0x00;                            //UBR10ΪUBR�Ĵ����ĸ�8λ
       UMCTL0 = 0x4A;                            // 0.41*8=3.28,01001010,�����ֿ��ֲ�
    }
    else if(Band==1)//2400
    {
        UTCTL0 = SSEL0;                          // ѡ��ACLKΪʱ��
        UBR00 = 0x0D;                             // 32768hz/2400 - 13.65��ȡʱ��/�����ʵ��������ָ�ֵ��UBR00
        UBR10 = 0x00;                            //UBR10ΪUBR�Ĵ����ĸ�8λ
        UMCTL0 = 0xAB;                            // 0.41*8=5.2,10101011,�����ֿ��ֲ�
    }
    else if(Band==2)//4800
    {
        UTCTL0 = SSEL0;                          // ѡ��ACLKΪʱ��
       UBR00 = 0x06;                             // 32768hz/4800- 6.82��ȡʱ��/�����ʵ��������ָ�ֵ��UBR00
       UBR10 = 0x00;                            //UBR10ΪUBR�Ĵ����ĸ�8λ
       UMCTL0 = 0xB7;                            // 0.41*8=6.56,10110111,�����ֿ��ֲ�
    }
    else if(Band==4)//19200��Ҫ����ʱ��ԴSMCLK
    {
       UTCTL0 = SSEL1+SSEL0; 
       /*UBR00 = 0x36;                             
       UBR10 = 0x00;                            
       UMCTL0 = 0x6B;*/    
       UBR00 = 0x34;                             
       UBR10 = 0x00;                            
       UMCTL0 = 0x00;  
    }
    else if(Band==5)//38400��Ҫ����ʱ��ԴSMCLK
    {
       UTCTL0 = SSEL1+SSEL0;                          
       UBR00 = 0x1A;                             
       UBR10 = 0x00;                            
       UMCTL0 = 0x00;                            
    }
    else if(Band==6)//115200��Ҫ����ʱ��ԴSMCLK
    {
       UTCTL0 = SSEL1+SSEL0;                          
     
      UBR00 = 0x08;                             
       UBR10 = 0x00;                            
       UMCTL0 = 0xAE;
    }
    else//9600
    {
       UTCTL0 = SSEL0;                          // ѡ��ACLKΪʱ��
       UBR00 = 0x03;                             // 32768hz/9600 - 3.41��ȡʱ��/�����ʵ��������ָ�ֵ��UBR00
       UBR10 = 0x00;                            //UBR10ΪUBR�Ĵ����ĸ�8λ
       UMCTL0 = 0x4A;                            // 0.41*8=3.28,01001010,�����ֿ��ֲ�
    }
                     
    UCTL0 &= ~SWRST;                          // �رո�λ
    IE1|=URXIE0;                              //�����ж�ʹ�ܣ������ж����ñ�����SWEST��0ʱ������Ч
}

/*���ڷ���1���ַ�*/
void SendChar(uchar sendchar)
{
      
      while (!(IFG1 & UTXIFG0));    //�ȴ����ͼĴ���Ϊ�գ������ͼĴ����������ʱ��IFG=1��while����1��������ѭ��         
      TXBUF0 = sendchar; 
      //while(!(TXEPT&UTCTL0));
      
}

/*����s1Buffer*/
void SendBuffer()
{
   DE;//485����״̬
   //P2OUT^=BIT5;
   //delay();//����ʱ��

  for(s1BufIdx = 0;s1BufIdx<7;s1BufIdx++)
  {
    SendChar(s1Buffer[s1BufIdx]);
  }
         
  s1BufIdx = 0;
 // delay();//�ȴ�����������λ
   RE; //485����״̬  
 
}

void TimerInit()
{
    TACTL|=TASSEL0+ID0+ID1+TACLR;//ACLK��8��Ƶ����ն�ʱ��
    TACCTL0=OUTMOD0+CCIE+OUT;//��λģʽ�����ж�,�˿ڳ�ʼ��Ϊ�ߵ�ƽ
    CCR0=400;//0.1s   
    //TACTL|=MC0;//ʹ�ܶ�ʱ��
}

#pragma vector=TIMERA0_VECTOR
__interrupt void Timer()
{
  
  //count++;
 // WDTCTL=WDT_ARST_1000+WDTCNTCL;//ι��
 /*if(count==RSTTIME)//20*0.5=10s
  {
    P2OUT=~BIT3;//��λ
    delay();
    P2OUT=BIT3;//��λ
    count=0;
  }*/
  RecOK=1;
  TACTL&=~(MC0+MC1);//ֹͣ��ʱ��
  CCR0=200;//0.1s  200 0.05s
}

/*���ڽ����ж�*/
#pragma vector=USART0RX_VECTOR
__interrupt void uart()
{
  if(s1BufIdx==0)
  {
    TACTL|=MC0;//ʹ�ܶ�ʱ��
  }
  CCR0=200;//0.1s  200 0.05s
  s1Buffer[s1BufIdx] = RXBUF0;
  s1BufIdx++;
  NBOK=1; 
    
    //P2OUT|=BIT6;
     
}

void Flash_Init()
{
   FCTL2 = FWKEY + FSSEL0 + FN0;//д�Ĵ�����ʱ��ѡ��SMCLK/2
}

/*��FLASH���ݣ�index��ʾ���ڼ����ֽ�����*/
char FlashRead(char index)
{
    char *Flash_ptr;//Flash��ַָ��
    Flash_ptr = ((char *) 0x1080)+index; //segA�洢�����׵�ַ
    return *Flash_ptr;
}

/*�ѵ�ַ�Ͳ�����д��0x1080��ʼ�ĵ�ַ���˶�128���ֽ�*/
void FlashWritten()
{
  char *Flash_ptr;//Flash��ַָ��
  Flash_ptr = (char *) 0x1080; //segA�洢�����׵�ַ
  FCTL1 = FWKEY + ERASE;//�������洢�ռ��
  FCTL3 = FWKEY; //������д��������
  *Flash_ptr = 0;//��0x1080��ʼ������
  
  FCTL1 = FWKEY + WRT;//дģʽ����
  
  //д�����ݣ���ַ�Ͳ��������ã�
  
  *Flash_ptr =s1Buffer[2];                   // д���ַ
   Flash_ptr++;
  *Flash_ptr =s1Buffer[3];                   // д�벨����
    
  FCTL1 = FWKEY;                            // �ر�дģʽ
  FCTL3 = FWKEY + LOCK;                     // ����Flash
  
}

/////////////��׼CRC16У���ѯ�� ����Ҫ������ô�����ģ�������ô��//// 
////////////////////////////// CRC ��λ�ֽ�ֵ�� *///////////////////////////////////////////
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
////////////////////////////////// CRC ��λ�ֽ�ֵ�� ///////////////////////////////////// 
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
///CRC16У�麯����ʹ�ú����������У������ָ�루�������ֿ����������ֽ���������ֵΪushort��16λCRCУ����///  
void CRC16_INT(unsigned char *puchMsg , unsigned short usDataLen)    
{ 
	unsigned char uchCRCHi = 0xFF;                          /* ��CRC�ֽڳ�ʼ�� */         
	unsigned char uchCRCLo = 0xFF;                         /* ��CRC�ֽڳ�ʼ�� */         
	unsigned int uIndex ;                              /* CRCѭ���е����� */         
	while (usDataLen--)                           /* ������Ϣ������ */       
	{ 
		uIndex = uchCRCHi ^ (*puchMsg++);        /*          */                 
		uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex]; /* ����CRC */                 
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
    BCSCTL1&=~XT2OFF;//�����ⲿʱ��  XT2 
    do
    {
      IFG1&=~OFIFG;//�������ʧЧ��־
      for(i=0xff;i>0;i--);
    }
    while(IFG1&OFIFG);
    BCSCTL2|=SELS+DIVS0+DIVS1;//ѡ��XT2ΪSMCLk��SMCLK8��Ƶ
    //BCSCTL2|=SELM1+DIVM0+DIVS1;//ѡ��XT2ΪMclk��8��Ƶ
    
    
}