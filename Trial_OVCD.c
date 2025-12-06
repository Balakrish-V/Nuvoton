#include "MS51_16K.h"
#include <math.h>
sbit statusled=P0^0;
sbit relay=P0^2;
sbit on=P0^1;
sbit insttrip=P2^0;
sbit restore=P1^7;
sbit rst=P1^6;
typedef struct {
    unsigned char Port;
    char Pin;
} SegmentData;
code SegmentData segmentData[7][3]={
{{0x39,0x02},{0x27,0x10},{0x1E,0x01}}, 
{{0x35,0x02},{0x2B,0x04},{0x1B,0x04}}, 
{{0x2D,0x10},{0x27,0x08},{0x0F,0x10}}, 
{{0x1D,0x02},{0x33,0x04},{0x17,0x20}}, 
{{0x2D,0x02},{0x1B,0x20},{0x17,0x08}}, 
{{0x39,0x04},{0x33,0x08},{0x0F,0x20}}, 
{{0x35,0x08},{0x2B,0x10},{0x2E,0x01}}};
#define    ENABLE1_ADC_AIN4         ADCCON0&=0xF0;ADCCON0|=0x04;AINDIDS=0x00;AINDIDS|=0x10;ADCCON1|=0x31;ADCCON2|=0X0E;//current mono
#define    ENABLE1_ADC_AIN1         ADCCON0&=0xF0;ADCCON0|=0x01;AINDIDS=0x00;AINDIDS|=0x02;ADCCON1|=0x31;ADCCON2|=0X0E;//voltage
#define    ENABLE1_ADC_AIN5         ADCCON0&=0xF0;ADCCON0|=0x05;AINDIDS=0x00;AINDIDS|=0x20;ADCCON1|=0x31;ADCCON2|=0X0E;//switch
#define segmentData2(x) (~(x))
#define Setflag(var,statex)    ((var) |= (1 << (statex)))  
#define clearflag(var,statex)  ((var) &= ~(1 << (statex)))
void port(void); 
void delay(unsigned int msec);
void set_clock_source(void);
void set_clock_division_factor(unsigned char value);
void timer0interrupt(void);
void timerDelay(void);
unsigned char Read_APROM_BYTE(unsigned int code *u16_addr);
void Write_DATAFLASH_BYTE(unsigned int u16EPAddr, unsigned char u8EPData);
volatile unsigned char xdata page_buffer[128];
unsigned char rdata;
void timer1interrupt(void);
void Timer2Delay(void);
void setmode(void);
void runmode(void);
void switchscan(void);
void eepsave(void);
void conversion(void);
void muxdisable(void);
void ctreference(void);
void adcconversion(void);
void ontconversion(void);
void onttimer(void);
void zc(void);
void ovlcut(void);
void switchtime(void);
void currentsense(void);
void initialmux(void);
void frequency(void);
void analog(void);
void analogdisp(void);
void displaybuffering(void);
void ledblink(void);
void uart_init(void);
void transmit(char datax);
void uartsendstring(char *str);
int searchstring(const char *string,const char *keyword);
void extractthedata(const char *bufferx);
void datacompare(const char *compbuffer);
void formatinttoString(char *bufferr,uint16_t convertiondigit,char totaldigits);
void statushandler(void);
void newmessagecheck(void);
void split(void);  
                       //      0    1    2    3    4   5     6   7     8    9   10-   11o  12u  13L 14c 15H  16f  17O  18s 19e  20t   21A  22p  23m  24  25n  26d,  27i  28b
unsigned char xdata arr[30]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x40,0x5c,0x3e,0x38,0x58,0x76,0x71,0x5c,0x6d,0x7b,0x78,0x77,0x73,0xff,0x00,0x54,0x5e,0x30,0x7c};
unsigned int k,counter,dispc,dispc1,dispc2,disp1,counter1,counter2=0,counter4=0,q,resultt1=0,resultt=0,scan,resulttinst=0,rdata2,ovldisp1;
unsigned char  dp,display[4],seg,channel,mode,scan1,display6,model=0,smode1=0,d,switchset2,sw2,sw1,check,cttrip,smode,hvch,hvcl,decrement,zerohigh,zerodetect,trip=0,reset1,start1,lvc,display2,freqcount,ctscan,counter5,j,reg,rega,regx,segi,regx1,i,regb,freqscan,scan2,startdelay=3;
extern float sqrt  (float val);
unsigned int idata relaytrip=0,hvc=0,vref2=0,disp4=0,initialovl=0,counttime=0,ovtime=0,run2=0,ont=0,dec1=0,dec2=0;
unsigned int xdata dx1=0,kwh=0,kwdisp1=0,kwdisp=0,runhour=0,dispvd=0;
extern float sqrt  (float val);
double resultx1=0;
unsigned long int xdata kwhcount=0,kwhdisp1=0,kwhdisp=0,kwavg=0,kwhdisp2=0;
signed int result1=0;
signed long int xdata watt=0,iwatt=0,icur=0,resultx=0,result3=0,volt=0,cur=0;
signed int resultxvolt=0,resultxcurrent=0,resultvolt=0,resultcurrent=0;
unsigned int xdata result2=0,dx2=0,dx3=0,iwattdisp=0,temp=0;
signed int xdata dec=0;
unsigned int xdata bluetoothreset=0,ovldisp=0,runcount=0,disp2=0,start=0,dispv=226,disp=0,adcsample=0,adcsample1=108,runL=0,setreset=0,intref=0,intref1=0,resultpositive=0,resultnegative=0,vtime=0,dec3=0,dec4=0;
unsigned char idata counttime4=0,kwhtime=0,counttime2=3,counter6=0,runstart1=0,kwh1byte=0,kwh2byte=0,kwh3byte=0,kwh4byte=0,displaytime=0,delaytrip=0,zerolow=0,timermode=0,runstart=0,app=0,settrip=0,setskip=0,setmemsave=0,counter3=0,runeprom=0,ontimereprom=0,runontswap=0,memsave=0,tog=0,onteprom=0,ovldisph=0,ovldispl=0,relayoff=0,run2h=0,run2l=0,onth=0,ontl=0,run1=0,run8=0,run7=0,switchpress=0,ontimerstartrun=0,ontimerstart=0,runle=0,runleontrun=0,zerocross=0,channelio=0,tripsense=0,zerotesthigh=0,zerotestlow=0,countertrip=0,tripcount=0,initialstart=0,startpress=0,stoppress=0,voltdisp=0,switchset1=0,volthigh=0,voltlow=0,start3=0,distime=0,setswitch=0,switchset=0,press=0,restart=0,ovl=0,counttime1=0,switchdisp=0;
unsigned int xdata dx=0,runont=0,displaycttime=0,hvt=0,ovt=0,lvt=0,maxvolt=0,max=0,min=999,minmaxcount=0;
unsigned char xdata tempbuffer[250],receivedbuffer[50],txbuf[7],timerr=0,ledstate=0;
unsigned char xdata factoryreset1=0,factoryreset=0,displaycttimeh=0,displaycttimel=0,ovleeprom=0,errorled=0,ctcalibrated=0,resetdone=0,bluetootherror=0,bluetoothconfig=0,mineprom=0,minh=0,minl=0,maxl=0,maxh=0,setmemsave1=0,hvth=0,hvtl=0,lvth=0,lvtl=0,ovth=0,ovtl=0,appon=0,deviceoff=0,received_data=0,readx=0,readx1=0,search=0,rec=0,stat=0,newmessagearrived=0,turnoff=0,responsecheck=0,echooff=0,moduleready=0,nameready=0,disconnected=0,connected=0;
uint8_t pdata e=0,z=0,length=0,x=0,f=0,savex=0,countern=0,m=0,minvolt=0,elt=0,deviceon=0,status=0;
void main()
{
set_clock_source();//16MHZ
port();
statusled=0;
restore=1;
rst=0;
relay=0;
displaybuffering();
ledblink();
Read_APROM_BYTE(15895);
kwh1byte=rdata;
Read_APROM_BYTE(15896);
kwh2byte=rdata;
Read_APROM_BYTE(15897);
kwh3byte=rdata;
Read_APROM_BYTE(15898);
kwh4byte=rdata;
kwhdisp=((kwh1byte<<24)|(kwh2byte<<16)|(kwh3byte<<8)|(kwh4byte));
if(kwhdisp>=999999)
	kwhdisp=0;

Read_APROM_BYTE(15980);
lvc=rdata;
if(lvc>=250)
	 lvc=160;

Read_APROM_BYTE(15981);
initialstart=rdata;
if(initialstart>5)
initialstart=0;
if(initialstart==1)
ontimerstart=1;

Read_APROM_BYTE(15982);           
hvch=rdata;
Read_APROM_BYTE(15983);
hvcl=rdata;
hvc=((hvch<<8)|hvcl);
if(hvc>=330)
	 hvc=250;

Read_APROM_BYTE(15984);           
ovldisph=rdata;
Read_APROM_BYTE(15985);
ovldispl=rdata;
ovldisp=((ovldisph<<8)|ovldispl);
if(ovldisp>1000)
	 ovldisp=0;


Read_APROM_BYTE(15986);           
onth=rdata;
Read_APROM_BYTE(15987);
ontl=rdata;
ont=((onth<<8)|ontl);
if((ont>=50000))
   ont=0;//1

Read_APROM_BYTE(15988);           
run2h=rdata;
Read_APROM_BYTE(15989);
run2l=rdata;
run2=((run2h<<8)|run2l);
if((run2>=50000))
{
	  run2=0;
}
Read_APROM_BYTE(15991);
delaytrip=rdata;
if(delaytrip>10)
   delaytrip=0;

Read_APROM_BYTE(15992);           
hvth=rdata;
Read_APROM_BYTE(15993);
hvtl=rdata;
hvt=((hvth<<8)|hvtl);
if(hvt>=999)
	 hvt=0;

Read_APROM_BYTE(15994);           
lvth=rdata;
Read_APROM_BYTE(15995);
lvtl=rdata;
lvt=((lvth<<8)|lvtl);
if(lvt>=999)
	 lvt=0;

Read_APROM_BYTE(15996);           
ovth=rdata;
Read_APROM_BYTE(15997);
ovtl=rdata;
ovt=((ovth<<8)|ovtl);
if(ovt>=999)
	 ovt=0;
	
Read_APROM_BYTE(15998);
ctcalibrated=rdata;
if(ctcalibrated>=5)
	ctcalibrated=0;

Read_APROM_BYTE(15973);          
displaycttimeh=rdata;
Read_APROM_BYTE(15974);
displaycttimel=rdata;
displaycttime=((displaycttimeh<<8)|displaycttimel);
if(displaycttime>=50000)
	 displaycttime=0;

Read_APROM_BYTE(15975);          
minh=rdata;
Read_APROM_BYTE(15976);
minl=rdata;
min=((minh<<8)|minl);
if(min>=2000)
	 min=0;
	
Read_APROM_BYTE(15977);           
maxh=rdata;
Read_APROM_BYTE(15978);
maxl=rdata;
max=((maxh<<8)|maxl);
if(max>=2000)
	 max=0;
uart_init();
timer0interrupt();
dp=0;
display[1]=28;                  
display[2]=13;
display[3]=19;
ctreference();
while(bluetoothconfig==0)
{
if(bluetootherror==0)
{
uartsendstring("AT+BLEMODE=9\r\n");//to off the bluetooth for reintialize
turnoff=1;
bluetoothreset=0;
while(turnoff==1&&bluetootherror==0);
countern=0;
bluetoothreset=0;
uartsendstring("AT\r\n");//inital AT check to respond AT from bluetooth
responsecheck=1;
while(responsecheck==1&&bluetootherror==0);
countern=0;
bluetoothreset=0;
uartsendstring("ATE0\r\n");//to siwtch off the repeat AT command mode
echooff=1;
while(echooff==1&&bluetootherror==0);
countern=0;	
bluetoothreset=0;
uartsendstring("AT+BLEMODE=0\r\n");//bluetooth slave mode
moduleready=1;
bluetoothreset=0;
while(moduleready==1&&bluetootherror==0);
clearflag(status,0);
countern=0;
dp=0;}
if(bluetootherror==1)
{
	rst=1;
	delay(5000);
	delay(5000);
	rst=0;
	resetdone=1;
	delay(5000);
	delay(5000);
}
bluetoothconfig=1;
}

while(1)
	{		
if((smode==1)||(restart==1)||(cttrip==1))
switchscan();
if(((reset1==1)&&(trip==0)))
{
  muxdisable();
  trip=1;
}
if((reset1==1)&&(trip==1))
 {
		  relaytrip++;
	 if(relaytrip>200)
    {
  	 zc();
	   relay=1;
		 trip=0;
		 reset1=0;
     relaytrip=0;
		 counter=0;
		 Setflag(status,0);
	  }
 }
if((relayoff==1))
{
	   zc();
	   relay=0;
	   relayoff=0;
	   ontimerstart=0;
	   ontimerstartrun=0;
	   clearflag(status,0);
	
	   if(volthigh==1)
			 hvt=(hvt+1>999)?0:hvt+1;
	   if(voltlow==1)
			 lvt=(lvt+1>999)?0:lvt+1;
	   if(ovl==1)
			 ovt=(ovt+1>999)?0:ovt+1;
	     setmemsave1=1;
}

if(smode==0)
{
if(ont>=1)
   tog=1;
if(ont==0)
   tog=0;
if((tog==1)&&(relay==1))//ontimer
{
  run7++;
	if(run7>=210)
	{
	run7=0;
	run2=run2-1;
	runeprom=1;
	}
	if(run2==0)
	{
	relayoff=1;
	initialstart=0;
	run2=ont;
	runeprom=1;		
	}
}
  runmode();	
	ovlcut();
if(((smode==0)&&(restart==0)&&(relay==1)&&(volthigh==0)&&(voltlow==0)&&(ctcalibrated==1))||(disp1>=100))
{
	    disp4=disp1*100;
	    disp4=disp4/ovldisp;
	    if((disp4>150)||(disp1>=100))
			{
				disp4=150;
			}	
			if(disp1<(ovldisp-(ovldisp/10)))
			{
				ovl=0;
				clearflag(status,3);
			}
			if((disp1>ovldisp)||(disp1>=100))
			{
			  ovl=1;
        Setflag(status,3);				
			}
}

if((cttrip==0)&&(reset1==0))
{
	freqcount++;
	if(freqcount>=15)//
	{	
	frequency();
	freqcount=0;
	}
for(d=0;d<=25;d++)//500msec
	{	
	analog();
	switchscan();
	}
	currentsense();
  displaytime++;
	dx1=dispv+dx1;
	dx2=iwatt+dx2;
	dx3=resultxcurrent+dx3;
	if(displaytime>=4)//1sec
	{
		displaytime=0;
		dispvd=dx1/4;
		iwattdisp=dx2/4;
		dispc=dx3/4;
		disp1=dispc/10;
		dx1=0;
		dx2=0;
		dx3=0;
		if((dispc>5)&&(relay==1)&&(ctcalibrated==0)&&(disp1<100))
		{
			displaycttime++;
			if(disp1>(ovldisp+((ovldisp*30)/100)))
			{
				  ovldisp=disp1;
				  ovleeprom=1;
			}
			if(displaycttime>=360)//6hoursku 21600
			{
				 displaycttime=0;
				 ctcalibrated=1;
				 ovleeprom=1;
			}
		}
		if(dispvd<(min-10))//230
		{
		min=dispvd;
		mineprom=1;
		}
		if(dispvd>(max+10))//230
		{
		max=dispvd;
		mineprom=1;
		}
		minmaxcount++; 		
		if(minmaxcount>=43200)
		{
		minmaxcount=0;
		min=999;
		max=0;
	  }    
}

if((cttrip==0)&&(restart==0)&&(volthigh==0)&&(runstart==1)&&(voltlow==0)&&(relay==0)&&(start3==0)&&(ontimerstartrun==1)&&(initialstart==1))
{
	reset1=1;
}
}
}
if(smode==1)
 {
	 setmode();
	 setreset++;
	 if((setreset>12498)&&(smode1==5))
	 { 
	 setmemsave=1;
	 smode=0;
	 smode1=0;
	 counter1=0;
	 counttime=0;
	 counttime4=0;
	 setreset=0;
	 cttrip=0;
	 restart=0;
	 settrip=0;
	 dispc=0;
	 ovl=0;
	 if(initialstart==1)
	 ontimerstart=1;
	 }
 }
 
if(connected==1&&smode==0&&bluetootherror==0)
	{
		   statusled=1;
		if(ledstate==41)
		{
			if(deviceon==1)
			{
				if(relay==0)
				   appon=1;
				   deviceon=0;
			}
			if(deviceoff==1)
			{
			  if(relay==1)
				  appon=1;
				  deviceoff=0;
			}
		}
		if(ont<=0)
		clearflag(status,1);
		else
		Setflag(status,1);
 	  if(delaytrip==1)
	  Setflag(status,2);
    else
	  clearflag(status,2);
		statushandler();//led&status check
		timerr++;
		if((timerr>=40)&&(setmemsave==0)&&(factoryreset==0))//1min one transfer240
		{
			timerr=0;
			uartsendstring("AT+BLESEND=29,");
			uartsendstring("V-");
	formatinttoString(txbuf,dispvd,3);//convert in to character
	uartsendstring(txbuf);//send volt
			
			uartsendstring("C-");
				 formatinttoString(txbuf,dispc,3);
	uartsendstring(txbuf);//send current
			
			uartsendstring("W-");
			 formatinttoString(txbuf,iwattdisp,4);//watt
  uartsendstring(txbuf);
			
			uartsendstring("K-");
		formatinttoString(txbuf,kwhdisp1,6);//kvh
  uartsendstring(txbuf);
			
						uartsendstring("D-");
			formatinttoString(txbuf,ovldisp,3);//ovl
  uartsendstring(txbuf);
			
	/*uartsendstring("AT+BLESEND=53,");//send command55
  uartsendstring("IPB");//product-identifier
	 formatinttoString(txbuf,dispvd,3);//convert in to character
	uartsendstring(txbuf);//send volt
	 formatinttoString(txbuf,dispc,3);
	uartsendstring(txbuf);//send current
	  formatinttoString(txbuf,run2,3);//ont	
	uartsendstring(txbuf);//sent ont
	  formatinttoString(txbuf,min,3);
  uartsendstring(txbuf);//minvolt		
	  formatinttoString(txbuf,max,3);
	uartsendstring(txbuf);//maxvolt
	  formatinttoString(txbuf,iwattdisp,4);//watt
  uartsendstring(txbuf);
		formatinttoString(txbuf,kwhdisp1,6);//kvh
  uartsendstring(txbuf);
	 formatinttoString(txbuf,lvt,3);//lvt
  uartsendstring(txbuf);
	  formatinttoString(txbuf,hvt,3);//hvt
  uartsendstring(txbuf);
	  formatinttoString(txbuf,ovt,3);//ovt
  uartsendstring(txbuf);
	  formatinttoString(txbuf,0,3);//only for elcb so 0 for this product 
  uartsendstring(txbuf);
	//uartsendstring("-");
	transmit(48+status);
	//uartsendstring("-");
	//set mode parameters
	  formatinttoString(txbuf,hvc,3);//hvc 
  uartsendstring(txbuf);
	  formatinttoString(txbuf,lvc,3);//lvc 
  uartsendstring(txbuf);
	  formatinttoString(txbuf,ovldisp,3);//ovl
  uartsendstring(txbuf);
		formatinttoString(txbuf,ont,3);//ovl
  uartsendstring(txbuf);*/
	uartsendstring("\r\n");//enddata 
		}
	}
if(disconnected==1&&smode==0&&bluetootherror==0)
	{
		statusled=0;
	statushandler();//led&status check we are use 
	}
if(bluetootherror==1)
{
	errorled++;
	if(errorled>2)
	{
	statusled=!statusled;
		errorled=0;
	}
	
	
}	
 //

}
}

void delay(unsigned int msec)
{
	unsigned int i,j;
	for(i=0; i<msec; i++)
	for(j=0; j<50; j++);
}
void ctreference(void)
{
	resultx=0;
  ENABLE1_ADC_AIN4;
  for(scan=0;scan<50;scan++)
	{
	  adcconversion();
		resultx=result1+resultx;
	}
   intref=resultx/50;
if(intref>520||intref<501)
{
			 intref=511;
}
}


void ovlcut(void)
{
if((ovl==1)&&(cttrip==0))
 {
if((restart==0)&&(smode==0))
	{
		  ovtime++;
			initialovl=((150-disp4));
			initialovl=(7+(initialovl));//11264//8448
	}
if(ovtime>initialovl)//For disp off and relay on
{	 
	 relayoff=1;
	 ovtime=0;
	 restart=1;
	 initialovl=0;
	 settrip=1;
	 counter=0;
}  
}
}


void port(void)
{
	/*SFRS=0;
	P00_PUSHPULL_MODE;
	P01_PUSHPULL_MODE;
	P02_PUSHPULL_MODE;
	P03_PUSHPULL_MODE;
	P04_PUSHPULL_MODE;
	P05_INPUT_MODE;
	P06_INPUT_MODE;
	P07_QUASI_MODE;
	P1M1=0;
	P1M2=0xFF;
	P30_INPUT_MODE;                      //setting INT0 pin P3.0 as Quasi mode with internal pull high*/
   SFRS=0;
	P00_PUSHPULL_MODE;
	P01_PUSHPULL_MODE;//led
	P02_PUSHPULL_MODE;//relay
	P03_PUSHPULL_MODE;
	P04_INPUT_MODE;//switch
	P05_INPUT_MODE;//CT
	P06_PUSHPULL_MODE;
	P07_INPUT_MODE;
	P1M1=0x00;
	P1M2=0xFF;//p16 rst,p17 restore
	P30_INPUT_MODE;  //voltage      
}
void set_clock_source(void)
{
	set_CKEN_HIRCEN;         
  while((CKSWT & SET_BIT5) == 0);
}
void set_clock_division_factor(unsigned char value)
{
  CKDIV = value;
}

void initial(unsigned char time)//100ussec delay
{
	for(k=0;k<time;k++)
	{
	  TMOD = TMOD|0x20;
	  TH1 = 121;        
    TR1 = 1;	
    while(TF1 == 0);	
    TF1 = 0;     
    TR1 = 0;
	}
}
void adcconversion(void)
{
	    clr_ADCCON0_ADCF;
      set_ADCCON0_ADCS;                  
      while(ADCF == 0);
		  result1=(ADCRH<<4)+(ADCRL&0x0F);
	    result1=result1>>2;
}

void timer0interrupt(void)
{
  TMOD = TMOD|0x01;
	TH0 = 0XFE; //FE  
  TL0 = 0X0B;//0B   350usec
	TR0 = 1;           
  ET0 = 1;           
  EA =  1; 
  TR0 = 1;	
}
void UART_ISR(void) interrupt 4 
{
	if(RI)//receive all incommingdata stored in tempbuffer 
	{
		RI=0;
		received_data=SBUF;
		newmessagearrived=0;
		if((received_data!='\n')&&(received_data!='\r')&&(received_data!='#'))
    {
     tempbuffer[m++]=received_data;
    }
		readx=0;//wait untill the newmessage arriving
		readx1=0;
		countern=0;
		if(m>(sizeof(tempbuffer-1)))//need to check
			  m=0;
		newmessagearrived=1;
	}	
}
void timer0_isr() interrupt 1
{
	TH0 = 0XFE; //65035 //375usec
  TL0 = 0X0B;
	TF0 = 0;
	if(bluetoothconfig==0)
	{
		  bluetoothreset++;
		if(bluetoothreset>50000)
		{
			bluetoothreset=0;
			bluetootherror=1;
		}
	}
	
		readx++;
	if(readx>60)//we need some delay to evaluate thedata so i make some delay
	{
		readx=0;
		readx1++;
	if(readx1>=3)
		{
		readx1=0;
		newmessagecheck();// all new message comparing here
		}
	}
	
if(factoryreset==1)
{
	
	muxdisable();
	ctcalibrated=0;
  displaycttime=0;
	ovldisp=0;
	kwhdisp1=0;
	kwhdisp=0;
	factoryreset=0;
	factoryreset1=1;
	ovleeprom=1;
}
if((relay==1)&&(resultxcurrent>=5))
{
runhour++;
if(runhour>=2666)
{
kwh=((kwavg*100)/3600);
kwhcount=kwhcount+kwh;
kwhdisp1=((kwhcount/1000)*1.07);//
kwhdisp1=kwhdisp1+kwhdisp;
if(kwhdisp1>99999)
kwhdisp1=0;
runhour=0;
kwhtime++;
}	
}

if((kwhtime>=10)||(factoryreset1==1))
{
	muxdisable();
	kwh1byte=(kwhdisp1>>24)&0XFF;
	kwh2byte=(kwhdisp1>>16)&0XFF;
	kwh3byte=(kwhdisp1>>8)&0XFF;
	kwh4byte=(kwhdisp1)&0XFF;
	Write_DATAFLASH_BYTE(15895,kwh1byte);
	Write_DATAFLASH_BYTE(15896,kwh2byte);
	Write_DATAFLASH_BYTE(15897,kwh3byte);
	Write_DATAFLASH_BYTE(15898,kwh4byte);
	kwhtime=0;
	factoryreset1=0;
}

if(runeprom==1)
{
	runeprom=0;
  muxdisable();
	run2h=run2/256;
	run2l=run2%256;
	Write_DATAFLASH_BYTE(15988,run2h);
	Write_DATAFLASH_BYTE(15989,run2l);
	Write_DATAFLASH_BYTE(15981,initialstart);
}
if(setmemsave==1)
{
	muxdisable();
	eepsave();
	setmemsave=0;
}
if(mineprom==1)
{
		    muxdisable();
	      minh=min/256;
	      minl=min%256;
	      Write_DATAFLASH_BYTE(15975,minh);
				Write_DATAFLASH_BYTE(15976,minl);
	
		    maxh=max/256;
	      maxl=max%256;
	      Write_DATAFLASH_BYTE(15977,maxh);
				Write_DATAFLASH_BYTE(15978,maxl);
	      mineprom=0;
}
if(ovleeprom==1)
{
	muxdisable();
	ovldisph=ovldisp/256;
	ovldispl=ovldisp%256;
	Write_DATAFLASH_BYTE(15984,ovldisph);
	Write_DATAFLASH_BYTE(15985,ovldispl);
	Write_DATAFLASH_BYTE(15998,ctcalibrated);		

	displaycttimeh=displaycttime/256;
	displaycttimel=displaycttime%256;
	Write_DATAFLASH_BYTE(15973,displaycttimeh);
	Write_DATAFLASH_BYTE(15974,displaycttimel);

//displaycttime	
	ovleeprom=0;
	
}

if(setmemsave1==1)
{
	     muxdisable();
	//hvt,lvt,ovt
	      hvth=hvt/256;
	      hvtl=hvt%256;
	      Write_DATAFLASH_BYTE(15992,hvth);
				Write_DATAFLASH_BYTE(15993,hvtl);
				
			  lvth=lvt/256;
	      lvtl=lvt%256;
	      Write_DATAFLASH_BYTE(15994,lvth);
				Write_DATAFLASH_BYTE(15995,lvtl);
	
        ovth=ovt/256;
	      ovtl=ovt%256;
	      Write_DATAFLASH_BYTE(15996,ovth);
				Write_DATAFLASH_BYTE(15997,ovtl);	
	      setmemsave1=0;
}

if((ontimerstart==1)&&(sw2==0)&&(relay==0)&&(volthigh==0)&&(voltlow==0)&&(ovl==0)&&(smode==0))//LED blinking
{
	runL++;
	if(runL>1306)
	{
		runstart1=runstart1+1;//for ondelay
		runle=!runle;
		runL=0;
	}
}

if(runstart1>=20)
{
	ontimerstartrun=1;
	runstart1=0;
}



if(((sw2==1)&&(switchset2==1)&&(smode==0)&&(restart==0)&&(cttrip==0)&&(volthigh==0)&&(voltlow==0))||(appon==1&&ovl==0))
{
	   
	   switchtime();
	if((counttime1==0)||(appon==1))
	{
		appon=0;
		counttime1=2;
		switchset2=0;
		switchdisp=0;
		ovl=0;
		restart=0;
		cttrip=0;
		startdelay=3;
		initialstart=!initialstart;
		if(initialstart==0)
		{
      relayoff=1;				
		}
		if(initialstart==1)
		{
		ontimerstart=1;
		}
		muxdisable();
		Write_DATAFLASH_BYTE(15981,initialstart);
	}   
}

if((((smode==0)&&(sw2==1)&&(switchset2==1)&&((restart==1)||(cttrip==1))))||(appon==1&&ovl==1))
{
	   switchtime();
	if((counttime1==0)||(appon==1))
	{
		appon=0;
		counttime=0;
		counttime1=2;
		switchset2=0;
		switchdisp=0;
		ovl=0;
		restart=0;
		ontimerstartrun=0;
		cttrip=0;
		settrip=0;
		counter6=0;
		initialstart=1;
	  ontimerstart=1;
	}   
}

if((sw1==1)&&(smode==0))
{
	counttime1=counttime2;
	counttime++;
	if(counttime>704)
		switchdisp=1;
  if(counttime>2816)
	{
    counttime2=counttime2-1;
		counttime=0;
	}
	if(counttime2==0)
	{
		counttime=0;
		counttime2=0;
    smode=1;
		switchset2=0;
		switchset=0;
		//counttime=0;
	  if(smode==1)
		{	
		display[1]=18;
    display[2]=19;
	  display[3]=20;
		dp=0;
		counter=0;
		relay=0;
		vtime=0;
		ovtime=0;
		voltlow=0;
		volthigh=0;
		restart=0;
		ovl=0;
		ontimerstartrun=0;
		}
	}
}

if(reset1==0)
{
	P1&=~(0x3f);
segi++;
if(segi>11)//9
{
		segi=1;
	   seg++;
	 if(seg>2)
		seg=0; 
 }
if(segi<8)
{
	/* In old code we are using if condition to make every individual pin high state . it took more memory(1.5 kb), so we are going to use
 this logic for memory optimization . it takes only 500 kb so in future we are going to use this logic for 6 pin segment	
	*/
	on=1;
rega=arr[display[seg+1]];// we just check every single bit is it 1 or zero
if(rega&(1<<(segi-1)))// if that bit is one this function will execute
	{
		P1M1=segmentData[segi-1][seg].Port&0x3F;//P1M1 and P2M2 these resgisters are pin mode configuration register . why we are check it 0x3f because we are going to use only 6 pins so we only check that 6 pins so other 2 pin for other use
    P1M2=segmentData2(segmentData[segi-1][seg].Port)|0xC0;// why we make or gate with 0xc0 the last to pins are we want to maintain in always high state.
    P1|=segmentData[segi-1][seg].Pin;//This register only make it pin high or low using this register , this register values are stored in array so we just called it
	}
//else
//P1&=~(0x3f);
}
if(segi==8)
{
	on=1;
	if((seg==1)&&(dp==1))
	{
		P1M1=(0x3C&0x3f);
		P1M2=((~0x3c)|0xc0);
		P1|=(1<<1);
	}
	if((seg==2)&&(dp==2))
	{
		P1M1=(0x3A&0x3f);
		P1M2=((~0x3A)|0xc0);
		P1|=(1<<2);
	}
	if(dp==0)
	{
		P1&=~(0x3f);
	}
}
if(segi==9)
{
if((initialstart==1)&&(ontimerstart==1)&&(ontimerstartrun==0)&&(relay==0)&&(volthigh==0)&&(voltlow==0)&&(ovl==0)&&(smode==0))
	{
	if(runle==0)
		{
		P1M1=0x1F;
		P1M2=0xe0;
		P1&=~(0x3F);
		on=0;
	  }
		if(runle==1)
		{
		P1M1=0x1F;
		P1M2=0xe0;
		P1&=~(0x3F);
	  on=1;
		}
	}
if((smode==0)&&(ontimerstartrun==1)&&(ovl==0)&&(voltlow==0)&&(volthigh==0)&&(cttrip==0)&&(relay==1))
	{
		P1M1=0x1F;
		P1M2=0xe0;
		P1&=~(0x3F);
		on=0;
	}
	if(ovl==1)
	{
	P1M1=0x2b;
	P1M2=0xd4;
	P1&=~(0x3F);
		on=0;
	}
}
if((segi==10)&&((volthigh==1)||(voltlow==1))&&(smode==0)&&(cttrip==0))
{
	P1M1=0x3b;
	P1M2=0xC4;
	P1&=~(0x3F);
	on=0;
}
if((segi==11)&&(ont>=1))
{
	P1M1=0x3e;
	P1M2=0xC1;
	P1&=~(0x3F);
	on=0;
}
}
}
void frequency(void)
{   
	       resultx=0;
		     result2=10;
    while(result2>4)
    {
		  ENABLE1_ADC_AIN1;
			adcconversion();
		  if(result1>=511)
      result2=result1-511;
      if(result1<511)
      result2=511-result1;
    }
	    //relay=1; 
    T2MOD = T2MOD|0xA0;//prescalar 16mHZ/16
	  TH2 = 0X00;        
    TL2 = 0X00;
    TR2 = 1;	   //timer2ON  
		initial(50);//5msec delay
	  result2=10;
    while(result2>2)
    {
		  ENABLE1_ADC_AIN1;
			adcconversion();
	    if(result1>=511)
      result2=result1-511;
      if(result1<511)
      result2=511-result1;
    }
    //relay=0;
    TR2 = 0;
		resultt=((TH2<<8)|TL2);
		resultt=resultt*4;//half cycle
	  resulttinst=resultt/360;
	  adcsample=((25500/191)*resulttinst);
		adcsample1=adcsample/100;
		adcsample1=255-adcsample1;
		if(adcsample1<97)//47hz
			adcsample1=97;
		if(adcsample1>118)//54//182 for 50hz
			adcsample1=118;
}

void analog(void)
{
	 //adcsample1=108;
   zc();
	 iwatt=0;
	 resultxvolt=0;
	 
for(scan1=0;scan1<2;scan1++)
{	
for(scan=0;scan<37;scan++)//8.4msec//145 //41 for 4.5msec
	{	  
		  TMOD = TMOD|0x20;//8bit timer
		  TH1 = adcsample1;//182 for 50hz   //55usec timer  //108 for110usecfor50,100for47,121for54//59,54,51
      TR1 = 1;
		  	     
		  ENABLE1_ADC_AIN1;//29usec
			adcconversion();
      resultvolt=result1;
		
			ENABLE1_ADC_AIN4;//29usec
			adcconversion();
		  resultcurrent=result1;
      
		  if(resultvolt>=512)///3usec
      volt=resultvolt-512;
      if(resultvolt<512)
      volt=512-resultvolt;
      
			if(resultcurrent>=intref)//3usec
      cur=resultcurrent-intref;
      if(resultcurrent<intref)
      cur=intref-resultcurrent;
			
			resultxvolt=resultxvolt+volt;//2.20usec
			//resultxcurrent=resultxcurrent+cur;//from adc start to here 67usec
			
			watt=((volt)*(cur));//1.26,0.2if multiplication done with fraction its 136usec extra
			iwatt=watt+iwatt;
			
      while(TF1 == 0);     			
      TF1 = 0;     
      TR1 = 0;
	}
	if(scan1==0)
	 {
	disp2=resultxvolt/14.8;
	if(disp2>=350)
	{
	start3=1;
	volthigh=1;
	relay=0;
	ontimerstartrun=0;
	}
   }
}
	
  iwatt=iwatt/1731;//315.7
  if(iwatt<0)
  iwatt=iwatt*(-1);//watts
  kwavg=iwatt; 
	
	
	disp=resultxvolt/37.4;
	distime++;
  dx=dx+disp;
  if(distime>=10)//100msec
	{
	dispv=dx/10;
	dx=0;
	distime=0;
	runstart=1;
	}
		
	if(disp>=350)
	{
	start3=1;
	volthigh=1;
	relay=0;
	ontimerstartrun=0;
	}
	 
 if((dispv<hvc-5)&&(disp<hvc-5))
 {
  volthigh=0;
 }
 if((dispv>lvc+5))
 {
  voltlow=0;
 }
 if((volthigh==0)&&(voltlow==0)&&(ovl==0))
 {
	if((start3==1)&&(initialstart==1))
	ontimerstart=1;
  start3=0;
  vtime=0;
 }
 if((dispv>hvc))
     volthigh=1;
 if((dispv<lvc))
     voltlow=1;
 
 if((((dispv>hvc)&&(dispv<hvc+20)&&(delaytrip==0))||(dispv<lvc))||((dispv>hvc)&&(delaytrip==1)))
 {
   vtime++;
 }
 if((dispv>=hvc+20)&&(dispv<hvc+40)&&(delaytrip==0))
	   vtime=vtime+2;
 if((dispv>=hvc+40)&&(delaytrip==0))
	   vtime=vtime+4;
 if(((vtime>1400)&&(delaytrip==0))||((delaytrip==1)&&(vtime>=1)))
	{	
    if(delaytrip==0)
    relayoff=1;
		if(delaytrip==1)
    relay=0;
		vtime=0;
		start3=1;
		ontimerstartrun=0;		
	}	 
	if(voltlow==1)
  Setflag(status,4);
  if(voltlow==0)
  clearflag(status,4);
  if(volthigh==1)
  Setflag(status,5);
  if(volthigh==0)
  clearflag(status,5);
  clearflag(status,6);
  clearflag(status,7);
}

void currentsense(void)
{
	//adcsample1=108;
	resultx=0;
	result3=0;
	cur=0;
for(scan=0;scan<90;scan++)//3degree 105usec
	{
		  TMOD = TMOD|0x20;//8bit timer
		  TH1 = adcsample1;//182 for 50hz   //55usec timer  //108 for110usec 
      TR1 = 1;
		  	   
			ENABLE1_ADC_AIN4;//29usec
			adcconversion();
		  resultcurrent=result1;
      
			if(resultcurrent>=intref)
      cur=resultcurrent-intref;
      if(resultcurrent<intref)
      cur=intref-resultcurrent;
			
			result3=cur*cur;
			resultx=resultx+result3;	
      while(TF1 == 0);
      TF1 = 0;     
      TR1 = 0;
	}
  resultx1=resultx/60;
	icur=sqrt(resultx1);
	icur=(icur*100);
  resultxcurrent=icur/48.2;//7.26
	if(resultxcurrent<=5)
		resultxcurrent=0;
 	 
	if((restart==0)&&(ovl==0))
	{
			ovtime=0;
	}
  if((relay==0)&&(restart==0))
	{
			ovl=0;
			vtime=0;
	}
}

void runmode(void)
{	
	if((restart==1)&&(ovl==1))
	{
	counter++;
	if(counter<6)//2sec
	{
	if(ovl==1)
    {
	dp=0;
	display[1]=11;                  
  display[2]=12;
  display[3]=13;
    }
	}
	if((counter>=6)&&(counter<18))//6sec
	{
	dp=1;
	temp=dispc;
	split();
	}
	if(counter>=18)
     counter=0;		
	}
	if((cttrip==0)&&(switchdisp==0)&&(restart==0))
	{
	 counter++;//1,4,1,8,4
	if(counter<3)//1sec
	{
	dp=0;
	display[1]=10;                  
  display[2]=12;
  display[3]=10;	
	}
	if((counter>=3)&&(counter<12))//4sec
	{
	dp=0;
	temp=dispvd;//dispvd
	split();		
	}
	if((volthigh==0)&&(voltlow==0)&&(ovl==0)&&(initialstart==0))
  {
	if(counter>=12)
	counter=0;
  }
	if((ontimerstartrun==1)&&(initialstart==1))
	{
	if(counter>12&&counter<=15)//1sec
	{
	dp=0;
	display[1]=10;                  
  display[2]=21;
  display[3]=10;
	}
  if(counter>15&&counter<23)//3sec
	{
	dp=1;
	temp=dispc;
	split();
	}
	if((counter>=23)&&(tog==0))
		  counter=0;
	if(tog==1)
	{
	if(counter>=23&&counter<26)//1sec
	{
	dp=0;
	display[1]=11;                  
  display[2]=25;
  display[3]=20;
	}
	if(counter>=26&&counter<35)//3sec
	{
	dp=0;
	temp=run2;
	split();
	}
  if(counter>=35)
  counter=0;	
	}
  }
if((ontimerstartrun==0)&&((volthigh==1)||(voltlow==1)))
{
	if(counter>12&&counter<23)//3sec
	{
  if((volthigh==1))
  {
	dp=0;
	display[1]=15;                  
  display[2]=12;
  display[3]=14;
  }
if((voltlow==1))
{
	dp=0;
	display[1]=13;                  
  display[2]=12;
  display[3]=14;
}
	}
if(counter>23)
	{
	  counter=0;
	}
}
}
	if((initialstart==1)&&(ontimerstart==1)&&(ontimerstartrun==0)&&(relay==0)&&(volthigh==0)&&(voltlow==0)&&(ovl==0))
	{
	dp=0;
	display[1]=10;                  
  display[2]=10;
  display[3]=10;
	}
 if((cttrip==1)&&(switchdisp==0))
	{
	dp=0;
  display[1]=16;
  display[2]=13;
	display[3]=20;	
	}
 if((switchdisp==1))
  {
	dp=0;
  if(counttime1>99)
	counttime1=99;
	display[1]=22;
  display[2]=counttime1/10;
	display[3]=counttime1%10;	
  }
}

void split(void)
{
	if(temp>999)
	{
		temp=999;
	}
	display[1]=temp/100;
	display2=temp%100;
  display[2]=display2/10;
	display[3]=display2%10;
}


void setmode(void)
{
if((sw2==1)&&(smode==1)&&(sw1==0)&&(switchset==1))
	{
	  counter++;
if((counter>125)&&(smode==1))//75	
{
  counter=0;
	switchset=0;
	switchset1=1;
  smode1=smode1+1;
	switchpress=0;
	setreset=0;
	
	if(memsave==1)
	{
		setmemsave=1;
		memsave=0;
	}
	if((sw2==1)&&(smode1==6))
	{
		smode1=1;
	  setreset=0;
	}
	counter4=480;	  
}
}
	
if(((smode1==1)||(smode1==2)||(smode1==3)||(smode1==4)||(smode1==5))&&(switchset1==1))
{
	   counter2++;
	if(counter2<counter4)
	 {
		 setswitch=1;
		 if(smode1==1)
		 {
		display[1]=11;//ont
    display[2]=25;
	  display[3]=20;
		dp=0; 
		 }
		if(smode1==2)
		{
		display[1]=15;//hvc
    display[2]=12;
	  display[3]=14;
		dp=0;
		}
		if(smode1==3)
		{
		display[1]=13;//lvc
    display[2]=12;
	  display[3]=14;
		dp=0;
		}
		if(smode1==4)
		{
		display[1]=14;//tripselection
    display[2]=10;
		if(delaytrip==0)
		display[3]=26;
		if(delaytrip==1)
		display[3]=1;
		dp=0;
		}
		if(smode1==5)
		{
		display[1]=19;//esc
    display[2]=5;
	  display[3]=14;
		dp=0;
		}
	 } 
	if((counter2>counter4))
	{
     setswitch=0;
		if(smode1==1)
		 {
		 dec=ont;
	   
		 }
		if(smode1==2)
		{
			dec=hvc;
			dp=0;
			
		}
		if(smode1==3)
		{
			dec=lvc;
			dp=0;
		}
	 if((smode1==2)||(smode1==3))
	 {		 
	 conversion();
	 }
   if(smode1==1)
	 {
    ontconversion();
	 }		 
		switchset1=0;
		counter2=0;
		switchpress=1;
	}
}
if((sw1==1)&&(switchpress==1)&&((smode1==1)||(smode1==2)||(smode1==3)||(smode1==4)))
{
	
	memsave=1;
	//switchpress=0;
	counter1++;
	setreset=0;
	if(counter1>300)
	{
		counttime4=1;
		counter1=0;
	}
	if((counttime4==1)&&(check==1))
	{
		counter3=counter3-5;
		check=0;
	if(counter3<=5)
	{
		counter3=5;
	}
  }
if((counter1>20)&&(switchset2==1)&&(counttime4==0))
{
  counter1=0;
	switchset2=0;
	decrement=1;
}
if((counter1>counter3)&&(counttime4==1))
{
	 decrement=0;
	 check=1;
	 counter1=0;
	 switchset2=0;
	if(smode1!=5)
	{
	  dec=dec+1;
	if(smode1==1)
	{
		if(dec>599)
		 dec=0;
	   ont=dec;
	}
	if(smode1==2)
		{
			if(dec>320)
		  dec=230;
			hvc=dec;
		  dp=0;
		}
	if(smode1==3)
		{
			if(dec>230)
		  dec=150;
			lvc=dec;
		  dp=0;
		}
	if((smode1==2)||(smode1==3))
	 {		 
	 conversion();
	 }
   if(smode1==1)
	 {
    ontconversion();
	 }
  }
}
}
if((decrement==1)&&(switchset2==1))
{
	 decrement=0;
	 switchset2=0;
  if((smode1==1)||(smode1==2)||(smode1==3)||(smode1==4))
    {
	if(dec==0)
	dec=1-dec;
	if(dec>0)
	dec=dec-1;
	
	if(smode1==1)
	{
	if(dec<1)
	dec=0;
	ont=dec;
	dp=1;
  }
	if(smode1==2)
	{
	if(dec<230)
	dec=320;
	hvc=dec;
	dp=0;
  }
	if(smode1==3)
	{
	if(dec<150)
	dec=230;
	lvc=dec;
	dp=0;
  }
	if(smode1==4)
	{
	delaytrip=!delaytrip;
	display[1]=14;//tripselection
  display[2]=10;
	if(delaytrip==0)
	display[3]=26;
	if(delaytrip==1)
	display[3]=1;
	dp=0;	
	}
	if((smode1==2)||(smode1==3)||(smode1==4))
	 {		 
	 conversion();
	 }
   if(smode1==1)
	 {
    ontconversion();
	 }
}
}
}

void ontconversion(void)
{
	if(dec>=1)
	{
	dp=1;
	dec1=dec/60;
	display[1]=dec1%10;
	dec2=dec%60;
	display[2]=dec2/10;
	display[3]=dec2%10;
	}
	if(dec==0)
	{
	dp=0;
	display[1]=11;
	display[2]=16;
	display[3]=16;
	}
}

void conversion(void)
{
	 display[1]=dec/100;
	 display2=dec%100;
   display[2]=display2/10;
	 display[3]=display2%10;
}

void eepsave(void)
{
	
	      //ovldisph=ovldisp/256;
	      //ovldispl=ovldisp%256;
	      //Write_DATAFLASH_BYTE(15984,ovldisph);
				//Write_DATAFLASH_BYTE(15985,ovldispl);
	
	      onth=ont/256;
	      ontl=ont%256;
	      Write_DATAFLASH_BYTE(15986,onth);
	      Write_DATAFLASH_BYTE(15987,ontl);
	      run2=ont;
	      runeprom=1;
	
	      hvch=hvc/256;
				hvcl=hvc%256;
				Write_DATAFLASH_BYTE(15982,hvch);
				Write_DATAFLASH_BYTE(15983,hvcl);
	
				Write_DATAFLASH_BYTE(15980,lvc);    
	      Write_DATAFLASH_BYTE(15991,delaytrip);
}

void muxdisable(void)
{
	P1&=~(0x3f);
	on=1;	
}

void switchscan(void)
{
		 result1=0;
		 resultx=0;
  ENABLE1_ADC_AIN5;
  for(scan=0;scan<25;scan++)
	{
	  adcconversion();
		resultx=result1+resultx;
	}
 vref2=resultx/25;

if((vref2>480)&&(vref2<=530))//switch1 press
{
	stoppress=0;
	startpress++;
	if(startpress>4)
	{
	sw1=1;
	sw2=0;
	}
}
if((vref2>300)&&(vref2<=350))//switch 2 press
{
	startpress=0;
	stoppress++;
	if(stoppress>4)
	{
	sw2=1;
	sw1=0;
	}
}

if(vref2>550)
{
	press=0;
	sw1=0;
	sw2=0;
	switchset=1;
	switchset2=1;
	counter1=0;
	counter3=150;
  counttime=0;
  counttime4=0;
	counttime2=3;
  counttime1=2;
  switchdisp=0;	
	startpress=0;
	stoppress=0;
}
}

void initialmux(void)
{
/*for(q=0;q<150;q++)
{	
for(i=1;i<4;i++)
{
if(i==1)
{
on=1;
mux1=1;
mux2=0;
mux3=0;
}
if(i==2)
{
on=1;
mux1=0;
mux2=1;
mux3=0;
}
if(i==3)
{
on=1;
mux1=0;
mux2=0;
mux3=1;
}
delay(1);
muxdisable();
delay(7);
}
}	*/
}
void switchtime(void)
{
	counttime++;
	if(counttime>704)
		switchdisp=1;
  if(counttime>2816)
	{
    counttime1=counttime1-1;
		counttime=0;
	}		
}
void zc(void)
{
	result2=10;
  while(result2>7)
  {		
	ENABLE1_ADC_AIN1;
	adcconversion();
	if(result1>=511)
  result2=result1-511;
  if(result1<511)
  result2=511-result1;
	}
}
void uart_init(void)
{
	//SFRS=0;
//	P0M1=0X80;
	//P0M2=0X7F;
	P0|=(1<<6);
  AUXR1 =0xfb;
	PCON |= 0x80; 
	SCON &=~(1<<5);
	SCON |=0x50;
	IP |=(1<<4);
	IPH |=(1<<4);
  T3CON &= 0xF8;  
	T3CON |= (1<<5);
  RH3 = 0xff;
  RL3 = 0x98;
  IE|=(1<<4);
  T3CON|=(1<<3); 	
}
void transmit(char datax)
{
	SBUF=datax;
	while(TI==0);
	TI=0;
}
void uartsendstring(char *str)
{     while(*str)
		{transmit(*str++);} 
}
int searchstring(const char *string,const char *keyword)// usingthis function we get the expected string is arraived or not thats it
{
    for(search=0;string[search]!='\0';search++)// check untill the string end
    {
        if(string[search]==keyword[0])//if keyword is ok it's execute
        {
            for(z=0;keyword[z]!='\0';z++)//it chechs untill the string end..
            {
            if(string[search+z]!=keyword[z])//check string match or not
                    break;
            }
            if(keyword[z]== '\0')
                return 1;
        }
     }
   return 0;
}
void extractthedata(const char *bufferx)
{
		  length=0;
	    x=0;
   while(bufferx[x]!='\0')
    {
			if((bufferx[x]=='+')&&(bufferx[x+1]=='D')&&(bufferx[x+2]=='A')&&(bufferx[x+3]=='T')&&(bufferx[x+4]=='A')&&(bufferx[x+5]==':'))//we get likethis>||<+DATA:10,helloworld we didn't get fixed sizedata so we take length and then we stotre the Data  
      {
		//we don't know the string end so we use the buffer Data length for identify the string ..
          if(bufferx[x+7]==',')//check comma placed expected positon or not if its ok then we take the stringsize
          {
              length=((bufferx[x+6]-48)*1);//here i use chartoint conversion  because if i get length like 20,it comes 2,0 if want the value like 20 , i need to merge two values soo.. single digit value is no issues but we receive more than 9 we need this conversion
              x+=8;
              length=((length+x)-1); 					
          }
          if((bufferx[x+7]!=',')&&(bufferx[x+8]==','))
          {
              length=(((bufferx[x+6]-48)*10)+((bufferx[x+7]-48)*1));
              x+=9;
              length=((length+x)-1);
		
          }
          memset(receivedbuffer,0,sizeof(receivedbuffer));//before storing we clear the garbage or previous memory 
					rec=0;
          break; 
      }
      x++;
   }
	if(length==0)
		 return;
     for(x=x;x<=length;x++)
     {
      receivedbuffer[rec++]=bufferx[x]; // storing the Data to the received buffer for comparision        
     }
     receivedbuffer[rec]='\0';// declare the string end for indentification to avoid the full string check
		 datacompare(receivedbuffer);
		 ledstate=40;// give acknowledgement to the mobile 
		 timerr=0;// resetting the voltage and current transmission counter

}
void datacompare(const char *compbuffer)// this function is not fully implemented here we going to handle every datas
{	
	   f=0;
	while(compbuffer[f]!='\0')
	{
		if(((compbuffer[f]|0x20)=='h')&&((compbuffer[f+1]|0x20)=='v')&&((compbuffer[f+2]|0x20)=='-'))//hv- or lv- or ont- we going to handle like this
		{
			 hvc=0;
			 for(savex=0;savex<3;savex++)
			 hvc =hvc*10+((compbuffer[f+3+savex]-48)*1);
			
			 setmemsave=1;
		}
		if(((compbuffer[f]|0x20)=='l')&&((compbuffer[f+1]|0x20)=='v')&&((compbuffer[f+2]|0x20)=='-'))
		{
			 lvc=0;
			 for(savex=0;savex<3;savex++)
			 lvc = lvc*10+((compbuffer[f+3+savex]-48)*1);
			 setmemsave=1;
		}
		if(((compbuffer[f]|0x20)=='f')&&((compbuffer[f+1]|0x20)=='c')&&((compbuffer[f+2]|0x20)=='t'))
		{
			/* ovldisp=0;
			 for(savex=0;savex<3;savex++)
			 ovldisp=ovldisp*10+((compbuffer[f+4+savex]-48)*1);*/
			
			 factoryreset=1;
		}
		if(((compbuffer[f]|0x20)=='o')&&((compbuffer[f+1]|0x20)=='n')&&((compbuffer[f+2]|0x20)=='t')&&((compbuffer[f+3]|0x20)=='-'))
		{
			     ont=0;
			   for(savex=0;savex<3;savex++)
			   ont=ont*10+((compbuffer[f+4+savex]-48)*1);
			
			    setmemsave=1;
		}
	if(((compbuffer[f]|0x20)=='t')&&((compbuffer[f+1]|0x20)=='r')&&((compbuffer[f+2]|0x20)=='p')&&((compbuffer[f+3]|0x20)=='-'))
		{
			if(((compbuffer[f+4]|0x20)=='i')&&((compbuffer[f+5]|0x20)=='m')&&((compbuffer[f+6]|0x20)=='e'))
				          delaytrip=1;
				
			if(((compbuffer[f+4]|0x20)=='d')&&((compbuffer[f+5]|0x20)=='l')&&((compbuffer[f+6]|0x20)=='y'))
				          delaytrip=0;
					
			        setmemsave=1;
		}
		if(((compbuffer[f]|0x20)=='d')&&((compbuffer[f+1]|0x20)=='e')&&((compbuffer[f+2]|0x20)=='v')&&((compbuffer[f+3]|0x20)=='-'))
		{
			if(((compbuffer[f+4]|0x20)=='o')&&((compbuffer[f+5]|0x20)=='n'))
			{
				deviceon=1;
				deviceoff=0;
			}
				
				if(((compbuffer[f+4]|0x20)=='o')&&((compbuffer[f+5]|0x20)=='f')&&((compbuffer[f+6]|0x20)=='f'))
				{
					deviceoff=1;
					deviceon=0;
				}
				if(((compbuffer[f+4]|0x20)=='r')&&((compbuffer[f+5]|0x20)=='s')&&((compbuffer[f+6]|0x20)=='t'))
				{
				}	
		}
		f++;
	}
memset(receivedbuffer,0,sizeof(receivedbuffer));//clear the received buffer , all buffers are immediatly clearing the buffer after comparing the datas
}
void formatinttoString(char *bufferr,uint16_t convertiondigit,char totaldigits) //integer to string conversion function for transmission
{
	 pdata char digitx=0;
		bufferr[totaldigits] = '\0';
    for (digitx=totaldigits-1;digitx>=0;digitx--) 
    {
        bufferr[digitx]=(convertiondigit%10)+'0';
        convertiondigit/=10;
    }
}
typedef struct {
    char state;
    char *command;
    char next_state;
}statusCommand;

xdata statusCommand commands[]={
    {10, "AT+BLESTATE?\r\n", 11},//conneted or not
    {20, "AT+BLEDISCON\r\n", 21},//disconnect
		{40, "AT+BLESEND=10,I_Received\r\n",41}///acknowledge command
};
void statushandler(void)
{
	for(stat=0;stat<sizeof(commands)/sizeof(commands[0]);stat++){
        if (ledstate == commands[stat].state) { //check the state is expected state
            uartsendstring(commands[stat].command);//sending the expected command								
            ledstate = commands[stat].next_state;//update the state
            return;
        }
    }
}
void newmessagecheck(void)
{
	if(newmessagearrived==1)// when we receive the new message this flag set
		{
			newmessagearrived=0;
		if(((searchstring(tempbuffer,"ready"))||(searchstring(tempbuffer,"OK")))||(bluetootherror==1))// we use this function for we get expected message or not
			{
				if(turnoff==1)
					turnoff=0;
				if(responsecheck==1)
					responsecheck=0;
				if(echooff==1)
				  	echooff=0;
				//if((moduleready==1)||(nameready==1))
				if(moduleready==1)
				{ moduleready=0;
					disconnected=1;
					//trunoffled
					ledstate=10;
				}
				if(resetdone==1)
				{
					resetdone=0;
					bluetootherror=0;
					statusled=0;
				}
				if(ledstate==21)
				   ledstate=5;
				
			}
			if((ledstate==11)&&(searchstring(tempbuffer,"STATE:0"))&&(connected==1))// check the module properly connected or not
			{
				connected=0;
        disconnected=1;
			  //ledstate=20;
			  tempbuffer[0]='\0';
				timerr=0;
			  m=0;
			}
			if((ledstate==11)&&(searchstring(tempbuffer,"STATE:1"))&&(disconnected==1))// if we receive disconnet request we disconnet the module
			{
				connected=0;
				disconnected=1;
				ledstate=20;
				tempbuffer[0]='\0';
				timerr=0;
				m=0;
			}
			if((disconnected==1)&&(searchstring(tempbuffer,"EVENT:BLE_CONNECTED")))//connected flag
			{
				connected=1;
				disconnected=0;
				//ledstate=1; turnon led
				ledstate=10;
				tempbuffer[0]='\0';
				 m=0;
				timerr=0;
			}
			if((connected==1))
			{
		if(searchstring(tempbuffer,"EVENT:BLE_DISCONNECT"))// disconneted flag
    {    connected=0;
         disconnected=1;
			   //ledstate=5;turnoff led
			ledstate=10;//12
			   tempbuffer[0]='\0';
			    m=0;
			   timerr=0;
    }
		else
		{
    extractthedata(tempbuffer);//here taking the message only ,dont  take report like ready ,ok,
		}
		 }
		 memset(tempbuffer,0,sizeof(tempbuffer));//clearing the buffer
			m=0;
		}
	if(((turnoff==1||(responsecheck==1)||(echooff==1)||(moduleready==1))&&(connected==0)))//incase we unable to get the correct response after half sec again we send the command
	{
			countern++;
			if(countern>=30)
			{
				countern=0;
				if((turnoff==1))
				uartsendstring("AT+BLEMODE=9\r\n");
				if(responsecheck==1)
				uartsendstring("AT\r\n");	
				if(echooff==1)
				uartsendstring("ATE0\r\n");	
				if(moduleready==1)
				uartsendstring("AT+BLEMODE=0\r\n");		
			}
		}
}
void ledblink(void)
{
for(e=0;e<5;e++)
{
	on=0;
	if(e==0)
	{
		P1M1=0x3b;
		P1M2=0xC4;
		P1&=~(0x3f);//red
	}
	if(e==1)
	{
		P1M1=0x2F;
		P1M2=0xd0;
		P1&=~(0x3f);//blue
	}
	if(e==2)
	{
		P1M1=0x1F;
		P1M2=0xe0;
		P1&=~(0x3f);//green 
	}
	if(e==3)
	{
		P1M1=0x2b;
		P1M2=0xd4;
	  P1&=~(0x3f);//violet
	}
	if(e==4)
	{
		P1M1=0x3e;
		P1M2=0xC1;
		P1&=~(0x3f);//yellow
	}
for(q=0;q<250;q++)
{	
delay(30);
}
}
on=1;
}
void displaybuffering(void)
{
for(e=0;e<7;e++)
{
for(q=0;q<200;q++)
{	
for(j=0;j<3;j++)
	{
	on=1;
	P1&=~(0x3f);
	P1M1=segmentData[e][j].Port&0x3F;
  P1M2=segmentData2(segmentData[e][j].Port)|0xC0;
  P1|=(segmentData[e][j].Pin&0x3f);
delay(1);
muxdisable();
delay(10);
	}
}
}
}
unsigned char Read_APROM_BYTE(unsigned int code *u16_addr)
{
    //UINT8 rdata;
    rdata = *u16_addr >> 8;
    return rdata;
}
void Write_DATAFLASH_BYTE(unsigned int u16EPAddr, unsigned char u8EPData)
{
    unsigned char looptmp = 0;
    unsigned int u16_addrl_r;
    unsigned int RAMtmp;

    //Check page start address
    u16_addrl_r = (u16EPAddr / 128) * 128;

    //Save APROM data to XRAM0
    for (looptmp = 0; looptmp < 0x80; looptmp++)
    {
        RAMtmp = Read_APROM_BYTE((unsigned int code *)(u16_addrl_r + looptmp));
        page_buffer[looptmp] = RAMtmp;
    }

    // Modify customer data in XRAM
    page_buffer[u16EPAddr & 0x7f] = u8EPData;

    //Erase APROM DATAFLASH page
    IAPAL = u16_addrl_r & 0xff;
    IAPAH = (u16_addrl_r >> 8) & 0xff;
    IAPFD = 0xFF;
    set_CHPCON_IAPEN;
    set_IAPUEN_APUEN;
    IAPCN = 0x22;
    set_IAPTRG_IAPGO;

    //Save changed RAM data to APROM DATAFLASH
    set_CHPCON_IAPEN;
    set_IAPUEN_APUEN;
    IAPCN = 0x21;

    for (looptmp = 0; looptmp < 0x80; looptmp++)
    {
        IAPAL = (u16_addrl_r & 0xff) + looptmp;
        IAPAH = (u16_addrl_r >> 8) & 0xff;
        IAPFD = page_buffer[looptmp];
        set_IAPTRG_IAPGO;
    }

    clr_IAPUEN_APUEN;
    clr_CHPCON_IAPEN;
}






      

