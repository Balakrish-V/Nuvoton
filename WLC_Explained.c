//BASE code - WLC
//no lora or gsm
//no uart,modbus

#include "numicro_8051.h"
#include <math.h> 
#define   CARRY CY
#define		  buzzer	   P16
sbit	startrly=P3^6;
//sbit relay=P2^1;
#define 	chip  	P02
#define 	enab 		P03
#define   ddata   P04
#define dclock    P05
#define alpha 0.95
sbit toplevel=P2^0;
//sbit botlevel=P3^7;
sbit stoprly=P3^7;
sbit crmrly=P1^2;
sbit relay2=P3^5;
//
/*#define   ddata   P00
#define   dclock  P01
#define 	chip  	P02
#define 	enab 		P03*/
//sbit dclock=P0^1;
////////////////////////////////////////////////////////////////////////////////
/*LoRa macros*/
/*#define LORARESET    P30
#define BUFFER_SIZE   100
#define QUIT           0
#define TARGET_ADD     1
#define TXLOCAL_ADD    2
#define TX_CONFIG      3
#define RXLOCAL_ADD    4
#define RX_CONFIG      5 
#define NEWID          6 
#define ACKID          7 
#define SUCCESS        1
#define ERROR          0*/

sbit edat=P1^7; 
sbit eclk=P3^4;
sbit P23=P2^3;
sbit P24=P2^4;
#define YPHASEMUXDIS P13=0;P24=1;P23=1;
#define BPHASEMUXDIS P13=0;P24=1;P23=0;
#define RPHASEMUXDIS P13=1;P24=1;P23=0;
#define YCTMUXDIS P13=1;P24=0;P23=0;
#define RCTMUXDIS P13=1;P24=1;P23=0;
#define BCTMUXDIS P13=1;P24=1;P23=1;
#define CTMUXDIS  P13=0;P24=0;P23=0;
#define FLTMUXDIS  P13=0;P24=0;P23=1;


//#define    ENABLE1_ADC_CH10       /*P22_INPUT_MODE;*/SFRS=0;ADCCON0&=0x30;ADCCON0|=0x0A;ADCCON1|=0x31;ADCCON2|=0X0E;//current & volt
//#define    ENABLE1_ADC_CH15       /*P25_INPUT_MODE;*/SFRS=0;ADCCON0&=0x30;ADCCON0|=0x0F;ADCCON1|=0x31;ADCCON2|=0X0E;//switch

/*-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
							lookup table routine
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
											  constant  declaration
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
#define dp '.'
#define hi '-'
#define ha '/'
#define ac ':'
#define nu ' ' 
#define ast '*'
#define ca 'A'
#define cb 'B'
#define cc 'C'
#define cd 'D'
#define ce 'E'
#define cf 'F'
#define cg 'G'
#define ch 'H'
#define ci 'I'
#define cj 'J'
#define ck 'K'
#define cl 'L'
#define cm 'M'
#define cn 'N' 
#define co 'O' 
#define cp 'P' 
#define cq 'Q'
#define cr 'R'
#define cs 'S'
#define ct 'T'
#define cu 'U'
#define cv 'V' 
#define cw 'W'
#define cx 'X' 
#define cy 'Y'
#define cz 'Z' 
#define da '0'
#define db '1'
#define dc '2'
#define dd '3'
#define de '5'
#define df '7'
#define dp '.'

	#define sa 'a'
	#define sb 'b'
	#define sc 'c'
	#define sd 'd'
	#define se 'e'
	#define sf 'f'
	#define sg 'g'
	#define sh 'h'
	#define si 'i'
	#define sj 'j'
	#define sk 'k'
	#define sl 'l'
	#define sm 'm'
	#define sn 'n' 
	#define so 'o'
	#define sp 'p' 
	#define sq 'q'
	#define sr 'r'
	#define ss 's'
	#define st 't'
	#define su 'u'
	#define sv 'v' 
	#define sw 'w'
	#define sx 'x' 
	#define sy 'y'
	#define sz 'z'

/*-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
							lcd initialisation values routine
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
//bdata unsigned char RAMflags _at_ 0x20;
bdata unsigned char RAMflags _at_ 0x21;
sbit menukey = RAMflags^0;
sbit inckey= RAMflags^1;
sbit deckey = RAMflags^2;
sbit sidekey= RAMflags^3;
sbit memsave = RAMflags^4;
sbit timedisplay = RAMflags^5;
sbit restart = RAMflags^6;
sbit start2 = RAMflags^7;
//menukey=0,inckey=0,deckey=0,sidekey=0,memsave =0,timedisplay=0,restart=0,start2=0
bdata unsigned char RAMflags1 _at_ 0x22;
sbit  voltlow = RAMflags1^0;
sbit volthigh = RAMflags1^1;
sbit ovld = RAMflags1^2;
sbit errordisplay = RAMflags1^3;
sbit phaseunbalance = RAMflags1^4;
sbit onttimewrite = RAMflags1^5;
sbit timeseg = RAMflags1^6;
sbit refresh = RAMflags1^7;
//volatile bit voltlow=0,volthigh=0,ovld=0,errordisplay=0,phaseunbalance=0,onttimewrite=0,timeseg=0,refresh=0;;

bdata unsigned char RAMflags2 _at_ 0x23;
sbit  rtcedit = RAMflags2^0;
sbit manualoff = RAMflags2^1;
sbit  phasereversed= RAMflags2^2;
sbit autosetclrshow = RAMflags2^3;
sbit autosetdone = RAMflags2^4;
sbit smode = RAMflags2^5;
sbit switchset1= RAMflags2^6;
//sbit  autosetdone= RAMflags2^7;

//volatile bit rtcedit=0,manualoff=0,phasereversed=0,autosetclrshow=0,autosetdone=0,smode=0,switchset1=0,autosetdone=0;
bdata unsigned char RAMflags3 _at_ 0x24;
sbit  turnoff = RAMflags3^0;
sbit drywrite = RAMflags3^1;
sbit  pumpsensed= RAMflags3^2;
sbit  vshow = RAMflags3^3;
sbit  displayrefresh= RAMflags3^4;
sbit  switchset2= RAMflags3^5;
sbit switchset3= RAMflags3^6;
sbit  switchset4= RAMflags3^7;
//volatile bit smode=0,switchset1=0,pumpsensed=0,vshow=0,displayrefresh=0,switchset2=0,switchset3=0,switchset4=0;;
bdata unsigned char RAMflags4 _at_ 0x25;
sbit   switchset6= RAMflags4^0;
sbit  switchset5= RAMflags4^1;
sbit  rtcstart= RAMflags4^2;
sbit  modechangedisp= RAMflags4^3;
sbit  drtstore= RAMflags4^4;
sbit  errorprint= RAMflags4^5;
sbit checkflag= RAMflags4^6;
sbit  write= RAMflags4^7;
//volatile bit switchset6=0,switchset5=0,rtcstart=0,modechangedisp=0,drtstore=0,errorprint=0,checkflag=0,write=0;

bdata unsigned char RAMflags5 _at_ 0x26;
sbit  rphfail= RAMflags5^0;
sbit yphfail = RAMflags5^1;
sbit bphfail = RAMflags5^2;
sbit  switchon= RAMflags5^3;
sbit  oftimestarterror= RAMflags5^4;
sbit  oftimererror= RAMflags5^5;
sbit ontimererror= RAMflags5^6;
sbit  showdisp= RAMflags5^7;
//volatile bit rphfail=0,yphfail=0,bphfail=0,switchon=0,oftimestarterror=0,oftimererror=0,ontimererror=0,showdisp=0;

bdata unsigned char RAMflags6 _at_ 0x27;
sbit   drterror= RAMflags6^0;
sbit  nopump= RAMflags6^1;
sbit  reset1= RAMflags6^2;
sbit  relayoff= RAMflags6^3;
sbit  dry= RAMflags6^4;
sbit  phasefail= RAMflags6^5;
sbit start1= RAMflags6^6;
sbit ondelayok= RAMflags6^7;
//volatile bit drterror=0,drydisp=0,reset1=0,relayoff=0,dry=0,phasefail=0,start1=0,ondelayok=0;
bdata unsigned char RAMflags7 _at_ 0x28;
sbit   phasewrite= RAMflags7^0;
sbit  adtransfer= RAMflags7^1;
sbit  pon= RAMflags7^2;
sbit wlctop1 = RAMflags7^3;
sbit  pumpon= RAMflags7^4;
sbit  startcrmoff= RAMflags7^5;
sbit dscroll= RAMflags7^6;
sbit errorlog= RAMflags7^7;

//volatile bit	phasewrite=1,adtransfer=0,pon=0,wlctop1=1,pumpon=0,startcrmoff=0,dscroll=0,errorlog=0;

xdata const char init2[]={0x38,0xa0,0x38,0x0a,0x08,0x0a,0x01,0x0a,0x06,0x0a,0x0f,0x0a,0x80,0x0a,0x01,0x0a};
//const char init2[]={0x38,0x0a,0x38,0x01,0x08,0x01,0x01,0x05,0x06,0x01,0x0f,0x01,0x80,0x01,0x01,0x00};

xdata  const char fline1[]={nu,nu,nu,nu,cs,se,st,nu,sm,so,sd,se,nu,nu,nu,nu};
xdata const char mod[]={cm,sa,sn,su,sa,sl,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu};
xdata const char mod1[]={cc,sy,sc,sl,si,sc,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu};
xdata const char mod2[]={nu,ca,su,st,so,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu};
xdata const char mod3[]={nu,cr,ct,cc,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu};
xdata const char pumpt[]={nu,nu,nu,cp,su,sm,sp,nu,ct,sy,sp,se,nu,nu,nu,nu};
xdata const char pump1[]={nu,co,sp,se,sn,sw,se,sl,sl,'/',cm,so,sn,so,nu,nu};
xdata const char pump2[]={nu,nu,nu,nu,cb,so,sr,se,sw,se,sl,sl,nu,nu,nu,nu};
xdata const char ONT[]={nu,nu,co,sn,nu,ct,si,sm,se,sr,ac,nu,nu,nu,nu,nu};
xdata const char OFT[]={nu,co,sf,sf,nu,ct,si,sm,se,sr,ac,nu,nu,nu,nu,nu};
xdata const char drtd[]={nu,nu,nu,nu,nu,cd,cr,ct,ac,nu,nu,nu,nu,nu,nu,nu};
xdata const char t1[]={nu,nu,ct,nu,hi,co,cn,ac,nu,nu,ac,nu,nu,nu,nu,nu};
xdata const char t1O[]={nu,nu,ct,nu,hi,co,cf,ac,nu,nu,ac,nu,nu,nu,nu,nu};
//xdata  const char t2[]={nu,nu,ct,'2',hi,co,cn,ac,nu,nu,ac,nu,nu,nu,nu,nu};
//xdata const char t2O[]={nu,nu,ct,'2',hi,co,cf,ac,nu,nu,ac,nu,nu,nu,nu,nu};
//xdata  const char t31[]={nu,nu,ct,'3',hi,co,cn,ac,nu,nu,ac,nu,nu,nu,nu,nu};
//xdata const char t3O[]={nu,nu,ct,'3',hi,co,cf,ac,nu,nu,ac,nu,nu,nu,nu,nu};
//xdata  const char t4[]={nu,nu,ct,'4',hi,co,cn,ac,nu,nu,ac,nu,nu,nu,nu,nu};
//xdata const char t4O[]={nu,nu,ct,'4',hi,co,cf,ac,nu,nu,ac,nu,nu,nu,nu,nu};
xdata const char vh[]={ch,si,sg,sh,nu,cv,so,sl,st,ac,nu,nu,nu,nu,cv,nu};
xdata const char vl[]={nu,cl,so,sw,nu,cv,so,sl,st,ac,nu,nu,nu,nu,cv,nu};
xdata const char lo[]={co,sv,se,sr,sl,so,sa,sd,ac,nu,nu,'.',nu,nu,ca,nu};
xdata const char dr[]={nu,nu,cd,sr,sy,sr,su,sn,ac,nu,nu,'.',nu,nu,ca,nu};
xdata const char ti[]={nu,nu,nu,ct,si,sm,se,ac,nu,nu,ac,nu,nu,nu,nu,nu};
xdata const char dm[]={nu,nu,cd,sa,st,se,':',nu,nu,ha,nu,nu,ha,nu,nu,nu};
xdata const char rd[]={cr,hi,cd,sa,st,se,':',nu,nu,ha,nu,nu,ha,nu,nu,nu};
xdata const char vin[]={cv,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu};
xdata const char pt[]={nu,nu,ac,nu,nu,nu,nu,cp,su,sm,sp,ac,nu,nu,nu,nu};
xdata const char va[]={nu,nu,nu,nu,cv,nu,nu,cp,su,sm,sp,ac,nu,nu,nu,nu};
xdata const char ai[]={ci,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu};
xdata const char am[]={nu,nu,nu,nu,ca,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu};
xdata const char tm[]={ct,'-',nu,nu,ac,nu,nu,nu,cm,so,sd,ac,nu,nu,nu,nu};
xdata const char model[]={cm,so,sd,se,sl,nu,cs,se,sl,se,sc,st,si,so,sn,nu};
xdata const char siph[]={nu,nu,nu,cp,ch,nu,cs,se,sl,se,sc,st,se,sd,nu,nu};
xdata const char proce[]={nu,nu,cp,sr,so,sc,se,ss,ss,si,sn,sg,dp,dp,dp,nu};
xdata const char day[]={nu,nu,cd,sa,sy,nu,cs,sk,si,sp,ac,nu,nu,nu,nu,nu};
xdata const char scr[]={nu,cs,sc,sr,so,sl,sl,ac,nu,nu,nu,nu,nu,nu,nu,nu};
xdata const char scra[]={nu,cs,sc,sr,so,sl,sl,ac,ca,su,st,so,nu,nu,nu,nu};
xdata const char scrm[]={nu,cs,sc,sr,so,sl,sl,ac,cm,sa,sn,su,sa,sl,nu,nu};
xdata const char errorhv[]={nu,nu,hi,ch,si,hi,cv,so,sl,st,ac,nu,nu,nu,cv,nu};
xdata const char errorlv[]={nu,nu,hi,cl,so,hi,cv,so,sl,st,ac,nu,nu,nu,cv,nu};
xdata const char errorovl[]={nu,co,sv,se,sr,sl,so,sa,sd,ac,nu,nu,nu,nu,ca,nu};
xdata const char errordry[]={nu,nu,cd,sr,sy,sr,su,sn,ac,nu,nu,nu,nu,nu,ca,nu};
xdata const char  errorpf[]={nu,hi,cp,sh,sa,ss,se,sf,sa,si,sl,ac,nu,nu,nu,cv};
xdata const char errorph[]={nu,cp,sh,sa,ss,se,nu,cu,sn,sb,sa,sl,sa,sn,sc,se};
xdata const char errorpr[]={nu,cp,sh,sa,ss,se,' ',cr,se,sv,se,sr,ss,se,sd,nu};
xdata const char ron[]={co,sn,nu,nu};
xdata const char rof[]={co,sf,sf,nu}; xdata const char ond[]={co,cn,ct,nu,ac};
xdata const char ofd[]={co,cf,ct,nu,ac};xdata const char sc1[]={ct,nu,co,cn};
xdata const char sc2[]={ct,nu,co,cf};//xdata const char sc3[]={ct,'2',co,cn};
//xdata const char sc4[]={ct,'2',co,cf};xdata const char sc5[]={ct,'3',co,cn};
//xdata const char sc6[]={ct,'3',co,cf};xdata const char sc7[]={ct,'4',co,cn};
//xdata const char sc8[]={ct,'4',co,cf};xdata const char mof[]={cm,sa,sn,'-',co,sf,sf};
xdata const char autos[]={nu,nu,nu,nu,nu,ca,su,st,so,ss,se,st,nu,nu,nu,nu};
xdata const char autoc[]={nu,nu,nu,nu,ca,su,st,so,sc,sl,se,sa,sr,nu,nu,nu};
xdata const char autocom[]={nu,nu,nu,cc,so,sm,sp,sl,se,st,se,sd,nu,nu,nu,nu};
//xdata const char autofail[]={nu,nu,nu,nu,nu,cf,sa,si,sl,se,sd,nu,nu,nu,nu,nu};
//xdata const char acshowr1[]={ch,cv,'-',nu,nu,nu,nu,co,sv,sl,'-',nu,nu,dp,nu,nu};
//xdata const char acshowr2[]={cl,cv,'-',nu,nu,nu,nu,cd,sr,sy,'-',nu,nu,dp,nu,nu};
xdata const char select[]={nu,nu,nu,nu,cs,se,sl,se,sc,st,se,sd,nu,nu,nu,nu};
xdata const char man[]={nu,nu,cm,sa,sn,su,sa,sl,nu,cm,so,sd,se,nu,nu,nu};
xdata const char cyc[]={nu,nu,cc,sy,sc,sl,si,sc,nu,cm,so,sd,se,nu,nu,nu};
xdata const char aut[]={nu,nu,nu,nu,ca,su,st,so,nu,cm,so,sd,se,nu,nu,nu};
xdata const char rrtc[]={nu,nu,nu,nu,cr,st,sc,nu,cm,so,sd,se,nu,nu,nu,nu};
xdata const char errorlogd[]={ce,'-',nu,nu,'-',nu,nu,'/',nu,nu,nu,nu,nu,ac,nu,nu};
xdata const char drystart[]={cd,cr,ct,nu,ac};
xdata const char erlog1[]={nu,cp,sr,se,ss,ss,nu,ci,sn,sc,hi,ck,se,sy,nu,nu};
xdata const char erlog2[]={nu,cv,si,se,sw,nu,ce,sr,sr,so,sr,sl,so,sg,nu,nu};
xdata const char erlog3[]={nu,'*','*',nu,cn,so,nu,ce,sr,sr,so,sr,nu,'*','*',nu};
xdata const char crlog1[]={nu,cc,sl,se,sa,sr,nu,ce,sr,sr,so,sr,sl,so,sg,nu};
xdata const char crlog2[]={nu,cp,sr,se,ss,ss,nu,cd,si,sg,si,st,hi,sk,se,sy};
xdata const char crlog3[]={ce,sr,sr,so,sr,sl,so,sg,nu,cc,sl,se,sa,sr,se,sd};
xdata const char crlog4[]={nu,nu,cs,su,sc,sc,se,ss,ss,sf,su,sl,sl,sy,nu,nu};
xdata const char  rst[]={nu,nu,cf,sa,su,sl,st,nu,cr,se,ss,se,st,nu,nu,nu};
xdata const char modesel[]={nu,cm,so,sd,se,nu,cs,se,sl,se,sc,st,si,so,sn,nu};
xdata const char modex[]={nu,nu,nu,nu,nu,cm,sa,sn,su,sa,sl,nu,nu,nu,nu,nu};
xdata const char mode1x[]={nu,nu,nu,nu,nu,cc,sy,sc,sl,si,sc,nu,nu,nu,nu,nu};
xdata const char mode2x[]={nu,nu,nu,nu,nu,nu,ca,su,st,so,nu,nu,nu,nu,nu,nu};
xdata const char mode3x[]={nu,nu,nu,nu,nu,nu,cr,ct,cc,nu,nu,nu,nu,nu,nu,nu};
xdata const char twophe[]={nu,'2',cp,ch,nu,cs,se,sl,se,sc,st,ac,cy,se,ss,nu};
xdata const char twophd[]={nu,'2',cp,ch,nu,cs,se,sl,se,sc,st,ac,cn,so,nu,nu};
xdata const char twoset[]={nu,nu,'2',cp,ch,nu,cs,se,st,st,si,sn,sg,ss,nu,nu};
xdata const char capcut[]={cc,sa,sp,hi,cc,su,st,nu,ct,si,sm,se,ac,nu,nu,ss};
xdata const char pn[]={nu,cr,su,sn,sn,si,sn,sg,nu,ci,sn,hi,nu,cp,ch,nu};
xdata const char pset[]={cp,su,sm,sp,nu,cs,se,st,st,si,sn,sg,ac,cy,se,ss};
xdata const char pset1[]={cp,su,sm,sp,nu,cs,se,st,st,si,sn,sg,ac,cn,so,nu};
xdata const char Ondly[]={nu,nu,co,sn,nu,cd,se,sl,sa,sy,ac,nu,nu,nu,ss,nu};
xdata const char wlcad[]={cw,sl,sc,nu,ca,sd,sd,sr,se,ss,ss,ac,nu,nu,nu,nu};   
xdata const char  asetclr[]={nu,cs,st,sa,sr,st,ss,nu,ci,sn,ac,nu,nu,cs,se,sc};
xdata const char  no_pump[]={nu,nu,nu,nu,nu,cn,so,hi,cp,su,sm,sp,nu,nu,nu,nu};
xdata const char  ovlcut[]={co,sv,sl,hi,cc,su,st,st,si,sm,se,':',nu,nu,ss,nu};
xdata const char  drycut[]={cd,sr,sy,hi,cc,su,st,st,si,sm,se,':',nu,nu,nu,ss};
xdata const char  ubvolt[]={cu,sn,sb,sa,sl,nu,cv,so,sl,st,':',nu,nu,nu,cv,nu};
xdata const char  ubcur[]={cu,sn,sb,sa,sl,nu,ca,sm,sp,ss,':',nu,nu,ca,nu,nu};
xdata const char unbc[]={nu,cc,su,sr,sr,se,sn,st,nu,cu,sn,sb,sa,sl,nu,nu};
xdata const char strdle[]={nu,cs,st,sa,sr,hi,cd,se,sl,st,sa,':',cy,se,ss,nu};
xdata const char strdld[]={nu,cs,st,sa,sr,hi,cd,se,sl,st,sa,':',cn,so,nu,nu};
xdata const char strdlt[]={cs,st,sr,hi,cd,sl,st,nu,ct,si,sm,se,':',nu,nu,ss};
xdata  const char sumpemty[]={nu,nu,nu,nu,cs,su,sm,sp,nu,cd,sr,sy,nu,nu,nu,nu};
//xdata const char wlcadtrans[]={nu,nu,cw,cl,cc,nu,cp,sa,si,sr,si,sn,sg,nu,nu,nu};
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
						    variable declaration routine
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

volatile unsigned char xdata page_buffer[128];
unsigned char rdata;
unsigned int counter=0,i=0,result1=0,intref=0,result2=0,vref2=0,bufferx=0,buffer=0,rtctime=0,ontime=0,offtime=0,run2=0;
xdata unsigned char adcome1=0,displayx=0,t3=0,channel=0,k=0,address=0,temp=0,dbuf[7];
char digit1[7],cblink=0;
double resultx1=0;
unsigned long rtcdate=0,bufferr=0,refday=123467;
unsigned long result=0;//,rtcdate=0,bufferr=0
xdata unsigned char ibm=0,reg=0,reg7=0,r3=0,h=0,mpress=0,setpress=0,smenu=0,ontimeh=0,ontimel=0,offtimeh=0,offtimel=0,hvh=0,lvh=0,hvl=0,lvl=0,run2l=0,run2h=0;
xdata unsigned char timeron=0,bufferh=0,bufferl=0,resetdata1=0,resetdata=0,limit=0,mode=0,disdat=0,datachang=0,blink=0,chang=0,cursor=0,spress=0,incpress=0,menupress=0,pumptype=0;
xdata unsigned char rtccredit=0,rtcmin=0,rtchour=0,count=0,eemem=0,buff=0,txbuf=0,datai=0,buf=0,h1on=0,h2on=0,h3on=0,h4on=0,m1on=0,m2on=0,m3on=0,m4on=0,h1off=0,h2off=0,h3off=0,h4off=0,m1off=0,m2off=0,m3off=0,m4off=0,timeok=0,curmin=0,curhour=0,runningmin=0,runninghour=0;
xdata unsigned int scan=0,scanx=0,dispr=0,start=0,run7=0,run3=0,resultt=0,combinevalue=0,runningtime=0,ontimex=0,offtimex=0,presenttime=0,futuretime=0,ontime1set=0,offtime1set=0,ontime2set=0,offtime2set=0,ontime3set=0,offtime3set=0,ontime4set=0,offtime4set=0;//y=0,year1=0,year2=0,month2=0,month1=0,yeardays=0,totalpass2=0,totalpass1=0,totalpass3=0,totalpass4=0,day2=0,day1=0,totaldays=0,
xdata unsigned char phscanx=0,phaseselection=0,setmode=0,phscan=0,limitx=0,start4=0,run3h=0,run3l=0,run8=0,flt=0,wtr=0,wtrmr=0,full=0,empty=0;//rtcdate1=0,rtcmonth=0,rtcyear=0,date=0,mon=0,yr=0,dayon=0,
xdata  signed long int resultx=0;
unsigned long int xdata result5=0,result6=0,store1=0;
xdata unsigned int result4=0,icur2=0;
signed int xdata resultxvolt=0,resultxcurrent=0,resultvolt=0,resultcurrent=0,resultr=0,resulty=0,resultb=0,dispry=0,dispyb=0,dispbr=0,dispry1=0,dispyb1=0,dispbr1=0;
signed long int xdata resultry=0,iwatt=0,icur=0,result3=0,volt=0,cur=0;
xdata unsigned char dryrestart=0,dryrunh=0,dryrunl=0,ovlh=0,ovll=0,sense=0,chkonce=0,autosetcount=0;
xdata unsigned int modechangingsubcount=0,errtime=0,errdate=0,dispcerror=0,errorvalue=0,errorvaluex=0,errorvalue1=0,runningdaymonth=0,drt=0,hv=0,lv=0,ovl=0,msec=0,avgvolt=0,ppdiff=0,ppvolt=0,pppercent=0,rypercent=0,ybpercent=0,brpercent=0,brdiff=0;
xdata unsigned char drth=0,drtl=0,m=0,date=0,mon=0,rtcdate1=0,rtcmonth=0,rtcyear=0,yr=0;
xdata unsigned char disphighh=0,disphighl=0,displowh=0,displowl=0,dryh=0,dryl=0,disperror=0,errt=0,ipress=0,errortime=0,errortime1=0,errtinc=0,errtinc1=0,errorsave1=0,errt1=0,errt2=0,errtview=0;
xdata unsigned char autosetcountsubcount=0,modedisp=0,modedispsubcount=0,modedispcount=0,modechangingcount=0,modechangekey=0,dryrestartcount=0,twodigitdisp=0,hour=0,minute=0,edate=0,emonth=0,errordisplay1=0,errorloginientry=0;
xdata unsigned char x3=0,countlcd=0,autosetdonesubcount=0,autosetclrshowsubcount=0,autosetclrshowcount=0,autosetclrprocesssubcount=0,autosetclrprocessmain=0;
xdata unsigned int dayskip=0,setcurrent=0,setvoltage=0,setcurrent1=0,setvoltage1=0,setvoltagex1=0,setvoltagex2=0,setvoltagex3=0,rphasesetvolt=0,yphasesetvolt=0,bphasesetvolt=0;
xdata unsigned char  runningdate=0,runningmonth=0,runningyear=0,curdate=0,curmonth=0,curyear=0;
xdata unsigned char phasenumber=0,errorvalueh=0,errorvaluel=0,oftimestart=0; 
xdata unsigned char switchoffsubcount=0,switchoffcount=0,sidepress=0,setkey=0,incpress1=0,scrolltype=1;
xdata unsigned int y=0,pwmvalue1=0,pwmvalue1x=0,totaldays=0,yeardays=0,year1=0,day1=0,month1=0,year2=0,day2=0,month2=0,totalpass1=0,totalpass2=0,totalpass3=0,totalpass4=0;
xdata unsigned char resetx=0,ulimit=0,refdate=0,refmon=0,refyear=0,monthday=0,dayon=0,dayskiph=0,dayskipl=0;
xdata unsigned int dryrun=0,resetcount=0,onrtcvalue1=0,offrtcvalue1=0,onrtcvalue2=0,offrtcvalue2=0,onrtcvalue3=0,offrtcvalue3=0,onrtcvalue4=0,offrtcvalue4=0;
xdata unsigned char crmon=0,adctime=0,wtrbot=0,wtrtop=0,wtrmrtop=0,wtrmrbot=0,fulltop=0,fullbot=0,emptytop=0,topfull=0,emptybot=0,wlcbot=0;
xdata unsigned int resultct=0,resultct1=0,j=0,rampsreference=0,yampsreference=0,bampsreference=0,rctval=0,yctval=0,bctval=0,rctval1=0,yctval1=0,bctval1=0,doad=0,rct1=0,yct1=0,bct1=0;
xdata unsigned int disp4=0,ovtime=0,initialovl=0,dtime=0,brvolt=0,ybvolt=0,ryvolt=0,disp2=0,ovlisl=0,ovlptg=0;
xdata unsigned char pumpsensingcount=0,crmoffcount=0,startrlyoffcount=0,stoprlyoffcount=0,pumpoff=0,address_store=0;
xdata unsigned char phrtime=0,phfail=0,twophase=0,testcount=0,asetpress=0,aclrpress=0,c=0,onrt=0,offrt=0,phase=0,nokeysubcount=0,nokeys=0,cttime=0,vfault=0,phasenumber1=0;
xdata unsigned int rphvolt1=0,yphvolt1=0,bphvolt1=0,hv2=0,lv2=0,ovl2=0,dryrun2=0,capcuttime=0,rphvolt=230,yphvolt=230,bphvolt=230;
xdata unsigned int  starrelayoffcount=0,drycut1=0,Nop=0,hvmax=0,hvmin=0,ev=0,timeoutx=0,drt1=0,setreset=0,rvolt=0,yvolt=0,bvolt=0,highvolt=0,lowvolt=0,overload=0,dryx=0,ondelaycheck=0,ondelay=0;
xdata unsigned char dryreadl=0,dryreadh=0,asetkey=0,aclrkey=0,autosetclrprocess=0,amps=0,timesegstart=0,edit=0,switchtime=5,switched=0,relayoffcount=0,relayoncount=0,rphasefailcount=0,yphasefailcount=0,bphasefailcount=0,twophaseselected=0;
 xdata unsigned char sumpdry=0,botlevel=0,starrelayoff=0,starondelay=0,stardelta=0,ubc=0,autoset=0,autoclr=0,autosetclrstart=0,ovlcut1=0,ubvolt1=0,ubcur1=0,unbalancecurrent=0,unbalancevolt=0;
//xdata unsigned char menukey=0,inckey=0,deckey=0,sidekey=0;
xdata  uint8_t d1=0,m1=0,y1=0;
xdata unsigned char deltaoncount=0,deltaon=0;
/*LoRa Variables*/
/*xdata unsigned char receivedData[BUFFER_SIZE] = {0};
xdata unsigned char responseok=0,lora_datareceived=0,timeout=0,timeouterror =0,flag=0,cnt=0,ack_receive_enable=0,newid_ok=0,reception_ok=0,value=0;
xdata unsigned int ack_timeout=0;
xdata char 	newid[10]="NEWID";
xdata char 	ackid[10]="ack";
typedef struct loraTxData_
{
    char *query;
    char *response;
}loraTxData;

xdata loratxqueryflag[6]={0};
xdata char responseerror = 0,errorcount = 0,ATtransmit=0,query_response=0;
xdata loraTxData loraquery[]=
{
    {"+++\r\n","Quit"},
    {"AT+CTXADDRSET=02\r\n","set target address"},
    {"AT+CADDRSET=01\r\n","set local address"},
    {"AT+CTX=868000000,6,0,1,21,1\r\n","freq: 868000000"},
    {"AT+CADDRSET=02\r\n","set local address"},
    {"AT+CRXS=868000000,6,0,1,1\r\n","freq: 868000000"},
		{newid,"OnTxDone"},
};*/
void set_clock_source(void);
void set_clock_division_factor(unsigned char value);
void portinit(void);
void adc_init(void);
void adcal(void);
void adcal1(void);
void delay(char gg);
void init(void);
void rct(void);
void yct(void);
void bct(void);
void adcconv(void);
void lcdinit(void);
void enabler(char t4);
void switchscan(void);
void Write_DATAFLASH_BYTE1(unsigned int u16EPAddr, unsigned char u8EPData);
unsigned char Read_APROM_BYTE(unsigned int code *u16_addr);
void fline(void);
void sline(void);
void valuewrite(void);
void splitl(void);
void valuewritex(void);
void display2(void);
void editmode(void);
void readdata(void);
void datawrite(void);
void digitsel(void);
void incrout(void);
void checkcblink(void);
void combin(void);
void split2(void);
void decrout(void);
void display(void);
void timer_init(void);
void valuewrite2(void);
void tx(void);
void rx(void);
void bstart(void);
void bstop(void);
void bitin(void);
void bitout(void);
void timprob(void);
void rwrbyte1(unsigned char k,unsigned char add);
void rread1(unsigned char jj);
void bcdtodecimal(unsigned char value);
void variablecombine(unsigned char value1,unsigned char value2);
void combine(void);
void timread(void);
void timecompare(void);
void analog(void);
void phrcheck(void);
void unbalcal(void);
void decide(void);
void vcutoff(void);
void errorrepeat(void);
void readerror(void);
void errorview(void);
void errorlogdisplay(void);
void autosetclr(void);
void dataout(void);
void errordisplayprint(void);
void pwm_init(void);
void Pwmduty(unsigned int dutycycle);
void monthfunction(unsigned int month,unsigned int year);
void remainingdays(unsigned int day,unsigned int month,unsigned int year);
void daysbetweenyears(void); 
void passeddays(unsigned int day,unsigned int month,unsigned int year); 
void readreference(void);
void ctscan(void);
void curprob(void);
void drycuttoff(void);
void ovlcuttoff(void);
void curdecide(void);
void pwmfunction(void);
void timesegread(void);
void zc(void);
void logerror(unsigned char errortype,unsigned char cterror,unsigned char skiperror);
void threedigit(void);
void threedigita(void);
void fourdigit(void);
char checkvolt(unsigned int v);
void checkvoltvalues(unsigned int ry1, unsigned int yb1,unsigned int br1);
void checkctvalues(unsigned int rct1, unsigned int yct1,unsigned int bct1);
int compare(int comp1, int comp2);
void unblanceampscut(void);
void pumpofffunction(void);
void pumponfunction(void);
void twophaseon(void);
void stardeltapumpon(void);
void stardeltapumpoff(void);
void floatscan(void);
/*****************************************UART Functions**************************/
/*void uart0_init();
void uart_transmit(const char *dt);
void UART_Send(unsigned char c);
void reset_recindex(void);*/
/*****************************LoRa Functions*******************************************************/
/*void lorareset(); 
void lora_resposecheck();
int stringcontains(const char *haystack, const char *needle);
int loraATtransmit(char queryresponseindex);
int loraTxdata(char queryindex);
int loraTxinit();
int loraRxinit();
void custom_sprintf_ctxaddr(char *buffer, int addr, const char *prefix);
void ack_timerout();
void id_extractor(char num);*/
void main()
{
nopump=0;
turnoff=1;
wlctop1=1;
phasewrite=1; //initilly it will after model selcection complete it goes to zero
init();       //port and clock initialisation
startrly=0;
stoprly=1;    //to avoid manual pump on and relay hold
crmrly=0;
CTMUXDIS;
adc_init();   //adc configuration
pwm_init();   //pwm init
readdata();   //eeprom read for default data
lcdinit();    // lcd init
floatscan();  // sump float initila scan
timer_init(); //timer interrupt init and turn on
//lorareset();
//uart0_init();
//loraRxinit();	
pwmvalue1x=800;
Pwmduty(800);  //give pwm for initial tuch led glow
timeseg=1;     //read RTC time
//delay(200);
///////////////////////////////This portion for 1ph /3 ph model selection
for(i=0;i<500;i++)
{
switchscan();
//if((vref2<990&&vref2>930))
if(vref2<530&&vref2>460)//menu key's vref it checks switch is pressed or not
{
	phaseselection=1;
}
}
	while(phaseselection==1)//one it enter in this loop, if you want to exit in this loop you have to turn off and  on the device 
	{
		Pwmduty(800);
		fline();
		for(t3=0;t3<16;t3++)
		{
			temp=model[t3];
			enabler(temp);
			delay(1);
		}
		for(phscanx=0;phscanx<12;phscanx++)
		{
		  	adcome1++;
	    	if(adcome1>3)
		  		adcome1=1;
		  	analog();
	  }
	 //dispr=dispry1/2;
	 //if(((dispyb1<(dispr+20))&&(dispyb1>(dispr-20)))&&((dispbr1<(dispr+20))&&(dispbr1>(dispr-20))))
		//if((dispyb1<(dispr+20))&&(dispyb1>(dispr-20)))
	 if(dispyb1<((dispry1*60)/100))
	 {
		phase=1;
	 }
     else
     {
    	phase=3;
     }
	 if(phasewrite==1)
	 {
  		phasewrite=0;
 		IE&=~(1<<7);
  		Write_DATAFLASH_BYTE1(61045,phase);
	//
		 errt=0;
		 errtview=0;
		 errorsave1=0;
		 IE&=~(1<<7);
		 Write_DATAFLASH_BYTE1(62000,errt);
	   	 Write_DATAFLASH_BYTE1(62001,errtview);
         Write_DATAFLASH_BYTE1(62002,errorsave1);
	//
		IE|=(1<<7);
		sline();
		//processing//
		for(t3=0;t3<16;t3++)
		{
			temp=proce[t3];
			enabler(temp);
			delay(1);
		}
		for(t3=0;t3<100;t3++)
		{
			delay(100);
		}
		/////////////
		sline();
		for(t3=0;t3<16;t3++)
		{
			temp=siph[t3];
			enabler(temp);
			delay(1);
		}
		chip=0;
		enabler(0xc2);
		chip=1;
		delay(3);
		enabler(phase+0x30);
		delay(3);
		chip=0;
		enabler(0x0c);
		delay(0x0a);
	 }
  }
counter=0;
displayx=1;
refresh=1;

resultx=0;
result1=0;
readreference();//reading CT's Reference 
	if(phase==3&&twophase==1)//if two phase mode is enabled it will come to this loop
	{
		for(phscanx=0;phscanx<12;phscanx++)
		{
		  adcome1++;
	    if(adcome1>3)
		  adcome1=1;
		  ctscan();
	  }
		if(rphvolt1>100&&yphvolt1>100&&bphvolt1<50)//if B-phase is fail the device enter in the two phase mode
		{
			twophaseselected=1;
			switched=1;
		}
	}
	if(dryrestart==1)// this is for dry restart timer value restore loop;
	{
		dry=1;
		restart=1;
		errordisplay=1;
		//drydisp=1;
		displayx=1;
		run3=drt1;
	    Read_APROM_BYTE(61062);
  		dryreadh=rdata;
		Read_APROM_BYTE(61063);
		dryreadl=rdata;
		errorvalue1=((dryreadh<<8)|dryreadl);
		if(errorvalue1>999)
			errorvalue1=0;
		stoprly=1;
	}
	fline();
	while(1)
	{
	while(smode==0&&errorlog==0)//this loop is normal runmode
    {
	if(pon==0)//to avoid manually turn on pump, in this code device only turn on and off the pump
	{
		stoprly=1;
	}
    switchscan();//this fuction for switch pressing detection function	
    adcome1++;//each adcome1++ we read each phase and amps
	if(adcome1>6)
		adcome1=1;	
	analog();//voltage calculation fuction
    ctscan();//current calculation function		
    switchscan();	//this fuction for switch pressing detection function	
    //we are taking avrage to avoid voltage and current oscillation//
	adctime++;
	
	ryvolt=ryvolt+dispry1;
	ybvolt=ybvolt+dispyb1;
	brvolt=brvolt+dispbr1;
		
	rct1=rct1+rctval1;
	yct1=yct1+yctval1;
	bct1=bct1+bctval1;
		
	rvolt=rvolt+rphvolt1;
	yvolt=yvolt+yphvolt1;
	bvolt=bvolt+bphvolt1;
		
	if(adctime>=10)
	{
		dispry=ryvolt/10;
		dispyb=ybvolt/10;
		dispbr=brvolt/10;
		
		rctval=rct1/10;
		yctval=yct1/10;
		bctval=bct1/10;
		
		if(rctval<10)
			rctval=0;
		if(yctval<10)
			yctval=0;
		if(bctval<10)
			bctval=0;
		
		rphvolt=rvolt/10;
		yphvolt=yvolt/10;
		bphvolt=bvolt/10;
			
		brvolt=0;
		ybvolt=0;
		ryvolt=0;
		rct1=0;
		yct1=0;
		bct1=0;
		rvolt=0;
		yvolt=0;
		bvolt=0;
		adctime=0;
		/////////////////////////////////////////
/*avgvolt=dispry1+dispyb1+dispbr1;
avgvolt=avgvolt/3;
ppdiff=0;
if(voltlow==0&&volthigh==0&&phasereversed==0&&phasefail==0)
{
ppvolt=dispry1;
unbalcal();
rypercent=pppercent;

ppvolt=dispyb1;
unbalcal();
ybpercent=pppercent;

ppvolt=dispbr1;
unbalcal();
brdiff=ppdiff;
brpercent=pppercent;
}*/
	if(sense<3)// to avoid initial error ditection
		sense++;
	}

	if(rctval>=10)//no pump flag clear if pump takes current
		Nop=0;

	if(sense>0&&switched==0)
	    phrcheck();//phase reversal check
			 
	if(sense>1&&switched==0)
    {
            decide();							// this funtion is decide all voltage related errors
			curdecide();						// this function is decide all current related errors
			checkctvalues(rctval,yctval,bctval);// this is for unbalance current detection
			vcutoff();							// vottage error cut-off function
            drycuttoff();						// dry run error cut-off function
            ovlcuttoff();						// overload error cut-off function
			unblanceampscut();					// unbalance currnt cut-off function
	}
		 timesegread();  // rtc time read function
		 floatscan();	 // sump float scanning
		 //
		 //
      //////////////////////////////if any error happens then only it will execute////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if((((volthigh==1||voltlow==1||phasereversed==1||phaseunbalance==1||phasefail==1)&&(start2==1))||((dry==1||ovld==1||nopump==1||unbalancecurrent==1||sumpdry==1)&&restart==1)))
    {
		errordisplay=1;
		if((errordisplay==1&&dryrestart==0))
		{
			displayx=1;
			//counter=0;
		}
		if(displayrefresh==0)
		{
			refresh=1;
			displayrefresh=1;
		}
    }
	
	if(refresh==1)
    {
		refresh=0;
        display();// this function contains only fixed strings
	}
		
	if(resetx==0&&modechangedisp==0)
        valuewritex();// this function contains not fixed strings
		
	pwmfunction();//this function handles led related pwm generation based on error and normal condition

//////////////////////////////////////////////////only for rtc mode//////////////////////////////////////////////
	if(mode==4&&switched==0&&autosetclrprocess==0&&sense>2)
	{
		daysbetweenyears();// day skip function

    	if (start2==0&&restart==0&&volthigh==0&&voltlow==0&&modedisp==0&&modechangedisp==0)
		{
        	for (c=1;c<=4;c++) 
			{
				ontimex=0,offtimex=0;
				switch(c)
				{
                case 1:
                    ontimex = ontime1set;
                    offtimex = offtime1set;
                    break;
                case 2:
                    ontimex = ontime2set;
                    offtimex = offtime2set;
                    break;
                case 3:
                    ontimex = ontime3set;
                    offtimex = offtime3set;
                    break;
                case 4:
                    ontimex = ontime4set;
                    offtimex = offtime4set;
                    break;
               }

            	if (ontimex>0&&offtimex>0&&dayon==0)
				{
                presenttime = ontimex;
                futuretime = offtimex;
                timeok=0;
                timecompare();// input time and rtc time comparision function

                 onrt=1+(c-1)*2;
                 offrt=onrt+1;

                if (timeok==1&&rctval1<10&&dayon==0)//time based on function
				{
                    timeron=onrt;
                    if (rtcstart==0) 
					{
								if(wlctop1==0&&wlcbot==1)//float state flag
								{
                        				reset1=1;
										pon=1;//pump on trigger flag
								}
                        rtcstart=1;
                        onttimewrite=1;// timer slot eeprom write flag
                    }
                  }
                if (timeok==0&&timeron==onrt&&dayon==0) //time based off function
				{
                    rtcstart=0;
                    timeron=offrt;
									onttimewrite=1;// timer slot eeprom write flag
                    if (rctval1>=10) {
                        relayoff=1;//pump off trigger Flag
                    }
                }
            }
        }
    }
}
    ///////////////////////////////auto & cyclic mode pump on function////////////////////////////////////////////////////////////////////
		
		if((rctval1<10)&&(pon==0)&&(switched==0)&&(autosetclrprocess==0)&&(sense>2)&&(volthigh==0)&&(voltlow==0)&&(restart==0)&&(start2==0)&&(phasereversed==0)&&(modedisp==0)&&(modechangedisp==0)&&(wlctop1==0)&&(wlcbot==1))
		{
			if((mode==2&&manualoff==0&&ontime>=1&&oftimestart==0)||(mode==3&&wlctop1==0)) 
			{
				//start4++;
		// if(start4>25)//5sec
		//{
			reset1=1;
			//start4=0;
				pon=1;
		// }
			}
		}

		if(phase==1)// if phase is single it disable star delta mode
		{
			stardelta=0;
		}

		if(stardelta!=1)// if device not in start delta mode it will execute
		{
			pumponfunction();
			pumpofffunction();
			twophaseon();
		}

		if(phase==3&&stardelta==1&&twophase!=1)//if the device in stardelta mode it will execute
		{
			stardeltapumpon();
			stardeltapumpoff();
		}

///////////////////////////////////////Cyclic mode on decrement function////////////////////////////////////////////////////////////////////////	 
	 
if((ontime>=1)&&(rctval1>=10)&&(restart==0)&&(mode==2)&&(oftimestart==0)&&(wlctop1==0))//ontime
{
	    run7++;
	 /*if(run7>=26)
	 {
		if((mode==2)&&(ontime>=1)&&(wlctop1==0))
	      cyclicon=1;
	 }*/
	  if(run7>=179)//158
	  {
		  run7=0;
    	  run8=run8+1;
	  }
	  if(run8==3)
	  {
			run8=0;
	    	run2=run2-1;
			ontimererror=1;
	  }
	  if(run2==0)
	  {
		if(mode==2)
           oftimestart=1;	
	  	relayoff=1;
	  	run2=ontime;
		run3=offtime;
		ontimererror=1;
		oftimererror=1;
		refresh=1;
	  }
}
/////////////////////////////////cyclic mode off timer decrement function////////////////////////////////////////////////////////////
if(((rctval1<10)&&(mode==2)&&(oftimestart==1)&&(restart==0)&&(ontime>=1)&&(dryrestart==0))||((dry==1)&&(rctval1<10)&&(restart==1)&&(dryrestart==1)))//oftime
   {
	 /*if((mode==2)&&(ontime>=1))
	  cyclicon=0;*/
	  run7++;
	  if(run7>=179)//158
	  {
	  run7=0;
    run8=run8+1;
	  }
		if(run8==3)
		{
		run8=0;
	  run3=run3-1;
		if(restart==0)
		oftimererror=1;
		if(restart==1&&dryrestart==1)
		{
			drt1=run3;
			drtstore=1;
		}
		}
	  if(run3==0)
	  {
	  //reset1=1;
	  if(restart==0)
		{
		oftimestart=0;
		oftimestarterror=1;
		}
		if(restart==1)
		{
			dry=0;
			if(sumpdry==0)
			restart=0;
			dryrestart=0;
			errordisplay=0;
			displayx=2;
			counter=0;
			run3=drt;
			drt1=drt;
			sense=0;
			adctime=0;
			start1=0;
			refresh=1;
			drterror=1;
			drtstore=1; 
			//if(stoprly==1)
		//stoprly=0;
		}
		if(restart==0)
	  		run3=offtime;
		if(restart==0)
			oftimererror=1;
		counter=0;
		refresh=1;
		
	  }
	}
	/////////////////////////////////////Fault reset function/////////////////////////////////
	 if(resetx==1&&resetcount==0)
	 {
		 resetx=0;
		 //start2=0;
		if(sumpdry==0)
		 restart=0;
		 //volthigh=0;
		 //voltlow=0;
		 ovld=0;
		 dry=0;
		 //phasefail=0;
		 displayx=1;
		 counter=0;
		 errordisplay=0;
		 refresh=1;
		 start1=0;
		 start=0;
		 unbalancecurrent=0;
		 ubc=0;
		 //rphfail=0;
		 //yphfail=0;
		 //bphfail=0;
		 //phaseunbalance=0;
		 if(dryrestart==1)
		 {
		     dryrestart=0;
			 drt1=drt;
			 drtstore=1;
		 }
		 vfault=0;
		 displayrefresh=0;
		 sense=0;
		// adctime=0;
		 pon=0;
		 drterror=1;
		 nopump=0;
		// stoprly=0;
	 }
	 ///////////////////////////////////display scrolling function////////////////////////////////////////////
//if(((scrolltype==1)&&(autosetclrprocess==0&&nopump==0))||(scrolltype==1&&dryrestart==1&&restart==1&&autosetclrprocess==0))
	if(((autosetclrprocess==0))||(scrolltype==1&&dryrestart==1&&restart==1&&autosetclrprocess==0)) 
		 counter++;
  	
	if(counter>65)
  	{
        refresh=1;
		if(scrolltype==1&&nopump==0)
		   displayx++;
		if(errordisplay==1&&dryrestart==0)
			 displayx=1;
		if(pumpsensed==0&&phase==1&&displayx==3)
			 displayx=1;	
		counter=0;
		if(displayx>3)
		{
			displayx=1;
		}
  	}
  //////////////////////////////////////////////////auto set and clear initial time delay function///////////////////////////////////////////////////////////
  if((asetkey==1||aclrkey==1)&&(autoset==0&&autoclr==0&&autosetclrprocess==0)&&(autosetdone==0)&&(autosetclrshow==0))
  {
      autosetcountsubcount++;
      if(autosetcountsubcount>16)
      {
			autosetcountsubcount=0;
			autosetcount++;
			if(switchtime!=0)
				switchtime--;
      }
	  if(autosetcount>4&&switchtime==0)//10
	  {
			autosetcount=0;
			if(asetkey==1&&rctval1>=10)
				autoset=1;
			if(aclrkey==1&&rctval1<10)
				autoclr=1;	
			//autosetdone=1;
			//autosetclrshow=1;
			autosetclrprocess=1;
			if((aclrkey==1&&rctval>=10)||(asetkey==1&&rctval1<10))
			{
				asetkey=0;
				autoset=0;
				aclrkey=0;
				autosetclrprocess=0;
				autoset=0;
				counter=0;
				displayx=1;
			}
            refresh=1;
       }
  }
  ///////////////////////////////////auto clear initiate function/////////////////////////////////////
	if(autosetclrprocess==1&&autosetdone==0&&autosetclrshow==0&&autosetclrstart==0)
	{
		autosetclrprocesssubcount++;
		if(autosetclrprocesssubcount>3)
		{
			autosetclrprocesssubcount=0;
			autosetclrprocessmain++;
		}
		if(autosetclrprocessmain>2)
		{
			autosetclrprocessmain=0;
			autosetclrstart=1;
		}
	}
////////auto set and clear function///////////////////////////////
   autosetclr();
/////////////auto set complition finish indication function//////////	 
  if(autosetdone==1)
  {
    autosetdonesubcount++;
    if(autosetdonesubcount>20)
    {
			autosetdonesubcount=0;
			autosetdone=0;
			//autosetclrshow=1;
			autosetclrshowcount=0;
			autosetclrprocess=0;
			autosetdone=0;
			autosetclrshow=0;
			autosetclrstart=0;
			autoclr=0;
			autoset=0;
      		counter=0;
			asetkey=0;
			aclrkey=0;
			displayx=1;
			refresh=1;
			refresh=1;
    }
  }
  ////////////////////////////////////////////////////////////////////
 /* if(autosetclrshow==1)
  {
    autosetclrshowsubcount++;
		if(autosetclrshowsubcount>8)
		{
			autosetclrshowsubcount=0;
			autosetclrshowcount++;
		}
    if(autosetclrshowcount>9)
    {
      autosetclrshowcount=0;
      autosetclrprocess=0;
      autosetdone=0;
      autosetclrshow=0;
			autosetclrstart=0;
			autoclr=0;
			autoset=0;
      counter=0;
			asetkey=0;
			aclrkey=0;
      displayx=1;
      refresh=1;
    }
  }*/
 /////////////////////////////////////dry restart initiate function///////////////////////////////
	if(dry==1&&restart==1&&errordisplay==1&&dryrestart==0&&vfault==0&&drt>=1)
	{
		dryrestartcount++;
		if(dryrestartcount>90)
		{
			dryrestartcount=0;
			dryrestart=1;
			run3=drt1;
			drterror=1;
			if(mode==2)
			{
				oftimestart=0;
				run2=ontime;
				ontimererror=1;
		    	oftimestarterror=1;
			}
      		refresh=1;
		}
	}
	///////////////////////////////////////////mode change display function////////////////////////////////////////
  if(modechangekey==1&&modedisp==0&&modechangedisp==0)
  {
		modechangingsubcount++;
		if(modechangingsubcount>2)
		{
			modechangingsubcount=0;
    		modechangingcount++;
		}
		
        if(modechangingcount>10)
        {
			modechangingcount=0;
			modechangedisp=1;
			refresh=1;
		}
  }
	if(modechangedisp==1)
	{
		nokeysubcount++;
		if(nokeysubcount>5)
		{
			nokeysubcount=0;
			nokeys++;
		}
		if(nokeys>15)
		{
			nokeys=0;
			modechangedisp=0;
			modedisp=1;
			IE&=~(1<<7);
			Write_DATAFLASH_BYTE1(61001,mode);// mode stored here
			IE|=(1<<7);
		}
	}
	if(modedisp==1)
	{
		modedispsubcount++;
		if(modedispsubcount>5)
		{
			modedispsubcount=0;
			modedispcount++;
		}
		if(modedispcount>10)
		{
			modedispcount=0;
			modedisp=0;
			refresh=1;
			counter=0;
			displayx=1;
		}
	}
  /////////////////////////// this for outer white led glow trigger///////////////////////
	if(switchon==1)
	 pwmvalue1=800;

	if(switchon==1)
	{
		switchoffsubcount++;
		if(switchoffsubcount>10)
		{
			switchoffsubcount=0;
			switchoffcount++;
		}
		if(switchoffcount>2)
		{
			switchoffcount=0;
			switchon=0;
		}
	}
}
//////////////////////////////////run mode end/////////////////////////////////////////////////////////////////
/////////////////////////////////set mode start////////////////////////////////////////////////////////////////
    if(smode==1)
    {
		// if you enter in the set mode it will turn off the pump automatically
		stoprly=1;
	    if(pon==1)
		{
			pon=0;
			if((mode==4)&&(timeron==1||timeron==3||timeron==5||timeron==7)&&(rtcstart==1))
                        rtcstart=0;
			crmrly=0;
			if(stardelta==1)
			{
				startrly=0;
				deltaoncount=0;
				deltaon=0;
				starrelayoff=0;
				starrelayoffcount=0;
				crmrly=0;//main
				relay2=0;
			}
		}
			edit=1;
			address_store=0;
	/////////////////////this is indicate which menu should show based on the mode/////////////////////
		switch(mode) 
			{
        case 1:
        if (smenu<14)
					smenu=14;
           break;
				
        case 2:
        if(smenu==4)
					smenu=14;
           break;
				
        case 3:
        if(smenu<3) 
					smenu=3;
        else if (smenu==4)
					smenu=14;
          break;
				
         case 4:
        if(smenu<3)
					smenu=3;
         break;		
}
////////////////////////////////////////////////////////////////////////////////////////////////////
		//////////////////////set mode display print/////////////////////////////	
			lcdinit();
      display2();
	  ////////////////////////////////////////////////////////////////
    }
	////////////////here it enter in the set mode//////////////////////////////////
    while(smode==1)
    {
			Nop=0;
//////white led glow/////////
    pwmvalue1x=800;
	Pwmduty(800);
////////////set mode menus and display function///////////////////		
  editmode();
if(((smenu>=1)&&(smenu<=25)&&(smenu!=14))||(smenu==28&&errt!=0)||(smenu>=30&&smenu<=34)||(smenu==36))
{
  digitsel();//digit select function
}
if(smenu!=28)
 incrout();//increment function

if(setreset>6000)//////////if user leave the device in set mode it automatically exit from the set mode use this function
		{
			setreset=0;
			smode=0;
			smenu=0;
			ondelaycheck=0;
			start4=0;
			edit=0;
			counter=0;
			errorlog=0;
			edit=0;
	errt1=0;
	errortime=0;
	errortime1=0;
	errt2=0;
	sense=0;
			
			//adctime=0;
			displayx=1;//1st display
			refresh=1;//display refresh command
		}
			//stoprly=0;
		
}
////////////////////////////////////////this is the set mode end////////////////////////////////////////////////////	
	}
}
/*****************************Interrupt*******************************************************/
/*void Serial_ISR(void) interrupt 4
{
    _push_(SFRS); 
    SFRS=0;	
    if (RI)
    { 
		  receivedData[cnt++]= SBUF; // Read received character
			timeout=0;
			lora_datareceived=1;
            if (cnt >= BUFFER_SIZE) 
                cnt = 0;      
				RI=0;
    }
    _pop_(SFRS);

}*/
void Timer() interrupt 3
{
  _push_(SFRS);
	SFRS=0;
  TF1=0;
  TH1=0xcb;
  TL1=0xea;
	//lora_resposecheck();
	//ack_timerout();
	///////////////////////on delay time delay function//////////////////////////////////////////////////////////
	if(reset1==1&&smode==0&&modedisp==0&&modechangedisp==0&&start2==0&&restart==0&&volthigh==0&&voltlow==0)
		 start4++;
	if(start4>99)
	{
		 start4=0;
		ondelaycheck++;
	}
	if(ondelaycheck>=ondelay)
	{
		stoprly=0;
		ondelaycheck=0;
		ondelayok=1;
		
	}
///////////////////////////amps incoming sensing time///////////////////////////////////////////
if(rctval1>=10||yctval1>=10||bctval1>=10)//initial don't care
{
 if(start1==0)
 {
	   start++;
	if(start>500)
	{
		start1=1;	
		start=0;
	}
}
}
///////////////////////////no pump detection function////////////////////////////////////////////////
if(pon==1&&rctval1<10&&nopump==0&&start2==0&&restart==0&&smode==0&&ondelaycheck==0&&starrelayoffcount==0)
{
	if(++Nop>6000)
	{
		Nop=0;
		restart=1;
		nopump=1;
		displayx=1;
		errordisplay=1;
		refresh=1;
		relayoff=1;
		pon=0;
	}
}
//////////////////RTC time date read function//////////////////////////////////////
if(timeseg==1)
{
  timeseg=0;
  timprob();  
  timread();
}
////////////////rtc hour&min  write function////////////////////////////////////// 
if(rtccredit==1)
{
      rtccredit=0;
      rwrbyte1(0x80,0x00);
      rwrbyte1(rtcmin,1);
     rwrbyte1(rtchour,2);
     rwrbyte1(0x00,0x00);
      timprob();
}
////////////////////rtc date, time, month write function//////////////////////////
if(rtccredit==2)
{
  rtccredit=0;
  rwrbyte1(0x80,0x00);
  rwrbyte1(rtcdate1,4);
  rwrbyte1(rtcmonth,5);
  rwrbyte1(rtcyear,6);
  rwrbyte1(0x00,0x00);
  timprob();
}
/*if(smenu==m5&&rtcedit==1&&rtccredit==1)
{
  rtccredit=0;
     timeseg=0;
     rwrbyte1(0x80,0x00);
     rwrbyte1(0x00,0x00);
}*/
///////////////////////////////////water level////////////////////////////////////////////////////////////////////////
if((smode==0)&&(modedisp==0&&modechangedisp==0))
{
		//bottom tank
	
	wtrbot++;
if(wtrbot<25)
  {
if(botlevel==0)//low checking
{
wtrmrbot++;
}
  }
   if(wtrbot>24)
	{
   if(wtrmrbot>9)
	{
     fullbot++;
	}
  if(wtrmrbot<10)
	{
		emptybot++;
		if(emptybot>5)
		{
		emptybot=0;
	  wlcbot=0;	
			//if(rctval>=10)
	     //relayoff=1;
			wlctop1=1;
			if(sumpdry==0)
			{
				relayoff=1;
			sumpdry=1;
				restart=1;
				refresh=1;
if(mode==2&&oftimestart==0)
 {
	 oftimestart=0;	
	  relayoff=1;
	  run2=ontime;
		run3=offtime;
		ontimererror=1;
		oftimererror=1;
 }
  errorview();
	logerror(9,0,1);
			}
		}
	}
  wtrmrbot=0;
  wtrbot=0;
if(fullbot>25)//24
	{
	fullbot=0;
	emptybot=0;
	wlcbot=1;
	 fullbot=0;
	if(sumpdry==1)
			{
				displayrefresh=0;
				sumpdry=0;
				if(ovld==0&&dry==0&&nopump==0)
				  restart=0;
				errordisplay=0;
				displayx=1;
				counter=0;
				refresh=1;
			}
	}
	}
	if(wlcbot==1)
	{
  wtrtop++;
if(wtrtop<25)//201
  {
if(toplevel==0)//full checking
{
wtrmrtop++;
}
  }
   if(wtrtop>24)//200
	{
   if(wtrmrtop>9)
	{
     fulltop++;
	}
  if(wtrmrtop<10)
	{
		checkflag=2;
		emptytop++;
		if(emptytop>5)
		{
		emptytop=0;
		wlctop1=0;
		fulltop=0;
		turnoff=0;
			if(mode==4&&rtcstart==1&&pon==0)
				  rtcstart=0;
		}
	}
  wtrmrtop=0;
  wtrtop=0;
if(fulltop>25)//24
	{
	wlctop1=1;
		if(pon==1&&rctval<10)
		pon=0;
	if((((phase==1||phase==3)&&rctval>=10))||(turnoff==0&&start2==0&&restart==0))
	{
		turnoff=1;
	topfull=1;
 if(relayoff==0)
	relayoff=1;
 if(mode==2&&oftimestart==0)
 {
	 oftimestart=1;	
	  relayoff=1;
	  run2=ontime;
		run3=offtime;
		ontimererror=1;
		oftimererror=1;
 }
	}
	fulltop=0;
	emptytop=0;
	}
	}
}
	

	
}

///////////////////////////////////////eeprom write////////////////////////////////////////////////////////////////////////////
 if(memsave==1)
  {
 datawrite();
 memsave=0;
  }
///////////////////////////////////cyclic timer on timer remaining time write function///////////////////////////////////////////////////////////////////////////////
 if(ontimererror==1)
  {
  ontimererror=0;
  run2h=((run2>>8)&(0x00ff));
	run2l=(run2&0x00ff);
	Write_DATAFLASH_BYTE1(61008,run2h);
	Write_DATAFLASH_BYTE1(61009,run2l);
	Write_DATAFLASH_BYTE1(61003,oftimestart);	
  }
////////////////////////////////cyclic mode off timer start flag store////////////////////////////////////////////////
  if(oftimestarterror==1)
{
	oftimestarterror=0;
	Write_DATAFLASH_BYTE1(61003,oftimestart);
}
//////////////////////////////// cyclic timer off timer remaining time write function//////////////////////////////
if(oftimererror==1)
{
	oftimererror=0;
	run3h=((run3>>8)&(0x00ff));
	run3l=(run3&0x00ff);
	Write_DATAFLASH_BYTE1(61014,run3h);
	Write_DATAFLASH_BYTE1(61015,run3l);
}
//////////////////////////////dry restart remaining time store////////////////////////////////////////////////////////////////////////////////////
	if(drtstore==1)
	{
  drtstore=0;
	run3h=((drt1>>8)&(0x00ff));
	run3l=(drt1&0x00ff);
	Write_DATAFLASH_BYTE1(61060,run3h);
	Write_DATAFLASH_BYTE1(61061,run3l);
	}

//////////////////////////////////////dry error value store//////////////////////////////////////////////////////	

if(drywrite==1)
{
	drywrite=0;
	dryreadh=errorvalue1/256;
	dryreadl=errorvalue1%256;
	Write_DATAFLASH_BYTE1(61062,dryreadh);
	Write_DATAFLASH_BYTE1(61063,dryreadl);
	
}
/////////////////cyclic timer on-time start  flag store///////////////////////////////////////////////////////// 
if(onttimewrite==1)
{
onttimewrite=0;
Write_DATAFLASH_BYTE1(61016,timeron);
}
/////////////////////dry restart error flag store///////////////////////////////////////////////////
if(drterror==1)
{
	drterror=0;
	Write_DATAFLASH_BYTE1(61059,dryrestart);
}
/////////////////////////////fault reset 5 sec time decrement loop///////////////////////////////////
if(resetcount!=0&&resetx==1)
{
	resetcount--;
}
////////////////////////////////set mode auto exit counter/////////////////////////////////////////////
if(smode==1)
setreset++;
//////////////////////////////////////////////////////////////////////////////////////////////////////
_pop_(SFRS);
}

void logerror(unsigned char errortype,unsigned char cterror,unsigned char skiperror)
{
	////////////////error log store function  like current, volt, with error value////////////////
	   Write_DATAFLASH_BYTE1(62002,errorsave1);
	   Write_DATAFLASH_BYTE1((62010+errt),errortype);
	  
		 Write_DATAFLASH_BYTE1(62000,errt);
	   Write_DATAFLASH_BYTE1(62001,errtview);
	
	   Write_DATAFLASH_BYTE1((62030+(errorsave1-1)),runninghour);
	   Write_DATAFLASH_BYTE1((62030+(errorsave1)),runningmin);

     
     Write_DATAFLASH_BYTE1((62051+(errorsave1-1)),runningdate);
	   Write_DATAFLASH_BYTE1((62051+(errorsave1)),runningmonth);
	if(!skiperror)//it skip phase unbalance and current unbalance errors phase reversal like that
	{
		    ev = cterror ? errorvalue1 : errorvalue;
        errorvalueh = ev / 256;
        errorvaluel = ev % 256;
		
	Write_DATAFLASH_BYTE1((62072+(errorsave1-1)),errorvalueh);
	Write_DATAFLASH_BYTE1((62072+(errorsave1)),errorvaluel);
	}
	Write_DATAFLASH_BYTE1((61090+(errt)),phasenumber);
}
void set_clock_source(void)
{
	/////set default clock 16 Mhz/////////////
	set_CKEN_HIRCEN;         
  while((CKSWT & SET_BIT5) == 0);
}
void set_clock_division_factor(unsigned char value)
{
  CKDIV = value;
}
void portinit(void)
{
	// in this function only said the gpio's state -> input/ out put
	dclock=0;
	ddata=0;
	
	SFRS=0;
	P0M1=0x80;
	P0M2=0x7F;
	P0=0xC0;
	//p2.5,p1.4,p1.1 for analog
	SFRS=0;
	P1M1=0x12;
	P1M2=0xed;
	P1=0x12;
	
	SFRS=2;
	P2M1=0x25;
	P2M2=0xda;
	P2=0x21;
  SFRS=0;
 
	//enable& buzzer
	SFRS=0;
	P3M1=0x00;
	P3M2=0xfF;
	P3=0x00;	
}
void timesegread(void)
{
	// this function set a RTC real time read Flag, every 30 sec once it will set, if the flag is one in interrupt MCU read RTC real time
if(timeseg==0)
{	
timesegstart++;
if(timesegstart>=240)//30sec
{
timeseg=1;
timesegstart=0;
} 
}
}
void analog(void)
{ 
	//use this function you can able to read input voltage
	zc();//voltage zero cross detection
	resultx=0;
	if(adcome1==1)
		scanx=164;
if(adcome1==2||adcome1==3)
	  scanx=147;
for(scan=0;scan<scanx;scan++)//2degree//213
	{
			if(adcome1==1)
		  {
				channel=0x0f; 
		  }
      if(adcome1==2)
		  {
				channel=0x0e; 
		  }
      if(adcome1==3)
		  {
				channel=0x07;
		  }
      ADCCON0=channel;
			//ADCCON0=0x0A;
		  for(j=0;j<=10;j++);
			clr_ADCCON0_ADCF;
      set_ADCCON0_ADCS;                  
      while(ADCF == 0);
	    result1=(ADCRH<<4)+(ADCRL&0x0F);
	    result1=result1>>2;
      resultr=result1;
		  if(adcome1==1)
		  {
				channel=0x0e;
		  }
      if(adcome1==2)
		  {
				channel=0x07;
		  }
      if(adcome1==3)
		  {
				channel=0x0f;
		  }
      ADCCON0=channel;
		  for(j=0;j<=10;j++);
			clr_ADCCON0_ADCF;//
      set_ADCCON0_ADCS;//                  
      while(ADCF == 0);//
	    result1=(ADCRH<<4)+(ADCRL&0x0F);//
	    result1=result1>>2;
		  resulty=result1;
      	
      volt=resultr-512;
      cur=resulty-512;
		
			resultry=volt-cur;//28usec for last three operations
			result3=resultry*resultry;
			resultx=resultx+result3;
	}
	
	resultx1=resultx/30;
	icur=sqrt(resultx1);
	if(adcome1==1)
	{
		if(phase==1)
    dispry1=icur/1.087;// here you can calibrate RY voltage- for single phase
		if(phase==3)
    dispry1=icur/1.070;// here you can calibrate RY voltage- for 3 phase
	}
	if(adcome1==2)
	{
  dispyb1=icur/1.016;// here you can calibrate YB voltage - for  3 phase
	}
	if(adcome1==3)
	{
  dispbr1=icur/1.017;// here you can calibrate BR voltage - for 3 phase
	}	
}
void zc(void)
{
	result2=10;
	TMOD|=0x01;
	TH0=0x00;
	TL0=0x00;
	TF0=0;
	TR0=1;
	while(result2>7)
	{
		if(adcome1==1)
		{
			channel=0x0f;//adc channel
			intref=512;//ry volt zero cross Reference
		}
  if(adcome1==2)
		{
			channel=0x0e;//adc channel
			intref=512;//yb volt zero cross Reference
		}
  if(adcome1==3)
		{
			channel=0x07;//adc channel
			intref=512;//br volt zero cross Reference
		}
		if(adcome1==4)
		{
			RCTMUXDIS;// mux channel selection
			intref=rampsreference;//RCT zero cross Reference
			channel=0x0A;//adc channel
		}
  if(adcome1==5)
		{
		  YCTMUXDIS;// mux channel selection
			intref=yampsreference;//RCT zero cross Reference
			channel=0x0A;//adc channel
		}
  if(adcome1==6)
		{
		BCTMUXDIS;// mux channel selection
		intref=bampsreference;//RCT zero cross Reference
		channel=0x0A;//adc channel
		}
		adcconv();
		if(result1>=intref)
  result2=result1-intref;
  if(result1<intref)
  result2=intref-result1;
	timeoutx=((TH0<<8)|TL0);
	if(timeoutx>38869)
	{
		// we are adding time out to avoid lock in the while loop
		TR0=0;
		TF0=0;
		result2=0;
	}		
	}
	
}
void ctscan(void)
{
	// this function used to read single phase line voltage and current reading
	zc();
	result6=0;
	result4=0;
	result5=0;
	store1=0;
	if(adcome1==3)
		scanx=263;
	if(adcome1!=3)
		scanx=320;
	for(scan=0;scan<scanx;scan++)//2degree//213
	{
		if(adcome1==1)
		{
			channel=0x0f;//R phase adc reading channel selection
		}
  if(adcome1==2)
		{
			channel=0x0e;//R phase adc reading channel selection
		}
  if(adcome1==3)
		{
			channel=0x07;//R phase adc reading channel selection
		}
		if(adcome1==4)
		{
			RCTMUXDIS;// mux channel selection
			channel=0x0A;//RCT  adc reading channel selection
		}
  if(adcome1==5)
		{
		YCTMUXDIS;// mux channel selection
		channel=0x0A;//YCT  adc reading channel selection
		}
  if(adcome1==6)
		{
		BCTMUXDIS;// mux channel selection
		channel=0x0A;//BCT phase adc reading channel selection
		}
     adcconv();
		
if(result1>=intref)
result4=result1-intref;
if(result1<intref)
result4=intref-result1;	
if(result4<3)
	result4=0;
  result5=result4*result4;
if(adcome1>=4&&adcome1<=6)
	 {
		store1=((alpha*store1)+(1-alpha)*(result5));
	 }
	 if(adcome1>=1&&adcome1<=3)
		  store1=result5;

	result6=store1+result6;	
	}
	CTMUXDIS;
	resultx1=result6/10;
  icur2=sqrt(resultx1);
	if(adcome1==1)
	{
		rphvolt1=icur2/2.531;// here you can calibrate R-N Phase voltage 
	}
	if(adcome1==2)
	{
		yphvolt1=icur2/2.522;// here you can calibrate Y-N Phase voltage 
	}
	if(adcome1==3)
	{
		bphvolt1=icur2/2.304;// here you can calibrate B-N Phase voltage 
	}
	if(adcome1==4)
	{
	rctval1=icur2/1.396;// here you can calibrate R CT 
  if(rctval1<10)
    rctval1=0;	
  if(restart==0&&start2==0)
  errorvalue1=rctval;	
	
	if((dry==1||ovld==1)&&(restart==0&&rctval1<10))
	{
		ovld=0;
		dry=0;
	}
	}
	if(adcome1==5)
	{
	yctval1=icur2/1.406;// here you can calibrate Y CT
  if(yctval1<10)
  yctval1=0;		
	}
	if(adcome1==6)
	{
	bctval1=icur2/1.367;// here you can calibrate B CT
  if(bctval1<10)
  bctval1=0;	
	}
}
void floatscan(void)
{
	//bottom float scanning funtion
	FLTMUXDIS ;
	channel=0x0A;
	result5=0;
	result6=0;
	for(scan=0;scan<50;scan++)
	{
	adcconv();
	result5=result5+result1;
	}
	result6=result5/50;
	if(result6>=512)
		botlevel=1;
	if(result6<512)
		botlevel=0;
}
void adcconv(void)
{
	// adc coversion function 
ADCCON0=channel;
for(j=0;j<=10;j++);
clr_ADCCON0_ADCF;//
set_ADCCON0_ADCS;//                  
while(ADCF == 0);//
result1=(ADCRH<<4)+(ADCRL&0x0F);//
result1=result1>>2;	
}
void readreference(void)
{
	// use this function initailly you can read R,Y,B ct References
	//EA=0;
	resultct=0;
	for(ibm=0;ibm<3;ibm++)
	{
		resulty=0;
		result1=0;
		resultct1=0;
		resultct=0;
		for(doad=0;doad<100;doad++)
		{
			//stoprly=1;
			if(ibm==0)
			{
				RCTMUXDIS;
				//channel=0x0A;
			}
			if(ibm==1)
			{
				YCTMUXDIS;
				//channel=0x0A;
			}
			if(ibm==2)
			{
				BCTMUXDIS;
				//channel=0x0A;
			}
			channel=0x0A;
			adcconv();
			resultct=resultct+result1;
		}
		resultct1=resultct/100;
		if(ibm==0)
	rampsreference=resultct1;//for R ct
		if(ibm==1)
	yampsreference=resultct1;//for Y ct
		if(ibm==2)
	bampsreference=resultct1;//for B ct
	}
	//EA=1;
}

void unbalcal(void)
{
	//we are not use this function in this code
ppdiff=ppvolt-avgvolt;
if(ppdiff>65000)
ppdiff=65536-ppdiff;
pppercent=(ppdiff*100)/avgvolt; 
}
void phrcheck(void)
{	
	// this function checks the phase reversal and phasefail error
//RPHASEMUXDIS;	
  result2=10;
	TMOD|=0x01;
	TH0=0x00;
	TL0=0x00;
	TF0=0;
	TR0=1;
  while(result2>7)
  {

		channel=0x0f;
		adcconv();
		
	if(result1>=511)
  result2=result1-511;
  if(result1<511)
  result2=511-result1;
	
	timeoutx=((TH0<<8)|TL0);
	if(timeoutx>38869)
	{
		TR0=0;
		TF0=0;
		result2=0;
	}	
	}
	//YPHASEMUXDIS;
	
  T2MOD = T2MOD|0xA0;//prescalar 16mHZ/16
	TH2 = 0X00;        
  TL2 = 0X00;
  TR2 = 1;	   //timer2ON
  delay(5);//1msec	
	
	result2=10;
	TMOD|=0x01;
	TH0=0x00;
	TL0=0x00;
	TF0=0;
	TR0=1;
  while(result2>7)
  {
		channel=0x0e;
		adcconv();
	if(result1>=511)
  result2=result1-511;
  if(result1<511)
  result2=511-result1;
	
	timeoutx=((TH0<<8)|TL0);
	if(timeoutx>38869)
	{
		TR0=0;
		TF0=0;
		result2=0;
	}	
	}
	TR2 = 0;
	result=((TH2<<8)|TL2);
	resultt=result/8;
	
	
if(((rphvolt1<80)&&(phase==3)&&start2==0)||((rphvolt1<80)&&(phase==3)&&(start2==1)&&(phasereversed==1)))//phase fail condition for rphase
{
	       
          rphfail=1;
          phasenumber=1;
	
				
}
if(((yphvolt1<80)&&(phase==3)&&start2==0)||((yphvolt1<80)&&(phase==3)&&(start2==1)&&(phasereversed==1)))//phase fail condition for yphase
{
	       
		 yphasefailcount=0;
          yphfail=1;
          phasenumber=2;
				
}
if(((bphvolt1<80)&&(phase==3)&&(start2==0))||((bphvolt1<80)&&(phase==3)&&(start2==1)&&(phasereversed==1)))//phase fail condition for b phase
{
	
          bphfail=1;
          phasenumber=3;
}
if((rphfail==1||yphfail==1||(bphfail==1))&&(phase==3))
{
	if(phasereversed==1&&start2==1)
	{
		start2=0;
		phasereversed=0;
	}
	phfail++;
	if(phfail>10)
	{
	phfail=0;
	phasefail=1;
	errorvalue=0;
	if(restart==1)
	   {
		      vfault=1;
			}
	}
}

if(phase==3&&rphfail==0&&yphfail==0&&bphfail==0&&twophaseselected==0&&voltlow==0&&volthigh==0)
	{
	 //phasereversed=1;
if((resultt>675)&&(resultt<1012))
{
      phasereversed=0;
	    phrtime=0;
}
if((resultt<675)||(resultt>=1012))
{
	   phrtime++;
	if((phrtime>6&&rctval1<10)||(phrtime>30&&rctval1>=10))
	{
		
      phasereversed=1;
		  phrtime=0;
		if(restart==1)
		{
			vfault=1;
		}
	}
}
}
}
void autosetclr(void)
{
if(autosetclrstart==1)
{
   testcount++;
	if(autoclr==1)
		{
			//auto clear function
			if(phase==1)
			{
		ovl=300;
		dryrun=10;
		lv=150;
		hv=320;
			}
			if(phase==3)
			{
				ovl2=ovl=320;
				dryrun2=dryrun=40;
				hv2=hv=470;
				lv2=lv=285;
			}
		//setpfd=65;
		//rst=0;
	if(sumpdry==0)
		restart=0;//current error flag
		start2=0;// voltage error flag
		volthigh=0;// high voltage error flag
		voltlow=0;// low voltage error flag
		ovld=0;// overload error flag
		dry=0;// dry run error flag
    msec=0;//error cuttoff counter 
			run7=0;
			unbalancecurrent=0;//unbalancecurrent error flag
			if(dryrestart==1)
		 {
		   dryrestart=0;// dryrestart trigger flag
			 drt1=drt;//dry restart time store variable
			 drtstore=1;//dry restart store flag
		 }
			drterror=1;
			errordisplay=0;//if this bit is one-> the display will show the error
			vfault=0;
		//ovtime=0;
		//dtime=0;
		//vtime=0;
		//ssc=0;
		//cct=0;
		//dispc=0;
		manualoff=0;
		 Nop=0;//no pump cut-off timer counter
		 nopump=0;// no pump error flag
		 pon=0;// pump on and off indication flag
		// if(stoprly==1)
			// stoprly=0;
//		cyclicon=0;
		//manualoffsense=0;
		}
		if(autoset==1&&rctval>=10)
		{
			// auto set loop
			if(testcount<=40)//40 times taking a average
			{
				if(phase==1)
				{
			setcurrent=rctval1+setcurrent;//this is for current
			setvoltage=dispry1+setvoltage;// this for voltage
				}
				if(phase==3)
				{
					setvoltagex1=dispry1+setvoltagex1;//
					setvoltagex2=dispyb1+setvoltagex2;//-> 3 phase average voltage
					setvoltagex3=dispbr1+setvoltagex3;//
					setcurrent=rctval1+setcurrent;// r phase current
				}
			//setpf=ftime+setpf;
			}
			if(testcount==40)
			{
				if(phase==1)
				{
				setcurrent1=setcurrent/40;
				setvoltage1=setvoltage/40;
				}
				if(phase==3)
				{
					rphasesetvolt=setvoltagex1/40;
					yphasesetvolt=setvoltagex2/40;
					bphasesetvolt=setvoltagex3/40;
					setvoltage1=((rphasesetvolt+yphasesetvolt+bphasesetvolt)/3);
					setcurrent1=setcurrent/40;
				}
				//setpf1=setpf/40;
				setcurrent=0;
				setvoltage=0;
				setvoltagex1=0;
				setvoltagex2=0;
				setvoltagex3=0;
				rphasesetvolt=0;
				yphasesetvolt=0;
				bphasesetvolt=0;
				//setpf=0;
				//if(rctval1>7)
				//{
					
				/*hv=(setvoltage1*13)/10;
					if(hv>=275&&phase==1)
						  hv=275;
				lv=(setvoltage1*7)/10;
					if(lv<=150&&phase==1)
						lv=150;
					hv=(setvoltage1*13)/10;
					if(hv>=600&&phase==3)
						  hv=600;
				lv=(setvoltage1*7)/10;
					if(lv<=285&&phase==3)
						lv=285;
				  
				ovl=(setcurrent1*12)/10;
					if(ovl>=200&&phase==1)
						ovl=200;
					if(ovl>=500&&phase==3)
						ovl=500;
				dryrun=(setcurrent1*8)/10;
					if(dryrun<=10)
						dryrun=10;*/
						//storing based on the Phase- 1 or 3 Phase
						if(twophaseselected==0)
				hv=(setvoltage1*12)/10;
					if(twophaseselected==1)
				hv2=(setvoltage1*12)/10;
					
					if(hv>=275&&phase==1)
						  hv=275;
					if(hv>=600&&phase==3&&twophaseselected==0)
						  hv=600;
					if(twophaseselected==1&&hv2>=600)
						hv2=600;
					
					if(twophaseselected==0)
				lv=(setvoltage1*8)/10;
					
					if(twophaseselected==1)
				lv2=(setvoltage1*8)/10;
					
					if(lv<=150&&phase==1)
					    	lv=150;

					if(lv<=285&&phase==3&&twophaseselected==0)
						lv=285;
					
					if(lv2<=285&&twophaseselected==1)
						lv2=285;
				  
					if(twophaseselected==0)
				ovl=(setcurrent1*12)/10;
					if(twophaseselected==1)
				ovl2=(setcurrent1*12)/10;
					if(ovl>=200&&phase==1)
						ovl=200;
					if(ovl>=500&&phase==3&&twophaseselected==0)
						ovl=500;
					
					if(ovl2>=500&&twophaseselected==1)
						ovl2=500;
					
					if(twophaseselected==0)
				dryrun=(setcurrent1*8)/10;
						if(twophaseselected==1)
				dryrun2=(setcurrent1*8)/10;
					if(dryrun<=10&&twophaseselected==0)
						dryrun=10;
					
					if(dryrun2<=10&&twophaseselected==1)
					  	dryrun2=10;
					
					
				//setpfd=(setpf1*85)/100;
         // if(setpfd<=65)
           //  setpfd=65;						
				//}
			}	
		}
	if(testcount>80)
	{
		testcount=0;// auto set clr counter flag
		memsave=1;//parameter save flag
		autosetclrstart=0;//auto set clear  start average Flag 
    autosetdone=1;// auto set complition flag
    refresh=1;// display update flag
    displayx=2;//display page 2
		sense=0;
		adctime=0;
		
    
		//readdata();
		//setfunction=0;
		//clrfunction=0;
		//testfunction=0;
		//testcount=0;	
	}
}		
}
void decide(void)
{
	// this function handles all voltage related errors
	if(twophaseselected==0)
	{
		//lowvolt& high volt this variable using reason is memory optimization 
	//3 phase and 1 phase  
	lowvolt=lv;
	highvolt=hv;
	}
	if(twophaseselected==1)
	{
		//only for 2 Phase 
	lowvolt=lv2;
	highvolt=hv2;
	}

if(phasefail==0&&phasereversed==0&&phaseunbalance==0)
{
chkonce++; 
if(chkonce>2)//1
{
chkonce=0;
	
if(((dispry<lowvolt||dispyb<lowvolt||dispbr<lowvolt)&&(phase==3))||(dispry<lowvolt&&phase==1))//295//low voltage condition
	{
		//low voltage detection loop
voltlow=1;
	if(start2==0&&phase==1)// this is for single phase error
		{
    errorvalue=dispry;
		if(restart==1)
	 {
		 vfault=1;
	 }
		}
//if(phase==3&&(dispry<lowvolt&&(dispry<dispyb)&&(dispry<dispbr))&&start2==0)
if((phase==3)&&(dispry<lowvolt)&&(start2==0))// this is for 3 phase voltage error
{

errorvalue=dispry;// this is using for store the error value to eeprom
phasenumber=1;// this is used to identify which phase error happen ex: phase number=1-R phase,2-Y phase , 3- B phase
	if(restart==1)// this bit indicates current error happened
	 {
		 vfault=1;// this bit only use for if already device in current error at the same time voltage error happens this flag will be set by the device
	 }
}
if((phase==3)&&(dispyb<lowvolt)&&(start2==0))
{
errorvalue=dispyb;
phasenumber=2;
	if(restart==1)
	 {
		 vfault=1;
	 }
}
if((phase==3)&&(dispbr<lowvolt)&&(start2==0))
{
errorvalue=dispbr1;
phasenumber=3;
	if(restart==1)
	 {
		 vfault=1;
	 }
}

}
}


if(((((dispry>highvolt)||(dispyb>highvolt)||(dispbr>highvolt))&&phase==3))||(dispry>highvolt&&phase==1))//high voltage condition
{
	//high voltage detection loop
	volthigh=1;
		if(start2==0&&phase==1)
		{
    errorvalue=dispry;
		if(restart==1)
	 {
		 vfault=1;
	 }
		}
if((phase==3)&&(dispry>highvolt)&&(start2==0))
{
  errorvalue=dispry;
  phasenumber=1;
	if(restart==1)
	 {
		 vfault=1;
	 }
}
if((phase==3)&&(dispyb>highvolt)&&(start2==0))
{
errorvalue=dispyb;
phasenumber=2;
	if(restart==1)
	 {
		 vfault=1;
	 }
}
if((phase==3)&&(dispbr>highvolt)&&(start2==0))
{
errorvalue=dispbr;
phasenumber=3;
	if(restart==1)
	 {
		 vfault=1;
	 }
}
   }
}
if(voltlow==0&&volthigh==0&&phasefail==0&&phasereversed==0&&phase==3)
{
 /*if((rypercent>5||ybpercent>5||brpercent>5)&&(twophaseselected==0))  	 //unbalance condition
 {phaseunbalance=1;
	 if(restart==1)
	 {
		 vfault=1;
	 }
 }*/
	
	checkvoltvalues(dispry,dispyb,dispbr);// this is for volatge unbalance checking function
	if(phaseunbalance==1)
	{
	if(restart==1)
	 {
		 vfault=1;
	 }
}    }                                                //normal condition
if(mode>=2&&mode<=4) 
{
	//mode=2 auto mode
	//mode=3 cyclic mode
	//mode=4 RTC mode
//this loop if all volatge comes under in the normal level the error will cleard by the devcie auto matically(only for the voltage error)
if(((checkvolt(dispry)&&checkvolt(dispyb)&&checkvolt(dispbr))&&(phase == 3))||(checkvolt(dispry)&&(phase==1)))
{ 
	//if the voltage in safe range the device wll clear the clear the error flag
  volthigh=0;
  voltlow=0;
 }
 if((phase==3)&&(rphvolt1>80&&yphvolt1>80&&bphvolt1>80))
 {
	//if the voltage in safe range the device wll clear the clear the error flag
	phfail=0;
  rphfail=0;
  yphfail=0;
  bphfail=0; 
	 //
	 phasefail=0;
	 //
	 rphasefailcount=0;
	 yphasefailcount=0;
	 bphasefailcount=0;
 }
 }
 if((mode>=2&&mode<=4)&&voltlow==0&&volthigh==0&&phasefail==0&&start2==1&&phaseunbalance==0&&phasereversed==0)//autonormal for voltage error
 {
if(start2==1)
{
	//if the voltage in safe range the device wll clear the clear the error flag
	start2=0;
	if(restart==0)
	errordisplay=0;
	displayrefresh=0;
	vfault=0;
	displayx=2;
  counter=0;
	refresh=1;
	sense=0;
	adctime=0;
	msec=0;
//	if(stoprly==1)
	//	stoprly=0;
}
} 
}
void vcutoff(void)
{
	// this function used for if any voltage error happens, it will start the counter flag to cut off the pump
	hvmax=highvolt+40;
if(((volthigh==1)||(voltlow==1)||(phaseunbalance==1)||(phasefail==1)||(phasereversed==1))||(vfault==1&&start2==0))
{
	if((restart==0)&&(ovld==1||dry==1))
	{
		dry=0;
		ovld=0;
		if(sumpdry==0)
		restart=0;
	}
	if(((restart==0)&&start2==0)||(vfault==1&&start2==0))
	{
		msec++;// this the loop timer counter flag
		//if(((((dispry>highvolt)&&(dispry<=hvmax))||(dispry<lowvolt))&&(phase==1))||((phase==3)&&((((dispry>highvolt)&&(dispry<=hvmax))||(dispry<lowvolt))||(((dispyb>highvolt)&&(dispyb<=hvmax))||(dispyb<lowvolt))||(((dispbr>highvolt)&&(dispbr<=hvmax))||(dispbr<lowvolt)))||(phaseunbalance==1)||(vfault==1&&start2==0))
		if((((dispry>highvolt)||(dispry<lowvolt))&&(phase==1))||((phase==3)&&((dispry||dispyb||dispbr)>highvolt)&&(phaseunbalance==1)))	
		msec=msec;
	//	if((((dispry>hvmin)&&(dispry<=hvmax))&&(phase==1))||((((dispry>hvmin)&&(dispry<=hvmax))||((dispyb>hvmin)&&(dispyb<=hvmax))||((dispbr>hvmin)&&(dispbr<=hvmax)))&&(phase==3)))
		//	msec=msec+2;
		if(((dispry>hvmax)&&(phase==1))||(((dispry>hvmax)||(dispyb>hvmax)||(dispbr>hvmax))&&phase==3))
			msec=msec+5;
	}
if((msec>196&&vfault==0)||((phasereversed==1||phasefail==1)&&phase==3&&start2==0)||(vfault==1&&msec>100&&start2==0&&(phase==3||phase==1)))//For disp off and relay on//100
{
if(rctval>=10)
	relayoff=1;
	 msec=0;
 start4=0;
	ondelaycheck=0;
displayrefresh=0;
	 start2=1;
	 //restart=0;
	 errorview();
IE&=~(1<<7);
	if(volthigh==1)
	{
		//volthighwrite=1;
		logerror(1,0,0);
	}
	if(voltlow==1)
	{
		//voltlowwrite=1;
		logerror(2,0,0);
	} 
	if(phaseunbalance==1)
	{
		//unbalancewrite=1;
		logerror(5,0,1);
	}
	if(phasefail==1)
	{
		//phasefailwrite=1;
		logerror(6,0,0);
	}
	if(phasereversed==1)
	{
		//phasereversedwrite=1;
		logerror(7,0,1);
	}
	  if(vfault==1&&restart==1)
		{
			refresh=1;
			/*if(dryrestart==1)
			{
				dryrestart=0;
				drterror=1;
		}*/}
		IE|=(1<<7);
}  
}
  
}
void unblanceampscut(void)
{
	// this is for unbalance current cuttoff function 
	if(unbalancecurrent==1)
	{
		if(restart==0&&start2==0)
		{
			ubc++;// these the loop cuttoff timer flag
		}
		if(ubc>230)
		{
			ubc=0;
			relayoff=1;// turnoff the relay
			restart=1;// current error indication flag
			errorview();// incrementing index of the error log
			displayrefresh=0;
			IE&=~(1<<7);// disable the global interrupt
			if(unbalancecurrent==1)
			{
					logerror(8,0,1);// error storing in the eeprom
			}
			IE|=(1<<7);// enable the global interrupt
		}
			
	}
}
void drycuttoff(void)
{
	// if dry run error happens this function will execute
if((dry==1))
{
	if((restart==0)&&(start2==0))
	{
		dtime++;// these the loop cuttoff timer flag
	}
if((dtime>(drycut1*9)))//For disp off and relay on//28173//40 sec
{
	dtime=0;
	relayoff=1;
	restart=1;
  errorview();
	displayrefresh=0;
	IE&=~(1<<7);
	if(dry==1)
	{
		if(unbalancecurrent==1)
			unbalancecurrent=0;
		drywrite=1;
		logerror(3,1,0);
	}
	IE|=(1<<7);
}
}	
}
void ovlcuttoff(void)
{
	// if overload run error happens this function will execute
if((ovld==1))
 {
if((restart==0&&start2==0))
	{
	 ovtime++;
if(ovld==1)
		{
			//initialovl=((150-disp4));
			//initialovl=(32+(initialovl*2));//11264//8448
			initialovl=((ovlcut1*9)-(((disp4-101)*((ovlcut1*9)-4))/(200-101)));
		}
	}
if(ovtime>initialovl)//For disp off and relay on
{	
	if(unbalancecurrent==1)// if device cut off based on the overload, that time unbalancecurrent current error happens it will clear the other error
	unbalancecurrent=0;
	 relayoff=1;
	 ovtime=0;
	 restart=1;
	 initialovl=0;
	displayrefresh=0;
	 errorview();
	IE&=~(1<<7);
	 if(ovld==1)
	 {
		 //ovlwrite=1;
		 logerror(4,1,0);
	 }
	 IE|=(1<<7);
}  
}
}
void curdecide(void)
{
	// this function handles all current related errors
	if(twophaseselected==0)
	{
		overload=ovl;
		dryx=dryrun;
	}	
	if(twophaseselected==1)
	{
		overload=ovl2;
		dryx=dryrun2;
	}
if((start2==0)&&(rctval>=10)&&(volthigh==0)&&(voltlow==0)&&(start1==1))
   {
		  disp2=overload;
		//  if(rctval>=500)
		  //rctval=500;
	    disp4=rctval*100;
	    disp4=disp4/disp2;
	    if(disp4>=200)
      disp4=200;			
			
			if(rctval>(dryx+(dryx/10)))
			{
				dry=0;
				dtime=0;
			}
			if(rctval<(overload-(overload/10)))
			{
				ovld=0;
				ovtime=0;
			}
			if(rctval<=dryx)			
			{
				dry=1;//dry run error flag
        unbalancecurrent=0;	
        ubc=0;				
			}
			if(rctval>=overload)
			{
				ovld=1;// over load error flag
        unbalancecurrent=0;	
        ubc=0;						
			}
	
  }	
} 
void pwmfunction(void)
{ 
	// this function for pwm generation function
	//  if(mode!=3)
	   // wlctop1=0;
	//if(switchon==0&&rctval1<10&&ovld==0&&dry==0&&volthigh==0&&voltlow==0&&phasereversed==0&&phasefail==0&&phaseunbalance==0&&wlctop1==0&&modedisp==0&&modechangedisp==0&&nopump==0)
   if(switchon==0) 	        
	   pwmvalue1=0;// pwm value zero percent

	if(switchon==0&&modedisp==0&&modechangedisp==0)
	{
	if(rctval1>=10)
		 pwmvalue1=200;// pwm value 20 percent-pump on
 
	if(wlctop1==1&&sense!=0&&start2==0&&restart==0&&wlcbot==1)
		 pwmvalue1=300;// pwm value 30 percent-float led
		
  if((volthigh==1||voltlow==1||phasereversed==1||phasefail==1||phaseunbalance==1))
		 pwmvalue1=400;// pwm value 40 percent-volatge error
	
	if((dry==1||ovld==1||unbalancecurrent==1||(sumpdry==1&&start2==0)||(nopump==1&&start2==0))&&(vfault==0))
		    pwmvalue1=500;// pwm value 50 percent-current  error
}
	//if(mode==1&&(modedisp==1||modechangedisp==1))
		//pwmvalue1=0;
	if(mode==2&&(modedisp==1||modechangedisp==1))
		        pwmvalue1=600;// pwm value 60 percent for auto mode
	if(mode==3&&(modedisp==1||modechangedisp==1))
		        pwmvalue1=700;// pwm value 70 percent for cyclic mode
	if(mode==4&&(modedisp==1||modechangedisp==1))
		         pwmvalue1=100;// pwm value 10 percent for RTC mode
	if(pwmvalue1!=pwmvalue1x)
	{
		// if pwm value change then only it will execute
		pwmvalue1x=pwmvalue1;
	Pwmduty(pwmvalue1);
	}
}
void adc_init(void)
{
	//p1.1 ch7
	//p2.5 ch15
	//p1.4 ch14
	SFRS=0;
	//AINDIDS0=0x80;//p11
	
	SFRS=2;
	AINDIDS1=0xC4;//p25,p14,p22
	SFRS=0;
	
  ADCCON1=0x31;	
	ADCCON2=0X0E;
}
void adcal(void)
{
	//adc calculation function
	  clr_ADCCON0_ADCF;
    set_ADCCON0_ADCS;                  
    while(ADCF == 0);
    result1=(ADCRH<<4)+(ADCRL&0x0F);
		result1=result1>>2;
}
/*void adcal1(void)
{
ADCCON0=channel;
adcal();
intref1=intref;
	
if(channel!=0x0c)
   intref1=512;
if(result1>=intref1)
result2=result1-intref1;
if(result1<intref1)
result2=intref1-result1;    
}*/
void delay(char gg)
{
	//delay for lcd display
	  TMOD|=0x01;
	  TH0=0xfe;
		TL0=0xf5;//200us
	  TR0=1;
    for(k=0;k<gg;k++)
    {
			TH0=0xfe;
			TL0=0xf5;//200us
			TF0=0;
			while(TF0==0);
    }
		TR0=0;
}
void timer_init(void)
{
	//timer initialisation function
  TR1=0;
  TMOD|=(1<<4);
  TH1=0xcb;
  TL1=0xea;
  IP&=~(1<<3);
  IPH|=(1<<3);
  IE|=(1<<3);
  IE|=(1<<7);
	TR1=1;
}
void init(void)
{
 set_clock_source();
 portinit();
}
void lcdinit(void)	
{
	//lcd initialisation function
	  reg=0;
    chip=0;
	while(reg<15)
	{
	reg7=init2[reg];
	enabler(reg7);
	reg++;
	if(reg==14)
	return;
	reg7=init2[reg];
	delay(reg7);
	reg++;
	}
}
void fline(void)
{
	//display 1st line
chip=0;
enabler(0x80);
delay(2);
chip=1;
}
void sline(void)
{
	//display 2nd line
chip=0;
enabler(0xc0);
delay(1);
chip=1;
}
void valuewrite(void)
{
	//this function for set mode value printing function
   chip=0;
   enabler(address);// it said priniting values starting address
   delay(3);
    chip=1;
    //if((smenu==1)||(smenu==3)||(smenu==2)||(smenu==12)||(smenu==15)||(smenu==16)||(smenu==17)||(smenu==18))
	//if((smenu>=1&&smenu<=3)||(smenu==12)||(smenu>=15&&smenu<=18)||(smenu>=30&&smenu<=33))
        limit=4;
    //if(smenu==4||(smenu==5)||(smenu==6)||(smenu==7)||(smenu==8)||(smenu==9)||(smenu==10)||(smenu==11)||(smenu==24))
    if((smenu>=4&&smenu<=11)||(smenu==24))  
		    limit=5;// limit said value's digit size
      /*if(smenu==x)
      limit=3;*/
      if(smenu==25||smenu==13)
         limit=7;
			if(smenu==34||smenu==19||smenu==22||smenu==36)
				  limit=3;
for(ibm=1;ibm<limit;ibm++)
{
	//if((ibm==3)&&(smenu==4||(smenu==5)||(smenu==6)||(smenu==7)||(smenu==8)||(smenu==9)||(smenu==10)||(smenu==11)||(smenu==24)))
if(ibm==3)// ibm ==3 skip the index like 32.0 to skipping the decimal printed position
	{ 
	if(((smenu>=4&&smenu<=11)||(smenu==24)))
	{
  chip=0;
   enabler(0xcb);
   delay(3);
   chip=1;}
   if(((smenu==17||smenu==18)||(smenu==32)||(smenu==33)))
   {
   chip=0;
   enabler(0xcc);
   delay(3);
   chip=1;
   }
  if((smenu==25||smenu==13))
   {
    chip=0;
   enabler(0xca);
   delay(3);
   chip=1;
   }
 }
   if((ibm==5)&&(smenu==25||smenu==13))
   {
   chip=0;
   enabler(0xcd);
   delay(3);
   chip=1;
   }
temp=dbuf[ibm];
enabler(temp);
delay(0x01);
}
}

void valuewrite2(void)
{
	// this only used for time display and error log value priniting
   chip=0;
   enabler(address);
   delay(3);
   chip=1;
	
  //if(((displayx==1)||(displayx==2)||(displayx==3&&phase==3))||(vshow==1&&autosetclrshow==1)||(autosetclrshow==1&&(amps==1||amps==2)))
   limit=4;
   if(((((displayx==1)&&amps==1&&autosetclrshow==0)||(autosetclrshow==0&&displayx==2&&timedisplay==1))&&(errorlog==0))||(errorlog==1&&twodigitdisp==0))
     	limit=5;     
   if((errorlog==1&&twodigitdisp==1)||((asetkey==1||aclrkey==1)&&(autosetclrprocess==0)))
     	limit=3;
    // if(errorlog==1&&twodigitdisp==0)
     //limit=5;
		 
	//if((asetkey==1||aclrkey==1)&&(autosetclrprocess==0))
    //     limit=3;
	for(ibm=1;ibm<limit;ibm++)
	{
		if(ibm==3)
		{
			if(displayx==2&&timedisplay==1&&asetkey==0&&aclrkey==0&&autosetclrprocess==0)
			{
				chip=0;
				enabler(0x83);
				delay(2);
				chip=1;
			}
  /*if(autosetclrshow==1&&amps==1)
  {
    chip=0;
    enabler(0x8e);
    delay(2);
    chip=1;
  }
  if(autosetclrshow==1&&amps==2)
  {
    chip=0;
    enabler(0xce);
    delay(2);
    chip=1;
  }*/
			if(errorlog==1&&vshow==1)
			{
				chip=0;
				enabler(0x88);
				delay(2);
				chip=1;
			}

			if(errorlog==1&&amps==1)
			{
				chip=0;
				enabler(0x8e);
				delay(2);
				chip=1;
			}
		}
		temp=dbuf[ibm];
		enabler(temp);
		delay(0x01);
	}
}

void errordisplayprint(void)
{
	//only priniting the error value in error log and  error happening time
    chip=0;
    enabler(address);
    delay(3);
    chip=1;
    if((errordisplay==1&&smode==0&&errorlog==0&&(volthigh==1||voltlow==1||phasefail==1))||((smode==1&&errorlog==1&&errorprint==1)&&(errordisplay1==1||errordisplay1==2||errordisplay1==6)))
       limit=4;//limit indicates digit size 
	if(((errordisplay==1&&smode==0&&errorlog==0&&vfault==0)&&(ovld==1||dry==1))||((smode==1&&errorlog==1&&errorprint==1)&&(errordisplay1==3||errordisplay1==4)))
         limit=5;
	for(ibm=1;ibm<limit;ibm++)
	{ 
		temp=dbuf[ibm];
		enabler(temp);
		delay(0x01);
	}
}

void valuewritex(void)
{
	//run mode voltage current time , drt , on and off time values printing function
	if(autosetclrshow==0&&autosetclrprocess==0&&modedisp==0&&autosetdone==0&&asetkey==0&&aclrkey==0)
  	{
		if(displayx==1)
		{
	////////////////////////////////////////////3 ph display printing///////////////////////////////////////////////////
  		if(phase==3)
  		{
		//buffer=wlcbot;
			buffer=dispry;
  			split2();
  			address=0x82;
			valuewrite2();

		//buffer=voltlow;
			buffer=dispyb;
  			split2();
  			address=0x87;
			valuewrite2();

			buffer=dispbr;
		//buffer=bampsreference;
  			split2();
  			address=0x8c;
			valuewrite2();

//if((errordisplay==0)||(errordisplay==1&&dryrestart==1&&autosetclrprocess==0&&autosetclrshow==0&&mode==3))
		if(errordisplay==0)
		{
			buffer=rctval;
			amps=1;
			split2();
			address=0xc2;
			valuewrite2();
			amps=0;

			buffer=yctval;
			amps=1;
			split2();
			address=0xc7;
			valuewrite2();
			amps=0;

			buffer=bctval;
			amps=1;
			split2();
			address=0xcc;
			valuewrite2();
			amps=0;
		}
	}
	/////////////////////////////////single phase display print section///////////////////////////////////
if(phase==1)
{
   //buffer=rctval;
	buffer=dispry;
    split2();
    address=0x80;
    valuewrite2();
//if((errordisplay==0)||(mode==3&&errordisplay==1&&dryrestart==1&&autosetclrprocess==0&&autosetclrshow==0))
	if(errordisplay==0)
    {
			//buffer=rctval1;
		buffer=rctval;
		amps=1;
		split2();
		address=0xc0;
		valuewrite2();
		amps=0;}
	}
}

//////////////////////////////////2nd page for 3 ph/////////////////////////////////////////////////////////////////////
if((displayx==2)&&(phase==3))
{
  timedisplay=1;
  buffer=runningtime;
  split2();
  address=0x80;
  valuewrite2();
  timedisplay=0;
  //if((mode==2&&rctval1>7)||(mode==2&&rctval1<7)||(mode==3&&dryrestart==1&&dry==1&&restart==1))
  if((mode==2&&oftimestart==0)||(mode==2&&oftimestart==1)||(dryrestart==1&&dry==1&&restart==1))
  {
 	if(oftimestart==0&&dryrestart==0&&wlctop1==0)
 	{
 		buffer=run2;
 		if(manualoff==0)
		{
			split2();
			address=0xcc;//cd
			valuewrite2();
		}
    }
//if(((rctval1<7&&oftshow==1)&&(mode==2)&&(oftimestart==1)&&(manualoff==0))||(dryrestart==1&&dry==1&&restart==1&&mode==3))
	if(((mode==2)&&(oftimestart==1)&&(manualoff==0)&&(dryrestart==0))||(dryrestart==1&&dry==1&&restart==1))
	{
	buffer=run3;
	if(manualoff==0)
	{
		split2();
		address=0xcc;//cd
		valuewrite2();}
	}
  }
}
/////////////////////////////////////////////////3rd page for 3 PH////////////////////////////////////////////////
if(displayx==3&&phase==3)
{
   buffer=rphvolt;
   split2();
   address=0x82;
   valuewrite2();
   buffer=yphvolt;
   split2();
   address=0x87;
   valuewrite2();
   buffer=bphvolt;
   split2();
   address=0x8c;
   valuewrite2();
}
///////////////////////////////////////////2nd page for 1 ph/////////////////////////////////////////////////////////
if((displayx==2)&&(phase==1))
{
  timedisplay=1;
  buffer=runningtime;
  split2();
  address=0x80;
  valuewrite2();
  timedisplay=0;
}
//////////////////////////1 ph page 1 and 2 data////////////////////////////////////////////////////////////////////
if(((((displayx==1)||displayx==2)&&(phase==1))&&(errordisplay==0))||(((displayx==1&&dryrestart==0)||(displayx==2))&&(phase==1)&&(errordisplay==1&&dryrestart==1&&autosetclrprocess==0&&autosetclrshow==0)))
{
	//if(((mode==2&&rctval1>7)||(mode==2&&rctval1<7))||(dryrestart==1&&dry==1&&restart==1&&mode==3))
	if(((mode==2&&oftimestart==0)||(mode==2&&oftimestart==1))||(dryrestart==1&&dry==1&&restart==1))
	{
		if(oftimestart==0&&wlctop1==0)
		{
			buffer=run2;
			if(manualoff==0)
			{
				split2();
				address=0xcc;//cd
				valuewrite2();
			}
		}
		//if(((rctval1<7&&oftshow==1)&&(mode==2)&&(oftimestart==1)&&(manualoff==0))||(dryrestart==1&&dry==1&&restart==1))
		if(((mode==2)&&(oftimestart==1)&&(manualoff==0)&&(dryrestart==0))||(dryrestart==1&&dry==1&&restart==1))
		{
			buffer=run3;
			if(manualoff==0)
			{
				split2();
				address=0xcc;//cd
				valuewrite2();
			}
	    }
	}
}
}
/////////////////////////////auto set/ auto clr switch time print section/////////////////////////////////
	if((aclrkey==1||asetkey==1)&&(autosetclrprocess==0&&switchtime!=0))
	{
		buffer=switchtime;
		split2();
		address=0xcb;
		valuewrite2();
		
	}
/*if(autosetclrshow==1&&modedisp==0)
{
  vshow=1;
	if(twophaseselected==0)
  buffer=hv;
	if(twophaseselected==1)
  buffer=hv2;
  split2();
  address=0x83;
  valuewrite2();
  vshow=0;

  amps=1;
	if(twophaseselected==0)
  buffer=ovl;
	if(twophaseselected==1)
  buffer=ovl2;
  split2();
  address=0x8b;
  valuewrite2();
  amps=0;

vshow=1;
	if(twophaseselected==0)
  buffer=lv;
	if(twophaseselected==1)
  buffer=lv2;
  split2();
  address=0xc3;
  valuewrite2();
  vshow=0;

  amps=2;
	if(twophaseselected==0)
  buffer=dryrun;
	if(twophaseselected==1)
  buffer=dryrun2;
  split2();
  address=0xcb;
  valuewrite2();
  amps=0;
}*/
}
void splitl(void)
{
	// this function converts every integer value in to spliting a single character based on 2 or 3 or 4 digit
    bufferx=buffer;
	
    if(errorlog==0&&errorprint==0)
    {
			if(((smenu==34||smenu==36)||(smenu==19||smenu==22))||((asetkey==1||aclrkey==1)&&(autosetclrprocess==0)))
			{
				digit1[1]=(bufferx/10);
				dbuf[1]=digit1[1]+0x30;
				bufferx=bufferx%10;
				digit1[2]=(bufferx);
				dbuf[2]=digit1[2]+0x30;
			}
    //if((smenu==1)||(smenu==2)||(smenu==3)||(smenu==12)||(smenu==15)||(smenu==16)||(smenu==17)||(smenu==18)||(autosetclrshow==0&&smode==0&&displayx==1&&amps!=1)||(autosetclrshow==0&&smode==0&&displayx==2&&timedisplay!=1)||(autosetclrshow==1&&(amps==1||amps==2||vshow==1)))
	if((smenu>=1&&smenu<=3)||(smenu==12)||(smenu==20||smenu==21||smenu==23)||(smenu>=15&&smenu<=18)||(smenu>=30&&smenu<=33)||(asetkey==0&&aclrkey==0&&autosetclrprocess==0&&autosetclrshow==0&&smode==0&&displayx==3&&amps!=1&&phase==3)||(asetkey==0&&aclrkey==0&&autosetclrprocess==0&&autosetclrshow==0&&smode==0&&displayx==1&&amps!=1)||(asetkey==0&&aclrkey==0&&autosetclrprocess==0&&autosetclrshow==0&&smode==0&&displayx==2&&timedisplay!=1))   
	{
		threedigit();
    }
   // if((smenu==4||smenu==5||smenu==6||smenu==7||smenu==8||smenu==9||smenu==10||smenu==11||smenu==24||(smode==0&&timedisplay==1&&timedisplay==1)))
	if((smenu>=4&&smenu<=11)||(smenu==24)||(smode==0&&timedisplay==1))   
	{
		fourdigit();
    }
		if(smenu==25||smenu==13)
		{
			if(smenu==25)
			{
				d1=runningdate;
				m1=runningmonth;
				y1=runningyear;
			}
			else
			{
				d1=refdate;
				m1=refmon;
				y1=refyear;
			}
			digit1[1] = d1/10;
			dbuf[1] = digit1[1]+0x30;
			digit1[2] =d1%10;
			dbuf[2] = digit1[2]+0x30;
			digit1[3] = m1/10;
			dbuf[3] = digit1[3]+0x30;
			digit1[4] = m1%10;
			dbuf[4] = digit1[4]+0x30;
			digit1[5] = y1/10;
			dbuf[5] = digit1[5]+0x30;
			digit1[6] = y1%10;
			dbuf[6]=digit1[6]+0x30;
		}
		if(amps==1&&autosetclrshow==0&&autosetclrprocess==0)
		{
			threedigita();
	/*digit1[1]=(bufferx/100);
	dbuf[1]=digit1[1]+0x30;
	bufferx=bufferx%100;
	digit1[2]=(bufferx/10);
	dbuf[2]=digit1[2]+0x30;
	dbuf[3]='.';
	digit1[3]=(bufferx%10);
	dbuf[4]=digit1[3]+0x30;*/
			//}
		}
	}
    if(errorlog==1&&errorprint==0)
    {
      if(twodigitdisp==1)
    {
      digit1[1]=(bufferx/10);
      dbuf[1]=digit1[1]+0x30;
      digit1[2]=(bufferx%10);
      dbuf[2]=digit1[2]+0x30; 
    }
    if(twodigitdisp==0)
    {
		 fourdigit();
		}
    }
    if(errorprint==1)
    {
      if(((volthigh==1||voltlow==1||phasefail==1)&&smode==0)||(errorlog==1&&(errordisplay1==1||errordisplay1==2||errordisplay1==6)))
      {
			threedigit();
/*digit1[1]=(bufferx/100);
dbuf[1]=digit1[1]+0x30;
bufferx=bufferx%100;
digit1[2]=(bufferx/10);
dbuf[2]=digit1[2]+0x30;
digit1[3]=(bufferx%10);
dbuf[3]=digit1[3]+0x30;*/
      }
      if((((ovld==1||dry==1)&&(vfault==0))&&(smode==0))||(errorlog==1&&(errordisplay1==4||errordisplay1==3)))
      {
				threedigita();
      
/*digit1[1]=(bufferx/100);
dbuf[1]=digit1[1]+0x30;
bufferx=bufferx%100;
digit1[2]=(bufferx/10);
dbuf[2]=digit1[2]+0x30;
dbuf[3]='.';
digit1[3]=(bufferx%10);
dbuf[4]=digit1[3]+0x30;
        */
      }
}
}

void threedigit(void)
{
	digit1[1]=(bufferx/100);
	dbuf[1]=digit1[1]+0x30;
	bufferx=bufferx%100;
	digit1[2]=(bufferx/10);
	dbuf[2]=digit1[2]+0x30;
	digit1[3]=(bufferx%10);
	dbuf[3]=digit1[3]+0x30;
}

void threedigita(void)
{
	digit1[1]=(bufferx/100);
	dbuf[1]=digit1[1]+0x30;
	bufferx=bufferx%100;
	digit1[2]=(bufferx/10);
	dbuf[2]=digit1[2]+0x30;
	dbuf[3]='.';
	digit1[3]=(bufferx%10);
	dbuf[4]=digit1[3]+0x30;
}

void fourdigit(void)
{
	digit1[1]=(bufferx/1000);
	dbuf[1]=digit1[1]+0x30;
	bufferx=bufferx%1000;
	digit1[2]=(bufferx/100);
	dbuf[2]=digit1[2]+0x30;
	bufferx=bufferx%100;
	digit1[3]=(bufferx/10);
	dbuf[3]=digit1[3]+0x30;
	digit1[4]=(bufferx%10);
	dbuf[4]=digit1[4]+0x30;
}

void split2(void)
{
	splitl();
	digit1[1]=dbuf[1]-0x30;
	digit1[2]=dbuf[2]-0x30;
	if(smenu!=34&&smenu!=23&&smenu!=35&&smenu!=19&&smenu!=36)
		digit1[3]=dbuf[3]-0x30;
	//if((smenu==4||smenu==5||smenu==6||smenu==7||smenu==8||smenu==9||smenu==10||smenu==11||smenu==13||smenu==24||smenu==25||timedisplay==1)&&(autosetclrshow==0))
	if(((smenu>=4&&smenu<=11)||(smenu==13)||(smenu==24)||(smenu==25)||(timedisplay==1))&&(autosetclrshow==0))
		digit1[4]=dbuf[4]-0x30; 
	if(smenu==25||smenu==13)
	{
		digit1[5]=dbuf[5]-0x30; 
		digit1[6]=dbuf[6]-0x30; 
	}
}
/*-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
								enable routine	
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void enabler(char t4)		
{
//PORTB=t4;
/*P0=t4;
enab=0;
for(r3=0;r3<100;r3++);
enab=1;*/
	//this function used for lcd command data send to the lcd
	for(x3=0;x3<100;x3++);
	x3=t4;
	dataout();
	enab=0;
	for(x3=0;x3<100;x3++);
	enab=1;
}

void dataout(void)
{// mux out put
	countlcd=0;
	while(countlcd<8)
	{
		dclock=0;
		x3=x3<<1;
		if(CARRY)
		{
			ddata=1;
			dclock=1;
		}
		if(!CARRY)
		{
			ddata=0;
			dclock=1;
		}
		countlcd++;
	}
	dclock=0;
	ddata=0;
}

void switchscan(void)
{
	/////////////button pressing identify function///////////////////////////////
	result1=0;
	resultx=0;
    //ENABLE1_ADC_CH15;
  	//channel=0x0f;
	YPHASEMUXDIS;/// we using to read switch input using mux 
	for(j=0;j<10;j++);
	ADCCON0=0x0A;//analog channel
    for(scan=0;scan<250;scan++)
	{
		clr_ADCCON0_ADCF;
		set_ADCCON0_ADCS;                  
		while(ADCF == 0);
		result1=(ADCRH<<4)+(ADCRL&0x0F);
		result1=result1>>2;
		resultx=result1+resultx;
	}
	CTMUXDIS;
    vref2=resultx/250;
    if(vref2>995)///if switch is not pressed it will execute
    {
		setpress=0;
		switchset4=1;
		switchset3=1;
		switchset2=1;
		switchset1=1;
		switchset5=1;
		switchset6=1;
		//
		menukey=0;
		modechangekey=0;
		sidekey=0;
		inckey=0;
		//
		menupress=0;
		asetpress=0;
		aclrpress=0;
		setpress=0;
		sidepress=0;
		incpress1=0;
		switchtime=5;
		ipress=0;
		modechangingcount=0;
//////////////////////////////to avoid auto set clr display glitch i use this loop//////////////////////
  		if((asetkey==1||aclrkey==1)&&(autosetclrprocess==0&&autosetclrstart==0&&autosetdone==0))
  		{
			autosetclrprocess=0;
			autoset=0;
			autoclr=0;
			autosetcountsubcount=0;
			autosetcount=0;
			autosetclrprocesssubcount=0;
			autosetclrprocessmain=0;
			autosetclrprocess=0;
			autosetclrstart=0;
			counter=0;
			displayx=1;
			refresh=1;
  		}
		asetkey=0;
		aclrkey=0;
	}
	if(asetkey==0&&aclrkey==0)
	{
		if(vref2<530&&vref2>460)//menu
		{
			modechangekey=0;
			sidekey=0;
			inckey=0;
			asetkey=0;
			aclrkey=0;
			menupress++;
			
			if((smode==1)&&(menupress>5)&&(switchset1==1))
			{
				menupress=0;
				switchset1=0;
				menukey=1;
			}
			sidepress=0;
			incpress1=0;
			setpress=0;
			asetpress=0;
			aclrpress=0;
			//
			if((smode==0)&&(menupress>1)&&(switchset1==1)&&(switchon==0)&&resetx==0)//for key pad led
			{
			//menupress=0;
			//switchset1=0;
			//menukey=1;
				if(modedisp==0&&modechangedisp==0)
				{
					switchon=1;
				}
			}
			if((smode==0&&scrolltype==2&&switchset1==1&menupress>2&&autosetclrprocess==0)&&((errordisplay==0)||(errordisplay==1&&dry==1&&dryrestart==1)))// this is for manual display scroll
			{
				menupress=0;
				switchset1=0;
				displayx=displayx+1;
				if(pumpsensed==0&&displayx==3&&phase==1)
					displayx=1;
				if(displayx>3)
					displayx=1;
				refresh=1;
				counter=0;
			}
			//
		}
		if(vref2>370&&vref2<410)//inckey- for set mode increment 
		{
			sidekey=0;
			menukey=0;
			asetkey=0;
			aclrkey=0;
			setkey=0;
			modechangekey=0;
			incpress1++;
			if((smode==1)&&(incpress1>=5)&&(switchset2==1))
			{
				incpress1=0;switchset2=0;
				inckey=1;
			}
			if(((start2==1||restart==1)&&(phasereversed==0))&&(incpress1>20&&switchset2==1&&asetkey==0&&aclrkey==0))// this loop for error reset 
			{
				incpress1=0;
				switchset2=0;
				resetx=1;
				resetcount=200;
				refresh=1;
			}
			sidepress=0;
			menupress=0;
			setpress=0;
			asetpress=0;
			aclrpress=0;
		}
		if(vref2>630&&vref2<710)//mode key for mode selection 
		{
			inckey=0;
			menukey=0;
			asetkey=0;
			aclrkey=0;
			sidekey=0;
			setkey=0;
			sidepress++;
			if((sidepress>=5))
			{
				sidepress=0;
				if(smode==0&&switchset3==1&&resetx==0&&modechangedisp==0&&modedisp==0&&autosetclrprocess==0)//doubt
				{
					modechangekey=1;
					switchset3=0;
				}
			}
			if(smode==0&&switchset3==1&&modechangedisp==1&&resetx==0)
			{
				switchset3=0;
				mode=mode+1;
				if(mode>4)
					mode=2;
				nokeys=0;
				nokeysubcount=0;
				if(switchon==1)
					switchon=0;

				if(mode==4&&rtcstart==1)
						rtcstart=0;

				if(mode==2&&oftimestart==1)
				{
					oftimestart=0;
					oftimestarterror=1;
				}
					
					reset1=0;
					start4=0;
					ondelaycheck=0;
					Nop=0;
					if(rctval>=10)
						relayoff=1;
					
			refresh=1;
		}
			menupress=0;
			incpress1=0;
			setpress=0;
			asetpress=0;
		aclrpress=0;
		}
if(vref2>930&&vref2<990)//run/setmode
{
	inckey=0;
    menukey=0;
    asetkey=0;
	aclrkey=0;
	sidekey=0;
	modechangekey=0;
	setpress++;
	if(smode==0&&errorlog==0&&switchset4==1&&autosetclrprocess==0&&setpress>30&&modedisp==0&&resetx==0&&modechangedisp==0)
    {
			setpress=0;
			switchset4=0;
			smode=1;
			smenu=1;
    }
	if(smode==1&&switchset4==1&&setpress>100&&modedisp==0&&adtransfer==0)
	{
		smode=0;
		smenu=0;
		displayx=1;
		counter=0;
		refresh=1;
		setpress=0;
		switchset4=0;
		errorlog=0;
		edit=0;
		errt1=0;
		errortime=0;
		errortime1=0;
		errt2=0;
		sense=0;
		stoprlyoffcount=0;
		//adctime=0;
		ondelaycheck=0;
		start4=0;
	}
	menupress=0;
	incpress1=0;
	sidepress=0;
    asetpress=0;
    aclrpress=0;		
    }
}
if(vref2>790&&vref2<860)//autosetkey
{
	inckey=0;
    menukey=0;
	aclrkey=0;
	sidekey=0;
	modechangekey=0;
	aclrpress=0;
	if(autoclr==0)
		asetpress++;
	if(smode==0&&rctval1>=10&&asetpress>5&&switchset5==1&&resetx==0&&autosetclrprocess==0&&autosetdone==0&&autosetclrshow==0&&modedisp==0&&modechangedisp==0)
    {
		 asetpress=0;
    	 asetkey=1;
		 switchset5=0;
		 counter=0;
		 displayx=1;
		 refresh=1;
    }
	menupress=0;
	incpress1=0;
	sidepress=0;	
	setpress=0;
}
if(vref2>250&&vref2<350)//autoclr key 
{
	inckey=0;
    menukey=0;
	asetkey=0;
	//sidekey=0;
	modechangekey=0;
  	if(autoset==0)
		aclrpress++;
    if((switchset6==1)&&(smenu>=1&&smenu<=36)&&(smode==1)&&(aclrpress>5))
    {
       sidekey=1;
       switchset6=0;
			 aclrpress=0;
    }
	if(smode==0&&rctval1<10&&aclrpress>9&&switchset6==1&&autosetclrprocess==0&&autosetdone==0&&autosetclrshow==0&&modedisp==0&&modechangedisp==0)
    {
		aclrpress=0;
        aclrkey=1;
		switchset6=0;
		counter=0;
		displayx=1;
		refresh=1;
    }
	menupress=0;
	incpress1=0;
	sidepress=0;	
	setpress=0;
	asetpress=0;
 }
}


void display(void)
{
	///////////entire run mode and error display section comes down this function///////////////////////////
	lcdinit();// every time we are re initializing due to avoid unwanted character printing 
    fline();//1st line
	if(resetx==0&&modechangedisp==0)// when there is no mode change and error reset function happen then only it will 
	{
		for(t3=0;t3<16;t3++)
		{
			if(autosetclrprocess==0&&autosetclrshow==0&&modedisp==0&&autosetdone==0&&asetkey==0&&aclrkey==0)
			{
					if((displayx==1||displayx==3)&&(phase==3))
							temp=vin[t3];
					if(((displayx==2)&&(phase==3))||(displayx==2&&phase==1))
							temp=pt[t3];
					if((displayx==1)&&(phase==1))
							temp=va[t3];
					if(displayx==3&&phase==1)
							temp=pumpt[t3];
			}
			if((autosetclrprocess==1&&autosetclrshow==0&&modedisp==0)||((asetkey==1||aclrkey==1)&&(autosetclrprocess==0)))
			{
				if((autoset==1)||(asetkey==1&&autosetclrprocess==0))
					temp=autos[t3];
				if((autoclr==1)||(aclrkey==1&&autosetclrprocess==0))
					temp=autoc[t3];
			}
//if(autosetclrshow==1&&modedisp==0)
  //  temp=acshowr1[t3];
  
if(modedisp==1)
{
	if(mode==1)
		temp=man[t3];
	if(mode==2)
		temp=cyc[t3];
	if(mode==3)
		temp=aut[t3];
	if(mode==4)
		temp=rrtc[t3];
} 
enabler(temp);
delay(1);
}
chip=0;
enabler(0x8c);
chip=1;
for(t3=0;t3<3;t3++)
{
	if((((displayx==2)&&(phase==3))||((displayx==1||displayx==2)&&(phase==1)))&&(asetkey==0&&aclrkey==0&&autosetdone==0&&autosetclrprocess==0&&autosetclrshow==0&&modedisp==0))
	{
		if(rctval1>=10)
			temp=ron[t3];
		if(rctval1<10)
			temp=rof[t3];
		enabler(temp);
		delay(1);
	}
}
sline();//2nd line
////////////////if any error happen then only this loop will execute///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
if(((errordisplay==1&&dryrestart==0&&asetkey==0&&aclrkey==0&&autosetclrprocess==0&&autosetclrshow==0&&modedisp==0)||(modedisp==1))||(errordisplay==1&&dryrestart==1&&displayx==1&&asetkey==0&&aclrkey==0&&autosetclrprocess==0))
{
  for(t3=0;t3<16;t3++)
  {
    temp=' ';
   if(modedisp==0)
   { 
		if(volthigh==1)
			temp=errorhv[t3];
		if(voltlow==1)
			temp=errorlv[t3];
		if(ovld==1&&vfault==0&&sumpdry==0)
			temp=errorovl[t3];
		if(dry==1&&vfault==0&&sumpdry==0)
			temp=errordry[t3];
		if(vfault==0&&unbalancecurrent==1)
			temp=unbc[t3];
		if(phaseunbalance==1&&phase==3)
			temp=errorph[t3];
		if(phasefail==1&&phase==3)
			temp=errorpf[t3];
		if(phasereversed==1&&phase==3)
			temp=errorpr[t3];
		if(sumpdry==1&&vfault==0&&start2==0)
			temp=sumpemty[t3];
		if(nopump==1&&start2==0&&sumpdry==0)
			temp=no_pump[t3];
   }
  if(modedisp==1)
  	temp=select[t3];
  enabler(temp);
  delay(1);
}
	
	if(phase==1)
	{
		if(volthigh==1||voltlow==1)
		   ulimit=3;
		if((ovld==1||dry==1)&&(vfault==0))
		    ulimit=0;
		sline();
		for(t3=0;t3<ulimit;t3++)
		{
				enabler(nu);
				delay(1);
		}
	}
if(errordisplay==1&&modedisp==0)
{
    chip=0;
	enabler(0xc0);
    chip=1;
    if(volthigh==1||voltlow==1||phasefail==1)
    {
if((ovld==0&&dry==0&&phasefail==0)||(vfault==1&&phasefail==0))
{
if(phase==3)
{
//if(phasenumber==1&&phase==3)
if(phasenumber==1)
{
enabler(cr);
enabler(cy);
delay(1);
}
//if(phasenumber==2&&phase==3)
if(phasenumber==2)
{
enabler(cy);
enabler(cb);
delay(1);
}
//if(phasenumber==3&&phase==3)
if(phasenumber==3)
{
enabler(cb);
enabler(cr);
delay(1);
}}
  }
if(phasefail==1&&phase==3)
{
//if(phasenumber==1&&phasefail==1&&phase==3)
	if(phasenumber==1)
enabler(cr);

//if(phasenumber==2&&phasefail==1&&phase==3)
	if(phasenumber==2)
enabler(cy);

//if(phasenumber==3&&phasefail==1&&phase==3)
	if(phasenumber==3)
enabler(cb);
delay(1);
}
errorprint=1;
buffer=errorvalue;
//buffer=dispry;
  split2();
  if(volthigh==1||voltlow==1)
  address=0xcb;
  if(phasefail==1)
  address=0xcc;
errordisplayprint();
errorprint=0;
  }
  if((ovld==1||dry==1)&&(vfault==0)&&(sumpdry==0))
  {
    //if(phasenumber==1&&phase==3)
    //enabler(cr);

    //if(phasenumber==2&&phase==3)
    //enabler(cy);

    //if(phasenumber==3&&phase==3)
   // enabler(cb);
errorprint=1;
buffer=errorvalue1;
//buffer=dispry;
  split2();
  if(dry==1)
  address=0xc9;
  if(ovld==1)
  address=0xca;
errordisplayprint();
errorprint=0;
  }
}
}
////////////////////////////////////////////////error printing section end here//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////auto set / auto clear display print section///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
if((autosetclrprocess&&modedisp==0)||((asetkey==1||aclrkey==1)&&(autosetclrprocess==0)))
{ 
  for(t3=0;t3<16;t3++)
  { 
		if((asetkey==1||aclrkey==1)&&(autosetclrprocess==0))
			temp=asetclr[t3];
  if((autoset==1||autoclr==1)&&(autosetdone==0&&autosetclrshow==0))
    temp=proce[t3];
  if(autosetdone==1)
   temp=autocom[t3];

  //if(autosetclrshow==1)
  //temp=acshowr2[t3];

    enabler(temp);
    delay(1);
   }
}
//////////////////////////////////////end of auto set/ clr function priniting section////////////////////////////////////////////////////////
//////////////////all second line normal run mode dispaly printing here///////////////////////////////////////////////////////////////////////////////////////
if((errordisplay==0&&autosetclrprocess==0&&autosetdone==0&&autosetclrshow==0&&modedisp==0&&asetkey==0&&aclrkey==0)||(asetkey==0&&aclrkey==0&&errordisplay==1&&dry==1&&dryrestart==1&&autosetclrprocess==0&&autosetdone==0&&autosetclrshow==0&&modedisp==0&&displayx!=1))
{
for(t3=0;t3<16;t3++)
{
  if((displayx==1)&&(phase==3))
   temp=ai[t3];
	
	if((displayx==3)&&(phase==3))
   temp=pn[t3];
	
  if(displayx==1&&phase==1)
   temp=am[t3];

  if(((displayx==2)&&(phase==3))||((displayx==2&&phase==1)))
  {
    if(mode==1)
   temp=mod[t3];
   if(mode==2)
   temp=mod1[t3];
   if(mode==3)
   temp=mod2[t3];
   if(mode==4)
   temp=mod3[t3];
   }
   if(displayx==3&&phase==1)
   {
    if(pumptype==1)
    temp=pump1[t3];

    if(pumptype==2)
    temp=pump2[t3];
   }

enabler(temp);
delay(1);
}
if(phase==3&&displayx==3)
{
	chip=0;
	enabler(0xcc);
	chip=1;
	if(twophaseselected==1)
		temp='2';
	if(twophaseselected==0)
		temp='3';
	
enabler(temp);
delay(1);	
}
chip=0;
if((mode!=4)||(mode==4&&dryrestart==1))
enabler(0xc7);
if(mode==4&&dryrestart==0)
enabler(0xc8);
chip=1;
if(((displayx==2)&&(phase==3))||(((displayx==1||displayx==2)&&(phase==1))))
//if((displayx==2)&&(phase==3))
{
	if(mode==4&&dryrestart==0)
        limitx=4;
	
  if((manualoff==0&&mode==2)||(mode==3)||(dryrestart==1&&mode==4))
           limitx=5;
  //if(manualoff==1&&mode==2)
  //limitx=7;
	
for(t3=0;t3<limitx;t3++)
{
    temp=' ';
if((mode==2)&&(oftimestart==0)&&(dryrestart==0)&&(wlctop1==0))
{
      temp=ond[t3];
	
	
}

if((mode==2)&&(manualoff==0)&&(oftimestart==1)&&(dryrestart==0))
{
           temp=ofd[t3];
	
}
//if(((mode==2)&&(rctval1<7)&&(manualoff==1)))
        //temp=mof[t3];
//if(mode==3&&rctval1<7&&restart==1&&dry==1&&dryrestart==1)
if(restart==1&&dry==1&&dryrestart==1)
     temp=drystart[t3];

if((mode==4)&&(timeron==1||timeron==3||timeron==5||timeron==7)&&(dryrestart==0))
 temp=sc1[t3];
 if((mode==4)&&(timeron==2||timeron==4||timeron==6||timeron==8)&&(dryrestart==0))
 temp=sc2[t3];
 
 
enabler(temp);
delay(1);
}
}
if(((displayx==2)&&(phase==3))||(((displayx==1||displayx==2)&&(phase==1))))
{
	if(mode==4&&dryrestart==0&&timeron!=0)
{
chip=0;
enabler(0xc9);
	delay(1);
	
	
	chip=1;
	if(timeron==1||timeron==2)
	enabler('1');
	if(timeron==3||timeron==4)
	enabler('2');
	if(timeron==5||timeron==6)
	enabler('3');
	if(timeron==7||timeron==8)
	enabler('4');
   delay(1);
}
}
}
////////////////////////////////////2nd line run mode display print end here///////////////////////////////////////////////////////////
}
////////////////////////////////////fault reset display printing section////////////////////////////////////////
	if(resetx==1&&modechangedisp==0)
	{
		for(t3=0;t3<16;t3++)
		{
			temp=rst[t3];
			enabler(temp);
       delay(1);
		}
		sline();
		for(t3=0;t3<16;t3++)
		{
			temp=autocom[t3];
			enabler(temp);
       delay(1);
		}
	}
	////////////////////////fault reset print section end here/////////////////////////////////////////////
	//////////////////////when you enter in the mode selection then only it will execute/////////////////////
	if(resetx==0&&modechangedisp==1)
	{
		fline();
	for(t3=0;t3<16;t3++)
		{
			temp=modesel[t3];
			enabler(temp);
delay(1);
		}
		sline();
		for(t3=0;t3<16;t3++)
		{
			if(mode==1)
			temp=modex[t3];
			if(mode==2)
			temp=mode1x[t3];
			if(mode==3)
			temp=mode2x[t3];
			if(mode==4)
			temp=mode3x[t3];
			
			enabler(temp);
delay(1);
		}
	}
	//////////////////////////////////////end of mode display//////////////////////////////////////////////////////////////
chip=0;
enabler(0x0c);
delay(0x0a);
}
void display2(void)
{
	//////////////////////////////////this function for set mode and error log display priniting ///////////////////////////
fline();//1st line
chip=1;
for(h=0;h<16;h++)
{//this loop print 1st line strings////////////////////////////////////////////////////////////
if((smenu!=27)&&(smenu!=28)&&(smenu!=30&&smenu!=31&&smenu!=32&&smenu!=33&&smenu!=34&&adtransfer==0))
  temp=fline1[h];
	if(smenu==27)
	temp=erlog1[h];
	if(smenu==28)
	temp=crlog1[h];
	if(smenu>=30&&smenu<=34)
	temp=twoset[h];
	//if(adtransfer==1)
		//temp=wlcadtrans[h];
enabler(temp);
delay(1);
}
sline();//2nd line 
for(h=0;h<16;h++)
{
if(smenu==1)
{
temp=ONT[h];
buffer=ontime;
	
}
if(smenu==2)
{
  temp=OFT[h];
buffer=offtime;
}
if(smenu==3)
{
  temp=drtd[h];
  buffer=drt;
}
if(smenu==4)
{
  temp=t1[h];
  buffer=onrtcvalue1;
}
if(smenu==5)
{
temp=t1O[h];
  buffer=offrtcvalue1;
}
if(smenu==6)
{
	temp=t1[h];
  buffer=onrtcvalue2;
}
if(smenu==7)
{
	temp=t1O[h];
buffer=offrtcvalue2;
}
if(smenu==8)
{
	temp=t1[h];
  buffer=onrtcvalue3;	
}
if(smenu==9)
{
	temp=t1O[h];
buffer=offrtcvalue3;
}
if(smenu==10)
{
	temp=t1[h];
  buffer=onrtcvalue4;	
}
if(smenu==11)
{
	temp=t1O[h];
buffer=offrtcvalue4;
}

if(smenu==12)
{
temp=day[h];
buffer=dayskip;
}
if(smenu==13)
{
temp=rd[h];
//bufferr=refday;
}
if(smenu==14&&edit==2)
{
	temp=pset[h];
}
if(smenu==14&&edit==1)
{
	temp=pset1[h];
}
if(smenu==15||smenu==30)
{
  temp=vh[h];
	if(smenu==15)
    buffer=hv;
	if(smenu==30)
		buffer=hv2;
		
}
if(smenu==16||smenu==31)
{
  temp=vl[h];
	if(smenu==16)
  buffer=lv;
	if(smenu==31)
		buffer=lv2;
}
if(smenu==17||smenu==32)
{
  temp=lo[h];
	if(smenu==17)
  buffer=ovl;
	if(smenu==32)
		buffer=ovl2;
}

if(smenu==18||smenu==33)
{
  temp=dr[h];
	if(smenu==18)
  buffer=dryrun;
	if(smenu==33)
  buffer=dryrun2;
}
if(smenu==19)
{
	temp=ovlcut[h];
	buffer=ovlcut1;
	
}

if(smenu==20)
{
	temp=drycut[h];
	buffer=drycut1;
}
if(smenu==21)
{
	temp=ubvolt[h];
	buffer=ubvolt1;
	
}
if(smenu==22)
{
	temp=ubcur[h];
	buffer=ubcur1;
}


if(smenu==23)
{
	temp=Ondly[h];
	buffer=ondelay;
}
if((smenu==24))
{
  temp=ti[h];
  buffer=rtctime;
}
if((smenu==25))
{
  temp=dm[h];
  //bufferr=rtcdate;
}
if(smenu==26)
{
	temp=scr[h];
}
if(smenu==27)//errorlog
{
	temp=erlog2[h];
	
	errtinc=errt;
	errtinc1=errorsave1;
}
if(smenu==28)
{
	temp=crlog2[h];
}
if(smenu==29&&twophase==1)
{
	temp=twophe[h];
}
if(smenu==29&&twophase==2)
{
	temp=twophd[h];
}
if(smenu==34)
{
	temp=capcut[h];
	buffer=capcuttime;
}
if(scrolltype==1&&smenu==26)
{
	temp=scra[h];
}
if(scrolltype==2&&smenu==26)
{
	temp=scrm[h];
}
if(smenu==35)
{
	if(stardelta==1)
		temp=strdle[h];
		if(stardelta==2)
			temp=strdld[h];
			
}
if(smenu==36)
{
	temp=strdlt[h];
	buffer=starondelay;
}
/*if(smenu==35&&adtransfer==0)
{
	temp=wlcad[h];
	buffer=wlcaddress;
}
if(adtransfer==1&&reception_ok==0)
{
	temp=proce[h];
}
if(adtransfer==1&&reception_ok==1)
{
	if(newid_ok==1)
	temp=autocom[h];
	if(newid_ok==0)
		temp=autofail[h];
	
}*/
enabler(temp);
delay(1);
}
chip=0;
enabler(0xc3);
delay(1);
chip=1;
if(smenu==4||smenu==5)
enabler('1');
if(smenu==6||smenu==7)
enabler('2');
if(smenu==8||smenu==9)
enabler('3');
if(smenu==10||smenu==11)
enabler('4');
delay(1);

//if((smenu==4)||(smenu==5)||smenu==6||smenu==7||smenu==8||smenu==9||smenu==10||smenu==11||(smenu==24))
if((smenu>=4&&smenu<=11)||(smenu==24))
{
  split2();
  chip=0;
   address=0xc8;
//enabler(0xc8);
valuewrite();
}
if(smenu==25||smenu==13)
{
  split2();
  chip=0;
   address=0xc7;
//enabler(0xc8);
valuewrite(); 
}
if(smenu==12||smenu==23||smenu==1||smenu==2||smenu==21||smenu==22)
{
  split2();
  chip=0;
   address=0xcb;
//enabler(0xcb);
valuewrite();
}
if((smenu==15)||(smenu==16)||(smenu==30)||(smenu==31))
{
split2();
chip=0;
address=0xca;
//enabler(0xca);
valuewrite();
}
if(smenu==34||smenu==36)
{
  split2();
  chip=0;
   address=0xcd;
//enabler(0xcb);
valuewrite();
}
if(smenu==19||smenu==20)
{
  split2();
  chip=0;
   address=0xcc;
//enabler(0xcc);
valuewrite();
}

if((smenu==17||smenu==18||smenu==32||smenu==33||smenu==3))
{
  split2();
  chip=0;
  address=0xc9;
//enabler(0xc9);
valuewrite();
}
chip=0;
enabler(0x0c);
delay(0x0a);
}
void editmode(void)
{
	// this function perform in set mode like menu increment and eeprom save and menu skipping 
 switchscan();
if(menukey==0)
  mpress=0;
if((menukey==1)&&(mpress==0))
{
	///eeprom storing start here
  mpress=1;
  if(write==1)
  {
    combin();//coimbining whole value char to interger 1,2,3 =123
    write=0;
    
     if(smenu==1)
     {
      ontime=buffer;
      run2=ontime;
			oftimestart=0;
      
			oftimestarterror=1;
			
     }
     if(smenu==2)
     {
        offtime=buffer;
        run3=offtime;
     }
     if(smenu==3)
     {
      drt=buffer;
			 drt1=drt;
			 drtstore=1;
     }
     if(smenu==4)
     {
      h1on=bufferh;
      m1on=bufferl;
     }
     if(smenu==5)
     {
      h1off=bufferh;
      m1off=bufferl;
     }
  if(smenu==6)
  {
     h2on=bufferh;
      m2on=bufferl;
  }
  if(smenu==7)
  {
      h2off=bufferh;
      m2off=bufferl;
  }
	if(smenu==8)
  {
     h3on=bufferh;
      m3on=bufferl;
  }
	if(smenu==9)
  {
      h3off=bufferh;
      m3off=bufferl;
  }
	if(smenu==10)
  {
     h4on=bufferh;
      m4on=bufferl;
  }
	if(smenu==11)
  {
      h4off=bufferh;
      m4off=bufferl;
  }
	if(smenu==12)
  {
    dayskip=buffer;
    if(dayskip>999)
    dayskip=0;
		dayskiph=dayskip/256;
		dayskipl=dayskip%256;
  }
	if(smenu==13)
	{
		if(refdate>31)
    {
    refdate=31;
		
    digit1[1]=refdate/10;
    digit1[2]=refdate%10;
    }
    if(refdate<1)
    {
    refdate=1;
    day1=refdate;
    digit1[1]=refdate/10;
    digit1[2]=refdate%10;
    }
    if(refmon>12)
    {
    refmon=12;
			month1=refmon;
    digit1[3]=refmon/10;
    digit1[4]=refmon%10;
    }
    if(refmon<1)
    {
     refmon=1;
			month1=refmon;
     digit1[3]=refmon/10;
     digit1[4]=refmon%10;}
		
		 year1=(20*100)+refyear;
	
	}
  if(smenu==15)
  {
  hv=buffer;
  }
if(smenu==16)
  {
    lv=buffer;
  }
  if(smenu==17)
  {
    ovl=buffer;
  }
  if(smenu==18)
  {
    dryrun=buffer;
  }
	if(smenu==19)
	{
		ovlcut1=buffer;
	}
	if(smenu==20)
	{
		drycut1=buffer;
	}
	if(smenu==21)
	{
		ubvolt1=buffer;
	}
	if(smenu==22)
	{
		ubcur1=buffer;
	}
	if(smenu==23)
	{
		ondelay=buffer;
	}
  if(smenu==24)
  {
    //write rct time here bufferh for hour , bufferl for min 
    if(bufferh>23)
    bufferh=23;
    if(bufferl>59)
    bufferl=59;
		
     rtcmin=(((bufferl/10)*0x10)|(bufferl%10));
		 rtchour=(((bufferh/10)*0x10)|(bufferh%10));
     rtccredit=1;
     rtctime=((bufferh*100)+(bufferl));
  }
  if(smenu==25)
  {
    if(date>31)
    {
    date=31;
    digit1[1]=date/10;
    digit1[2]=date%10;
    }
    if(date<1)
    {
    date=1;
    digit1[1]=date/10;
    digit1[2]=date%10;
    }
    if(mon>12)
    {
    mon=12;
    digit1[3]=mon/10;
    digit1[4]=mon%10;
    }
    if(mon<1)
    {
     mon=1;
     digit1[3]=mon/10;
     digit1[4]=mon%10;}
  //rtcdate=((digit1[1]*100000)+(digit1[2]*10000)+(digit1[3]*1000)+(digit1[4]*100)+(digit1[5]*10+digit1[6]));
  rtcdate1=(((date/10)*0x10)|(date%10));
  rtcmonth=(((mon/10)*0x10)|(mon%10));
  rtcyear=(((yr/10)*0x10)|(yr%10));
  rtccredit=2;
  }
	/*if(smenu==26)
	{
		//memsave==1;
	}*/
	if(smenu==30)
	{
		hv2=buffer;
	}
	if(smenu==31)
	{
		lv2=buffer;
	}
	if(smenu==32)
	{
		ovl2=buffer;
	}
	if(smenu==33)
	{
		dryrun2=buffer;
	}
	if(smenu==34)
	{
		capcuttime=buffer;
	}
	if(smenu==36)
	{
		starondelay=buffer;
	}
	if(smenu!=24&&smenu!=25)
	    memsave=1;
		// eeprom store end here
  }
while(memsave==1);
cblink=0;
blink=0;
chang=0;
datachang=0;
digit1[1]=0;
digit1[2]=0;
digit1[3]=0;
digit1[4]=0;
digit1[5]=0;
digit1[6]=0;
	setreset=0;
//fline();
	//
	errorlog=0;
	errt1=0;
	errortime=0;
	errortime1=0;
	errt2=0;
	//
if(adtransfer==0)
  smenu++;// menu incrementation
if(smenu==30&&twophase!=1)
{
smenu=35;//1
}
if(smenu==35&&twophase==1)
{
	smenu=1;
}
if(smenu==15&&edit!=2)
{
	smenu=1;
}
if(smenu==36&&stardelta!=1)
{
	smenu=1;
}

if(smenu>36)
	smenu=1;

if(errt==0&&smenu==28)
	smenu=29;
if(phase==1&&smenu==21)
	smenu=23;
if(smenu==29&&phase==1)
	smenu=1;
	 //smenu=1;
//if(mode==1&&smenu<14)
//smenu=14;
if(mode==2&&smenu==4)
smenu=14;
if(mode==3&&smenu<3)
 smenu=3;
 if(mode==3&&smenu==4)
 smenu=14;
 if(mode==4&&smenu<3)
 smenu=3;
 if(mode==4&&smenu==13&&dayskip<=0)
	 smenu=14;
 if(smenu==15)
	 timeseg=1;
	 
   display2();// displaying the corresponding menu and it's parameter
}
if(smenu==26||smenu==14||smenu==29||smenu==35)
{
	/// this menu like parameter enable and disable, yes or no this kind of settings only show here 
if(smenu==26)
{
	if(inckey==0)
		dscroll=0;
	if(inckey==1&&dscroll==0)
	{
		dscroll=1;
		scrolltype=scrolltype+1;
		if(scrolltype>2)
			scrolltype=1;
		
		IE&=~(1<<7);
		Write_DATAFLASH_BYTE1(61039,scrolltype);
		IE|=(1<<7);
		//display2();
		showdisp=1;
	}
}
if(smenu==29)
{
	if(inckey==0)
		dscroll=0;
	if(inckey==1&&dscroll==0)
	{
		dscroll=1;
		twophase=twophase+1;
		if(twophase>2)
			twophase=1;
		IE&=~(1<<7);
		Write_DATAFLASH_BYTE1(61046,twophase);
		IE|=(1<<7);
		if(twophase==1)
		{
			stardelta=2;
			IE&=~(1<<7);
			Write_DATAFLASH_BYTE1(61070,stardelta);
			IE|=(1<<7);
		}
		
		showdisp=1;
	//	display2();
	}
}
	if(smenu==14)
	{
		if(inckey==0)
		dscroll=0;
	if(inckey==1&&dscroll==0)
	{
		dscroll=1;
		edit=edit+1;
		if(edit>2)
			edit=1;
		showdisp=1;
	}
	}
	if(smenu==35)
	{
		if(inckey==0)
		dscroll=0;
		if(inckey==1&&dscroll==0)
		{
			dscroll=1;
			stardelta=stardelta+1;
			if(stardelta>2)
				stardelta=1;
			showdisp=1;
			write=1;
		}
		
	}
	if(showdisp==1)// to avoid multiple time re-enter in this display function, i have use this flag
	{
		showdisp=0;
		display2();
	}
	
}
/////////////////// this is for lora pairing///////////////////////////
/**************Address Storing********************/
	/*address_store=0;
newid_ok=0;
while(adtransfer)
{
	if(address_store==0){
		loraTxinit();
			memset(newid, 0, 10);
		custom_sprintf_ctxaddr(newid,wlcaddress,"NEWID:");
		loraTxdata(NEWID);
		loraRxinit();
		ack_receive_enable=1;
		address_store=1;
}
if(reception_ok==1)
{
	display2();
	delay(200);
	reception_ok=0;
	adtransfer=0;
	smenu=32;
}	
}	*/

}

void datawrite(void)
{
	///////////////////////eeprom write function//////////////////////////////////////
	IE&=~(1<<7);
	if(drt<=0)
		drt=0;
	if(drt>999)
		drt=999;
	
      drth=drt/256;
      drtl=drt%256;
      Write_DATAFLASH_BYTE1(61004,drth);
      Write_DATAFLASH_BYTE1(61005,drtl);

       Write_DATAFLASH_BYTE1(61001,mode);

        ontimeh=ontime/256;
	      ontimel=ontime%256;
	      Write_DATAFLASH_BYTE1(61006,ontimeh);
				Write_DATAFLASH_BYTE1(61007,ontimel);

        
        run2h=run2/256;
	      run2l=run2%256;
	      Write_DATAFLASH_BYTE1(61008,run2h);
	      Write_DATAFLASH_BYTE1(61009,run2l);

        offtimeh=offtime/256;
	      offtimel=offtime%256;
				
	      Write_DATAFLASH_BYTE1(61010,offtimeh);
				Write_DATAFLASH_BYTE1(61011,offtimel);

        run3h=run3/256;
	      run3l=run3%256;
	      Write_DATAFLASH_BYTE1(61014,run3h);
	      Write_DATAFLASH_BYTE1(61015,run3l);


        if(h1on>23)
          h1on=23;
       if(m1on>59)
         m1on=59;
	      Write_DATAFLASH_BYTE1(61017,h1on);
	      Write_DATAFLASH_BYTE1(61018,m1on);
        onrtcvalue1=((h1on*100)+m1on);


        if(h1off>23)
      h1off=23;
       if(m1off>59)
       m1off=59;
	      Write_DATAFLASH_BYTE1(61019,h1off);
	      Write_DATAFLASH_BYTE1(61020,m1off);
        offrtcvalue1=((h1off*100)+(m1off));

        if(h2on>23)
         h2on=23;
       if(m2on>59)
       m2on=59;
	      Write_DATAFLASH_BYTE1(61021,h2on);
	      Write_DATAFLASH_BYTE1(61022,m2on);
        onrtcvalue2=((h2on*100)+(m2on));

        if(h2off>23)
      h2off=23;
       if(m2off>59)
       m2off=59;
	      Write_DATAFLASH_BYTE1(61023,h2off);
	      Write_DATAFLASH_BYTE1(61024,m2off);
        offrtcvalue2=((h2off*100)+(m2off));
			 
			 
			   if(h3on>23)
         h3on=23;
         if(m3on>59)
         m3on=59;
	      Write_DATAFLASH_BYTE1(61025,h3on);
	      Write_DATAFLASH_BYTE1(61026,m3on);
        onrtcvalue3=((h3on*100)+(m3on));
				 
				  if(h3off>23)
      h3off=23;
       if(m3off>59)
       m3off=59;
	      Write_DATAFLASH_BYTE1(61027,h3off);
	      Write_DATAFLASH_BYTE1(61028,m3off);
        offrtcvalue3=((h3off*100)+(m3off));
			 
			  if(h4on>23)
         h4on=23;
         if(m4on>59)
         m4on=59;
	      Write_DATAFLASH_BYTE1(61029,h4on);
	      Write_DATAFLASH_BYTE1(61030,m4on);
        onrtcvalue4=((h4on*100)+(m4on));
				 
			if(h4off>23)
      h4off=23;
       if(m4off>59)
       m4off=59;
	      Write_DATAFLASH_BYTE1(61031,h4off);
	      Write_DATAFLASH_BYTE1(61032,m4off);
        offrtcvalue4=((h4off*100)+(m4off));

   if(hv>320&&phase==1)
		hv=320;
	if(hv<231&&phase==1)
		hv=231;
    if(hv>600&&phase==3)
		hv=600;
	if(hv<400&&phase==3)
		hv=400;
hvh=hv/256;
hvl=hv%256;
Write_DATAFLASH_BYTE1(61033,hvh);
Write_DATAFLASH_BYTE1(61034,hvl);


if(lv>230&&phase==1)
		lv=230;
	if(lv<150&&phase==1)
		lv=150;
    if(lv>400&&phase==3)
      lv=400;
    if(lv<280&&phase==3)
      lv=280;
lvh=lv/256;
lvl=lv%256;
Write_DATAFLASH_BYTE1(61035,lvh);
Write_DATAFLASH_BYTE1(61036,lvl);
		
		
		
		
if(ovl>200&&phase==1)
		ovl=200;
	if(ovl<10&&(phase==1||phase==3))
		ovl=10;
    if(ovl>500&&phase==3)
		  ovl=500;
ovlh=ovl/256;
ovll=ovl%256;
    Write_DATAFLASH_BYTE1(61037,ovlh);
    Write_DATAFLASH_BYTE1(61038,ovll);

    if(dryrun>((ovl*9)/10))
		  dryrun=10;
	  if(dryrun<10)
		dryrun=((ovl*9)/10);
    dryrunh=dryrun/256;
    dryrunl=dryrun%256;
    Write_DATAFLASH_BYTE1(61012,dryrunh);
    Write_DATAFLASH_BYTE1(61013,dryrunl);

   
		
		Write_DATAFLASH_BYTE1(61039,scrolltype);
		
		Write_DATAFLASH_BYTE1(61040,refdate);
		Write_DATAFLASH_BYTE1(61041,refmon);
		Write_DATAFLASH_BYTE1(61042,refyear);
		
		
		Write_DATAFLASH_BYTE1(61043,dayskiph);
		Write_DATAFLASH_BYTE1(61044,dayskipl);
		if(twophase==1)
		{
		 if(hv2>650)
		hv2=650;
	if(hv2<400)
		hv2=400;
hvh=hv2/256;
hvl=hv2%256;
Write_DATAFLASH_BYTE1(61047,hvh);
Write_DATAFLASH_BYTE1(61048,hvl);
	
	if(lv2>370)
		lv2=370;
	if(lv2<280)
		lv2=280;
lvh=lv2/256;
lvl=lv2%256;
Write_DATAFLASH_BYTE1(61049,lvh);
Write_DATAFLASH_BYTE1(61050,lvl);
	
	if(ovl2<10)
		ovl2=10;
    if(ovl2>500)
		  ovl2=500;
ovlh=ovl2/256;
ovll=ovl2%256;
    Write_DATAFLASH_BYTE1(61051,ovlh);
    Write_DATAFLASH_BYTE1(61052,ovll);

	if(dryrun2>((ovl2*9)/10))
		  dryrun2=10;
	  if(dryrun2<10)
		dryrun2=((ovl2*9)/10);
    dryrunh=dryrun2/256;
    dryrunl=dryrun2%256;
    Write_DATAFLASH_BYTE1(61053,dryrunh);
    Write_DATAFLASH_BYTE1(61054,dryrunl);
		
		Write_DATAFLASH_BYTE1(61055,capcuttime);
		
		
}
		
		if(ondelay>999)
			ondelay=5;
		if(ondelay<=0)
			ondelay=5;
		
hvh=ondelay/256;
hvl=ondelay%256;
Write_DATAFLASH_BYTE1(61057,hvh);
Write_DATAFLASH_BYTE1(61058,hvl);
		
		if(ovlcut1>99)
			ovlcut1=99;
		if(ovlcut1<=0)
			ovlcut1=14;
	Write_DATAFLASH_BYTE1(61065,ovlcut1);
		
		if(drycut1>999)
			drycut1=999;
		
		if(drycut1<=0)
			drycut1=10;
		
			dryrunh=drycut1/256;
    dryrunl=drycut1%256;
		Write_DATAFLASH_BYTE1(61066,dryrunh);
    Write_DATAFLASH_BYTE1(61067,dryrunl);
		
		
		if(ubvolt1>100)
			ubvolt1=100;
			if(ubvolt1<=0)
			ubvolt1=50;
		Write_DATAFLASH_BYTE1(61068,ubvolt1);
		
				if(ubcur1>10)
					ubcur1=10;
				
				if(ubcur1<=0)
					ubcur1=3;
		Write_DATAFLASH_BYTE1(61069,ubcur1);
		
		
		Write_DATAFLASH_BYTE1(61070,stardelta);
		
				if(starondelay<=0)
				 starondelay=5;
				if(starondelay>99)
				 starondelay=99;
				Write_DATAFLASH_BYTE1(61071,starondelay);
		/*if(wlcaddress>9)
			wlcaddress=9;
		Write_DATAFLASH_BYTE1(61056,wlcaddress);*/
		
//if(smenu==4||smenu==5||smenu==6||smenu==7||smenu==8||smenu==9||smenu==10||smenu==11)
if(smenu>=4&&smenu<=11)
 {
  combine();
 }
 IE|=(1<<7);
}
void readdata(void)
{
	////////////////////read the data from the eeprom////////////////////////////////
  Read_APROM_BYTE(62000);
  errt=rdata;
  if(errt>10)
   {
     errt=0;
   }
  Read_APROM_BYTE(62001);
  errtview=rdata;
  if(errtview>10)
   {
     errtview=0;
   }
  Read_APROM_BYTE(62002);
  errorsave1=rdata;
  if(errorsave1>20)
   {
     errorsave1=0;
   }
	 
	Read_APROM_BYTE(61045);
 phase=rdata;
 if(phase>5)
	 phase=3;//3
 
 Read_APROM_BYTE(61001);
  mode=rdata;
  if(mode>5)
  mode=3;//1
	

Read_APROM_BYTE(61003);
oftimestart=rdata;
if(oftimestart>=250)
 {
	 oftimestart=0;
 } 

 
  Read_APROM_BYTE(61004);
  drth=rdata;
  Read_APROM_BYTE(61005);
  drtl=rdata;
  drt=((drth<<8)|drtl);
  if(drt>999)
   drt=10;

  

 

  Read_APROM_BYTE(61006);           
  ontimeh=rdata;
  Read_APROM_BYTE(61007);
  ontimel=rdata;
  ontime=((ontimeh<<8)|ontimel);
  if(ontime>=50000)
  {
	ontime=001;
  }
	
  Read_APROM_BYTE(61008);           
  run2h=rdata;
	Read_APROM_BYTE(61009);
	run2l=rdata;
	run2=((run2h<<8)|run2l);
	if(run2>=1000)
  {
	   run2=001;
  }
	

 Read_APROM_BYTE(61010);           
  offtimeh=rdata;
  Read_APROM_BYTE(61011);
  offtimel=rdata;
  offtime=((offtimeh<<8)|offtimel);
  if(offtime>=1000)
  {
	offtime=001;
  }
        
					

  Read_APROM_BYTE(61014);           
  run3h=rdata;
	Read_APROM_BYTE(61015);
	run3l=rdata;
	run3=((run3h<<8)|run3l);
	if(run3>=1000)
  {
	   run3=001;
  }
	Read_APROM_BYTE(61060);           
  run3h=rdata;
	Read_APROM_BYTE(61061);
	run3l=rdata;
	drt1=((run3h<<8)|run3l);
	if(drt1>=1000)
  {
	   drt1=drt;
  }


  Read_APROM_BYTE(61016);           
        timeron=rdata;
		    if(timeron>=10)
        {
	       timeron=0;
        }	
	
       Read_APROM_BYTE(61017);
				 h1on=rdata;
				 if(h1on>250)
					  h1on=16;
				 Read_APROM_BYTE(61018);
				 m1on=rdata;
				 if(m1on>250)
					  m1on=10;
				 onrtcvalue1=((h1on*100)+m1on);


         Read_APROM_BYTE(61019);
				 h1off=rdata;
				 if(h1off>250)
					  h1off=16;
				 Read_APROM_BYTE(61020);
				 m1off=rdata;
				 if(m1off>250)
					  m1off=40;
				 offrtcvalue1=((h1off*100)+m1off);


       Read_APROM_BYTE(61021);
				 h2on=rdata;
				 if(h2on>250)
					  h2on=9;
				 Read_APROM_BYTE(61022);
				 m2on=rdata;
				 if(m2on>250)
					  m2on=40;
				 onrtcvalue2=((h2on*100)+m2on);

         Read_APROM_BYTE(61023);
				 h2off=rdata;
				 if(h2off>250)
					  h2off=9;
				 Read_APROM_BYTE(61024);
				 m2off=rdata;
				 if(m2off>250)
					  m2off=41;
				 offrtcvalue2=((h2off*100)+m2off);
				 
				 
				 Read_APROM_BYTE(61025);
				 h3on=rdata;
				 if(h3on>250)
					  h3on=9;
				 Read_APROM_BYTE(61026);
				 m3on=rdata;
				 if(m3on>250)
					  m3on=42;
				 onrtcvalue3=((h3on*100)+m3on);
				 
				 Read_APROM_BYTE(61027);
				 h3off=rdata;
				 if(h3off>250)
					  h3off=9;
				 Read_APROM_BYTE(61028);
				 m3off=rdata;
				 if(m3off>250)
					  m3off=43;
				 offrtcvalue3=((h3off*100)+m3off);
				 
				 Read_APROM_BYTE(61029);
				 h4on=rdata;
				 if(h4on>250)
					  h4on=9;
				 Read_APROM_BYTE(61030);
				 m4on=rdata;
				 if(m4on>250)
					  m4on=44;
				 onrtcvalue4=((h4on*100)+m4on);
				 
				  Read_APROM_BYTE(61031);
				 h4off=rdata;
				 if(h4off>250)
					  h4off=9;
				 Read_APROM_BYTE(61032);
				 m4off=rdata;
				 if(m4off>250)
					  m4off=50;
				 offrtcvalue4=((h4off*100)+m4off);

				 
         Read_APROM_BYTE(61033);           
         hvh=rdata;
				 Read_APROM_BYTE(61034);
				 hvl=rdata;
				 hv=((hvh<<8)|hvl);
				 if(hv>320&&phase==1)
	        hv=260;
          if(hv>1000&&phase==3)
	        hv=460;//460
					 if(hv<400&&phase==3)
	        hv=400;//460

          Read_APROM_BYTE(61035);           
         lvh=rdata;
				 Read_APROM_BYTE(61036);
				 lvl=rdata;
				 lv=((lvh<<8)|lvl);
				 if(lv>230&&phase==1)
	        lv=180;
          if(lv>1000&&phase==3)
	        lv=370;//280
					if(lv<280&&phase==3)
	        lv=280;//280

         Read_APROM_BYTE(61037);           
         ovlh=rdata;
         Read_APROM_BYTE(61038);
				 ovll=rdata;
         ovl=((ovlh<<8)|ovll);
         if(ovl>500&&phase==1)
	           ovl=80;
          if(ovl>500&&phase==3)
	        ovl=120;

					 Read_APROM_BYTE(61012);           
         dryrunh=rdata;
         Read_APROM_BYTE(61013);           
         dryrunl=rdata;
         dryrun=((dryrunh<<8)|dryrunl);
          if(dryrun>500&&phase==1)
              dryrun=10;    
          if(dryrun>500&&phase==3)
            dryrun=70;
					if(dryrun<10)
            dryrun=((ovl*90)/10);
					
					Read_APROM_BYTE(61039);           
         scrolltype=rdata;
					if(scrolltype>3)
						scrolltype=1;
					
				Read_APROM_BYTE(61040);           
         refdate=rdata;
					if(refdate>31)
						refdate=1;
					day1=refdate;
					
					Read_APROM_BYTE(61041);           
         refmon=rdata;
					if(refmon>12)
						refmon=1;
					month1=refmon;
					
					Read_APROM_BYTE(61042);           
         refyear=rdata;
					if(refyear>99)
						refyear=0;
					year1=(20*100)+refyear;
					
					
					//Write_DATAFLASH_BYTE1(61043,dayskiph);
		//Write_DATAFLASH_BYTE1(61044,dayskipl);
					
					Read_APROM_BYTE(61043);           
         dayskiph=rdata;
					
					Read_APROM_BYTE(61044);           
         dayskipl=rdata;
					dayskip=((dayskiph<<8)|dayskipl);
					if(dayskip>999)
						dayskip=0;
					
					Read_APROM_BYTE(61046);           
         twophase=rdata;
					if(twophase>10)
						twophase=2;
					
					Read_APROM_BYTE(61047);           
         hvh=rdata;
				 Read_APROM_BYTE(61048);
				 hvl=rdata;
				 hv2=((hvh<<8)|hvl);
          if(hv2>650)
	        hv2=650;//460
					 if(hv2<400)
	        hv2=400;
					 
					 
					Read_APROM_BYTE(61049);           
         lvh=rdata;
				 Read_APROM_BYTE(61050);
				 lvl=rdata;
				 lv2=((lvh<<8)|lvl);
				 
          if(lv2>370)
	        lv2=370;//280
					if(lv2<280)
	        lv2=280;//280
					
					
					 Read_APROM_BYTE(61051);           
         ovlh=rdata;
         Read_APROM_BYTE(61052);
				 ovll=rdata;
         ovl2=((ovlh<<8)|ovll);
         if(ovl2>500)
	           ovl2=80;
     
				  Read_APROM_BYTE(61053);           
         dryrunh=rdata;
         Read_APROM_BYTE(61054);           
         dryrunl=rdata;
         dryrun2=((dryrunh<<8)|dryrunl);   
          if(dryrun2>500)
            dryrun2=70;
					if(dryrun2<10)
            dryrun2=((ovl*90)/10);
					
					Read_APROM_BYTE(61055);           
         capcuttime=rdata;
					if(capcuttime>99)
						capcuttime=1;
					

					Read_APROM_BYTE(61057);           
         hvh=rdata;
					Read_APROM_BYTE(61058);           
         hvl=rdata;
					ondelay=((hvh<<8)|hvl); 
					if(ondelay>999)
						ondelay=5;
					
					Read_APROM_BYTE(61059);           
         dryrestart=rdata;
					if(dryrestart>5)
						dryrestart=0;
					
					/*Read_APROM_BYTE(61056);           
         wlcaddress=rdata;
					if(wlcaddress>9)
						wlcaddress=0;*/
					
				
					
					Read_APROM_BYTE(61065);           
         ovlcut1=rdata;
					if(ovlcut1>99)
					ovlcut1=14;
					
					
		
		
		     Read_APROM_BYTE(61066);           
         dryrunh=rdata;
         Read_APROM_BYTE(61067);           
         dryrunl=rdata;
         drycut1=((dryrunh<<8)|dryrunl);   
		     if(drycut1>999)
			   drycut1=10;
		
		
		
		
		Read_APROM_BYTE(61068);           
         ubvolt1=rdata;
		if(ubvolt1>100)
			ubvolt1=50;
	
				
				Read_APROM_BYTE(61069);           
         ubcur1=rdata;
		if(ubcur1>10)
			ubcur1=9;
		
		Read_APROM_BYTE(61070);           
        stardelta=rdata;
			if(stardelta>2)
				stardelta=2;
			
			Read_APROM_BYTE(61071);           
        starondelay=rdata;
					if(starondelay>99)
						starondelay=5;
            combine();
         

}
void digitsel(void)
{
	/////////digit selection function for set mode 
switchscan();
if(!sidekey)
spress=0;
if(sidekey&&spress==0&&smenu!=28&&smenu!=29)
	{
setreset=0;		
	delay(1);
	if(sidekey)
		{
		blink++;
		cblink++;
		spress=1;
		chang=1;
    write=1;
		chip=0;
    //if((smenu==1)||(smenu==2)||(smenu==3)||(smenu==12)||(smenu==15)||(smenu==16)||(smenu==17)||(smenu==18))
if((smenu>=1&&smenu<=3)||(smenu==12)||(smenu>=15&&smenu<=18)||(smenu==20||smenu==21||smenu==23)||(smenu>=30&&smenu<=33))   
			{
    if(blink>3)
    blink=1;
    if(cblink>3)
    cblink=1;}
			
			if(smenu==34||smenu==19||smenu==22||smenu==36)
			{
			if(blink>2)
    blink=1;
    if(cblink>2)
    cblink=1;}
    
   //if(smenu==4||smenu==5||smenu==6||smenu==7||smenu==8||smenu==9||smenu==10||smenu==11||smenu==24)
		if((smenu>=4&&smenu<=11)||(smenu==24))
    {
    if(blink>4)
    blink=1;
    if(cblink>4)
    cblink=1;
    }
    /*if(smenu==x)
    {
      if(blink>2)
    blink=1;
    if(cblink>2)
    cblink=1;
    }*/
    if(smenu==25||smenu==13)
    {
      if(blink>6)
      blink=1;
      if(cblink>6)
      cblink=1;
      cursor=0xc6+blink;
    }
		//cursor=0xc9+blink;
  if((smenu==1)||(smenu==2)||(smenu==23)||smenu==21||smenu==22)
    cursor=0xca+blink;
   // if((smenu==4||smenu==5)||smenu==6||smenu==7||smenu==8||smenu==9||smenu==10||smenu==11||smenu==24)
	if((smenu>=4&&smenu<=11)||(smenu==24))
    cursor=0xc7+blink;
    if(smenu==3)
    cursor=0xc8+blink;
    if(smenu==15||smenu==16||smenu==30||smenu==31)
    cursor=0xc9+blink;
		if(smenu==34||smenu==36)
			cursor=0xcc+blink;
			
    if(smenu==17||smenu==18||smenu==32||smenu==33)
    cursor=0xc8+blink;
    if((smenu==17||smenu==18||smenu==32||smenu==33)&&(cblink>2))
    cursor=0xc8+blink+1;
   // if((smenu==4||smenu==5||smenu==6||smenu==7||smenu==8||smenu==9||smenu==10||smenu==11||smenu==24)&&(cblink>=3))
    if(((smenu>=4&&smenu<=11)||(smenu==24))&&(cblink>=3))  
		cursor=0xc7+blink+1;
      if((smenu==25||smenu==13)&&(cblink>2&&cblink<5))
      cursor=0xc6+blink+1;
      if((smenu==25||smenu==13)&&(cblink>4&&cblink<7))
      cursor=0xc6+blink+2;
			if(smenu==12)
		cursor=0xc9+blink+1;
			if(smenu==19||smenu==20)
		cursor=0xcb+blink;
			
		enabler(cursor);
		delay(10);
		enabler(0x0d);
		delay(10);
		chip=1;
		}
	}
	if(smenu==28&&errt!=0&&sidekey==1&&spress==0)
	{
		///////this is for error  log increment
		setreset=0;
		spress=1;
		errt=0;
		errtview=0;
		errorsave1=0;
		 IE&=~(1<<7);
		 Write_DATAFLASH_BYTE1(62000,errt);
	   Write_DATAFLASH_BYTE1(62001,errtview);
    	Write_DATAFLASH_BYTE1(62002,errorsave1);
			IE|=(1<<7);
		fline();
		chip=1;
		for(t3=0;t3<16;t3++)
		{
			temp=crlog3[t3];
			enabler(temp);
			delay(1);
		}
		sline();
		chip=1;
		for(t3=0;t3<16;t3++)
		{
			temp=crlog4[t3];
			enabler(temp);
			delay(1);
		}
			chip=0;
enabler(0x0c);
delay(0x0a);
		
		
	}
}
void incrout(void)
{
	///////this function for set mode incrementation
switchscan();
if(!inckey)
incpress=0;
if(inckey&&chang==1&&incpress==0&&smenu!=26&&smenu!=27&&smenu!=29)
	{
		setreset=0;
	incpress=1;
	if(inckey)
		{
    delay(1);
		digit1[cblink]++;
		datachang=1;
    }
		if(digit1[cblink]>9)
		  digit1[cblink]=0;
    
		disdat=digit1[cblink]+0x30;
		chip=0;
    if((smenu==1)||(smenu==2))
		cursor=0xca+blink;
    //if(smenu==4||smenu==5||smenu==6||smenu==7||smenu==8||smenu==9||smenu==10||smenu==11||smenu==24)
		if((smenu>=4&&smenu<=11)||(smenu==24))
    cursor=0xc7+blink;
    if(smenu==3)
    cursor=0xc8+blink;
    if(smenu==17||smenu==18||smenu==32||smenu==33)
    cursor=0xc8+blink;
    if((smenu==17||smenu==18||smenu==32||smenu==33)&&(cblink>2))
    cursor=0xc8+blink+1;
   //if((smenu==4||smenu==5||smenu==6||smenu==7||smenu==8||smenu==9||smenu==10||smenu==11||smenu==24)&&(cblink>=3)&&(blink>=3))
if(((smenu>=4&&smenu<=11)||(smenu==24))&&((cblink>=3)&&(blink>=3)))   
		{
   cursor=0xc7+blink+1;
    }
      if(smenu==15||smenu==16||smenu==30||smenu==31)
      cursor=0xc9+blink;
			if(smenu==34||smenu==36)
				cursor=0xcc+blink;
			if(smenu==19||smenu==20)
				cursor=0xcb+blink;
				
			if(smenu==12||smenu==23||smenu==21||smenu==22)
			cursor=0xca+blink;	
		enabler(cursor);
		delay(10);
		chip=1;
		enabler(disdat);
		delay(10);
		chip=0;
		enabler(cursor);
		delay(10);
		enabler(0x0d);
		}
	if(inckey==1&&smenu==27&&errt!=0&&incpress==0)
	{
		setreset=0;
		errorlog=1;
		incpress=1;
	errortime=errortime+1;
 errortime1=errortime1+2;
	if(errortime>10)//for error
		{
     errortime=1;
		 errtinc=errt;
		}
	if(errortime1>20)//for number
	 {
		errortime1=2;
		errtinc1=errorsave1;
	 }
	 if((errortime>errtinc)&&(errtview!=10))
	 {
		 errortime=1;
		 errortime1=2;
		 errtinc=errt;
		 errtinc1=errorsave1;
	 }
	if(errtinc>=errortime)//for error
		{
		errt1=errtinc-errortime;		
		}
if(errtinc1>=errortime1)//for number
		{
		errt2=errtinc1-errortime1;		
		}			
if((errortime>errtinc)&&(errtview!=10))//for error
		{
		errt1=errortime;		
		}
if((errortime1>errtinc1)&&(errtview!=10))//for number
		{
		errt2=errortime1;		
		}		
 if(errtview==10)
		{
			if(errortime>errtinc)//for error
			{
		  errtinc=errt+11;//6
			errtinc=errtinc-1;
			errt1=errtinc-errortime;
			}
			if(errortime1>errtinc1)//for number
			{
		  errtinc1=errorsave1+22;
			errtinc1=errtinc1-2;
			errt2=errtinc1-errortime1;
			}
		}
    
    errorrepeat();
    errorlogdisplay();
	}
	if(inckey==1&&smenu==27&&errt==0&&incpress==0)
	{
		incpress=1;
		fline();
		for(t3=0;t3<16;t3++)
		{
			temp=erlog3[t3];
			enabler(temp);
			delay(1);
		}
		sline();
		for(t3=0;t3<16;t3++)
		{
			temp=' ';
			enabler(temp);
			delay(1);
		}
		chip=0;
enabler(0x0c);
delay(0x0a);
	}
}

void checkcblink(void)
{///not used
if(digit1[cblink]>2)
		digit1[cblink]=1;
}
void combin(void)
{/// this function used to coimbining the single char value to whole integer value 
  /*if(smenu==x)
  {
    buffer=digit1[1]*10+digit1[2];
  }*/

//if((smenu==1)||(smenu==2)||(smenu==3)||(smenu==12)||(smenu==15)||(smenu==16)||(smenu==17)||(smenu==18))
	if((smenu>=1&&smenu<=3)||(smenu==12)||(smenu==20)||(smenu==21||smenu==23)||(smenu>=15&&smenu<=18)||(smenu>=30&&smenu<=33))
{
buffer=digit1[1]*100+digit1[2]*10+digit1[3];}
if(smenu==34||smenu==19||smenu==22||smenu==36)
{
	buffer=digit1[1]*10+digit1[2];
}

//if((smenu==4||smenu==5||smenu==6||smenu==7||smenu==8||smenu==9||smenu==10||smenu==11||smenu==24))
if((smenu>=4&&smenu<=11)||(smenu==24))
{
buffer=digit1[1]*1000+digit1[2]*100+digit1[3]*10+digit1[4];
bufferh=buffer/100;
bufferl=buffer%100;
}
if(smenu==25)
{
  date=digit1[1]*10+digit1[2];
  mon=digit1[3]*10+digit1[4];
  yr=digit1[5]*10+digit1[6];
}
if(smenu==13)
{
	refdate=digit1[1]*10+digit1[2];
	refmon=digit1[3]*10+digit1[4];
	refyear=digit1[5]*10+digit1[6];
}

//if(smenu==6||smenu==7)
//buffer=buffer*10+digit1[4];
}

///////////////////////////////////////////////////RTC//////////////////////////////////////////////////////////////////////////////////////////////////////
void tx(void)
{
count=0;
while(count<8)
{
eemem=eemem&0xbf;
buff=txbuf&0x80;
if(buff==0x80)
eemem=eemem|0x40;
bitout();
txbuf=txbuf<<1;
count++;
}
bitin();
}
void rx(void)
{
datai=0;
count=0;
while(count<8)
{
datai=datai<<1;
bitin();
buff=eemem&0x80;
if(buff==0x80)
datai=datai|0x01;
count++;
}
}
void bstart(void)
{	
edat=1;//port
eclk=0;
for(r3=0;r3<1;r3++);
eclk=1;
for(r3=0;r3<1;r3++);
edat=0;
for(r3=0;r3<1;r3++);
eclk=0;
for(r3=0;r3<1;r3++);
}
void bstop(void)
{	
edat=0;
for(r3=0;r3<1;r3++);
eclk=1;
for(r3=0;r3<1;r3++);
edat=1;
for(r3=0;r3<1;r3++);
eclk=0;
for(r3=0;r3<1;r3++);
}

void bitin(void)
{
eemem=eemem|0x80;
P17_INPUT_MODE;
eclk=1;
for(r3=0;r3<1;r3++);
if(edat!=1)
eemem=eemem&0x7f;
eclk=0;
for(r3=0;r3<1;r3++);
P17_PUSHPULL_MODE;
}
void bitout(void)
{
buff=eemem&0x40;
if(buff==0x40)
edat=1;
if(buff!=0x40)
edat=0;
for(r3=0;r3<1;r3++);
eclk=1;
for(r3=0;r3<1;r3++);
eclk=0;
for(r3=0;r3<1;r3++);
}
void timprob(void)
{
rread1(0);
datai=datai&0x80;
if(datai==0x80)
{
datai=0;
rwrbyte1(datai,0x00);
}
}
void rwrbyte1(unsigned char k,unsigned char add)
{
bstart();
txbuf=0xd0;
tx();
txbuf=add;
tx();
txbuf=k;
tx();
bstop();
for(r3=0;r3<5;r3++);
}
void rread1(unsigned char jj)
{
bstart();
txbuf=0xd0;
tx();
txbuf=jj;
tx();
bstart();
txbuf=0xd1;
tx();
count=0;
rx();
eemem=eemem|0x40;
bitout();
bstop();
}
void bcdtodecimal(unsigned char value)
{
    buf= ((value & 0x0F) + (((value & 0xF0) >> 0x04) * 0x0A));
}
void variablecombine(unsigned char value1,unsigned char value2)
{
	combinevalue=((value1*100)+value2);//for time combining
}

void combine(void)
{
	//////RTC time coimbining 12:45 int 1245
variablecombine(h1on,m1on);
ontime1set=combinevalue;
variablecombine(h1off,m1off);	
offtime1set=combinevalue;
	
variablecombine(h2on,m2on);
ontime2set=combinevalue;
variablecombine(h2off,m2off);
offtime2set=combinevalue;
	
	variablecombine(h3on,m3on);
ontime3set=combinevalue;
variablecombine(h3off,m3off);
offtime3set=combinevalue;
	
	variablecombine(h4on,m4on);
ontime4set=combinevalue;
variablecombine(h4off,m4off);
offtime4set=combinevalue;
}
void timread(void)
{
rread1(1);
curmin=datai;
bcdtodecimal(curmin);
runningmin=buf;
rread1(2);
curhour=datai;
bcdtodecimal(curhour);
runninghour=buf;
variablecombine(runninghour,runningmin);
runningtime=combinevalue;	
rtctime=runningtime;

  rread1(4);
	curdate=datai;
  bcdtodecimal(curdate);
	runningdate=buf;
	day2=runningdate;
	
	rread1(5);
	curmonth=datai;
  bcdtodecimal(curmonth);
	runningmonth=buf;
	month2=runningmonth;
	
	variablecombine(runningdate,runningmonth);
	runningdaymonth=combinevalue;
	rread1(6);
	curyear=datai;
  bcdtodecimal(curyear);
	runningyear=buf;
	year2=((20*100)+runningyear);

	//rtcdate=(((runningdate/10)*100000)+((runningdate%10)*10000)+((runningmonth/10)*1000)+((runningmonth%10)*100)+((runningyear/10)*10)+(runningyear%10));
	//rtcdate=((12*10000)+(5*100)+(25)); 
	
 // rtcdate=((runningdate*10000)+(runningmonth*100)+(runningyear));
//rtcdate=170525;
}
void timecompare(void)
 {
if(futuretime<presenttime)
		{
			if(runningtime<=futuretime)
			 runningtime=runningtime+2400;
			 futuretime=futuretime+2400;
			 
		}
	      timeok=0;	   
		if((runningtime>=presenttime)&&(runningtime<futuretime))
		{
			 timeok=1;
		}
	}
  ////////////////////////////////////////////day skip//////////////////////////////////////////////////////////////////////////////////////////////////////
void monthfunction(unsigned int month,unsigned int year)
{
	if((month==1)||(month==3)||(month==5)||(month==7)||(month==8)||(month==10)||(month==12))
		 monthday=31;
	if((month==4)||(month==6)||(month==9)||(month==11))
		 monthday=30;
	if(month==2)
		 monthday=28;
	if((month==2)&&(((year%4==0)&&(year%100!=0))||(year%400==0)))
	   monthday=29;	
}
void passeddays(unsigned int day,unsigned int month,unsigned int year)
{
	totaldays=0;
	for(m=1;m<month;m++)
	{
	  monthfunction(m,year);
	  totaldays=monthday+totaldays;
	}
  totaldays=totaldays+day;
}
void remainingdays(unsigned int day,unsigned int month, unsigned int year)
{
	totaldays=0;
	monthfunction(month,year);
  totaldays=monthday-day;//remaining days in start month
	
	for(m=month+1;m<=12;m++)
	{
	  monthfunction(m,year);
		totaldays=monthday+totaldays;
	}
}
void daysbetweenyears(void)     
 {
	if(year1==year2)
	{
		totalpass3=0;
	  passeddays(day2,month2,year2);
		totalpass2=totaldays;
		passeddays(day1,month1,year1);
		totalpass1=totaldays;
		totalpass3=(totalpass2-totalpass1);
	}
	if(year1!=year2)
	{
		totalpass4=0;
		totalpass3=0;
	  remainingdays(day1,month1,year1);//to calculte the intial starting year
    totalpass4=totaldays;		
		
  for(y=year1+1;y<year2;y++)//for intermediate years
  {
		yeardays=365;
   if(((y%4==0)&&(y%100!=0))||(y%400==0))
	 {
	   yeardays=366;
	 }
	 totalpass4=totalpass4+yeardays;	
	 }

   passeddays(day2,month2,year2);//for remeining years
	 totalpass3=totalpass4+totaldays;
	 }
	  dayon=totalpass3%(dayskip+1);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void errorrepeat(void)
{
   Read_APROM_BYTE(62011+errt1);
    errordisplay1=rdata;
		if(errordisplay1>=100)
		errordisplay1=0;
    readerror();
}
void errorview(void)
{
	errt=errt+1;
	errtview=errtview+1;
	if(errtview>10)
		 errtview=10;
	if(errt>10)
		 errt=1;
	errorsave1=errorsave1+2;
	if(errorsave1>20)
		 errorsave1=2;
}
void readerror(void)
{
	/// this for error read from the eeprom
if(errordisplay1>=1&&errordisplay1<=9)
{
Read_APROM_BYTE(62031+(errt2));
hour=rdata;
Read_APROM_BYTE(62031+(errt2+1));
minute=rdata;
errtime=(hour*100)+minute;
if(errtime>2359)
errtime=0;

Read_APROM_BYTE(62052+(errt2));
edate=rdata;
Read_APROM_BYTE(62052+(errt2+1));
emonth=rdata;
errdate=(edate*100)+emonth;
if(errdate>3112)
errdate=0;

if((errordisplay1>=1&&errordisplay1<=4)||(errordisplay1==6))
{
Read_APROM_BYTE(62073+(errt2));
errorvalueh=rdata;
Read_APROM_BYTE(62073+(errt2+1));
errorvaluel =rdata;
errorvaluex=((errorvalueh<<8)|errorvaluel);
if(errorvaluex>1000)
errorvaluex=0;
Read_APROM_BYTE(61091+errt1);
phasenumber1=rdata;
if(phasenumber1>5)
phasenumber1=0;
}
	
}	
}
void errorlogdisplay(void)
{
	ulimit=0;
  fline();
  for(t3=0;t3<16;t3++)
  {
    temp=errorlogd[t3];
    enabler(temp);
    delay(1);
  }
  twodigitdisp=1;
  buffer=errortime;
  split2();
  address=0x82;
  valuewrite2();
  twodigitdisp=0;

  vshow=1;
  buffer=errdate;
  split2();
  address=0x85;
  valuewrite2();
  vshow=0;

  amps=1;
  buffer=errtime;
  split2();
  address=0x8b;
  valuewrite2();
  amps=0;

  sline();
  for(t3=0;t3<16;t3++)
  {
    if(errordisplay1==1)//->high voltage
    temp=errorhv[t3];
    if(errordisplay1==2)//->low voltage
    temp=errorlv[t3];
    if(errordisplay1==3)//-> dry run 
    temp=errordry[t3];
    if(errordisplay1==4)//-> over load 
    temp=errorovl[t3];
    if(errordisplay1==5&&phase==3)//->phase unbalance
    temp=errorph[t3];
    if(errordisplay1==6&&phase==3)//->phase fail
    temp=errorpf[t3];
    if(errordisplay1==7&&phase==3)//->phase reversal
    temp=errorpr[t3];
		if(errordisplay1==8&&phase==3)//-> current unbalancecurrent
			temp=unbc[t3];
	if(errordisplay1==9)//-> sump empty
			temp=sumpemty[t3];
    enabler(temp);
    delay(1);
    }
	if(phase==1)
	{
		sline();
		if(errordisplay1==1||errordisplay1==2)
			ulimit=3;//this is for value priniting limit
		
		/*if(errordisplay1==3||errordisplay1==4)
			ulimit=2;*/
		
		for(t3=0;t3<ulimit;t3++)
		{
			temp=' '; 
			enabler(temp);
    delay(1);
		}
	}
  //if(errordisplay1==1||errordisplay1==2||errordisplay1==3||errordisplay1==4||errordisplay1==6)
	if((errordisplay1>=1&&errordisplay1<=4)||(errordisplay1==6))
    {
     sline();
	if((errordisplay1>=1&&errordisplay1<=2)&&(phase==3))
	{		
//if((phasenumber1==1&&(errordisplay1==1||errordisplay1==2))&&(phase==3))
		if(phasenumber1==1)
{
enabler(cr);
delay(1);
enabler(cy);
delay(1);
}
//if((phasenumber1==2&&(errordisplay1==1||errordisplay1==2))&&(phase==3))
if(phasenumber1==2)
{
enabler(cy);
delay(1);
enabler(cb);
delay(1);
}
//if((phasenumber1==3&&(errordisplay1==1||errordisplay1==2))&&(phase==3))
if(phasenumber1==3)
{
enabler(cb);
delay(1);
enabler(cr);
delay(1);
}
}
if((errordisplay1==6))//|errordisplay1==3||errordisplay1==4)&&(phase==3))
{
  if(phasenumber1==1)
enabler(cr);
if(phasenumber1==2)
enabler(cy);
if(phasenumber1==3)
enabler(cb);
delay(1);
}
errorprint=1;
buffer=errorvaluex;
//buffer=dispry;
  split2();
  if(errordisplay1==1||errordisplay1==2)
  address=0xcb;
  if(errordisplay1==6)
  address=0xcc;
   if(errordisplay1==3)
  address=0xc9;
  if(errordisplay1==4)
  address=0xca;
errordisplayprint();
errorprint=0;
  }
chip=0;
enabler(0x0c);
delay(0x0a);
  }
void pumponfunction(void)
{

	////////pump on function
	if (reset1&&ondelayok) 
	{
    reset1=0;
		ondelayok=0;
    if (phase==1) 
			{
        crmrly=1;
        crmon=1;
      } 
			else if(phase==3)
				{
        startrly=1;
        pumpon=1;
    }
}

if(crmon&&phase==1) 
{
   if(++pumpsensingcount>10)
			{  
        pumpsensingcount=0;
        crmon=0;
				pumpsensed=1;
        pumptype=(rctval1>=10)?1:2;
        refresh=1;
        counter=0;
        if(pumptype==2)
					  {
            startrly=1;
            pumpon=1;
            startcrmoff=1;
            }
       }
}
if(pumpon)
	{
    if(++startrlyoffcount>45)
			  {  
       startrlyoffcount=0;
       startrly=0;
       pumpon=0;
        }
  }
if(startcrmoff)
{
    if(++crmoffcount>37)
			{
        crmoffcount=0;
        crmrly=0;
        startcrmoff=0;
      }
}
}
void pumpofffunction(void)
{
	//pump off function
	if(relayoff==1)
      {
				pon=0;
				Nop=0;
				relayoff=0;
				
      if((mode==4)&&(timeron==1||timeron==3||timeron==5||timeron==7)&&(rtcstart==1))
                        rtcstart=0;
			run7=0;
			run8=0;
		if(phase==1&&pumptype==1)
		{
			crmrly=0;
			refresh=1;
		}
			if(phase==1&&pumptype==2)
			{
				stoprly=1;
			//	pumpoff=1;
				
			}
			if(phase==3)
			{
				stoprly=1;
				//pumpoff=1;
			}
      start1=0;
			if(restart==1||start2==1)
				pumpoff=0;
      }
			if(pumpoff==1)
			{
				stoprlyoffcount++;
				if(stoprlyoffcount>45)
				{
					stoprlyoffcount=0;
					stoprly=0;
					pumpoff=0;
					refresh=1;
				}
			}	
}
void twophaseon(void)
{
	// two phase pump on function 
	if(twophaseselected==1&&switched==1)
{
	relay2=1;
}
if(relay2==1)
{
	relayoncount++;
	if(relayoncount>14)
	{
		relayoncount=0;
		relayoffcount++;
		if(relayoffcount>capcuttime)
		{
			relayoffcount=0;
			relay2=0;
			switched=0;
		}
	}
}
}
void stardeltapumpon(void)
{
	//need to check two phase enabled or disabled
	// stardelta pump on function 
	if(reset1==1&&ondelayok==1)
	{
		reset1=0;
		ondelayok=0;
		pon=1;
		crmrly=1;//main
		relay2=1;//star
		starrelayoff=1;
	}
	if(starrelayoff==1)
		starrelayoffcount++;
	if(starrelayoffcount>=(starondelay*8))
	{
		starrelayoff=0;
		starrelayoffcount=0;
		relay2=0;//star
		deltaon=1;
		//startrly=1;//delta
	}
	if(deltaon==1)
		deltaoncount++;
	if(deltaoncount>3)
	{
		deltaoncount=0;
		deltaon=0;
		startrly=1;
		
	}
}
void stardeltapumpoff(void)
{
	// stardelta pump off function 
	if(relayoff==1)
	{
		    pon=0;
				Nop=0;
				relayoff=0;
      if((mode==4)&&(timeron==1||timeron==3||timeron==5||timeron==7)&&(rtcstart==1))
                        rtcstart=0;
			run7=0;
			run8=0;
			start1=0;
			crmrly=0;
			startrly=0;
			relay2=0;
			stoprly=1;
			refresh=1;
			deltaon=0;
			reset1=0;
			deltaoncount=0;
			starrelayoff=0;
			starrelayoffcount=0;
			ondelayok=0;
			ondelaycheck=0;
	}
}
void pwm_init(void)
{
	//pwm init function
	PWM0_CLOCK_FSYS;
	PWM0_CENTER_TYPE ;
	PWM0_IMDEPENDENT_MODE;
	PWM0_CLOCK_DIV_8;
	ENABLE_PWM0_CH5_P15_OUTPUT;
	SFRS=0;
	
	PWM0PH=((1000&0xff00)>>8);
	PWM0PL=(1000&0x00ff);
	
	ENABLE_SFR_PAGE1;
	PWM0C5H=((0&0xff00)>>8);
	PWM0C5L=(0&0x00ff);
	SFRS=0;
	PWM0CON0&=~(1<<7);
	PWM0CON0|=(1<<6);
	PWM0CON0|=(1<<7);
}
void Pwmduty(unsigned int dutycycle)
{
	//PWM0PH=((1000&0xff00)>>8);
	//PWM0PL=(1000&0x00ff);
	ENABLE_SFR_PAGE1;
	PWM0C5H=((dutycycle&0xff00)>>8);
	PWM0C5L=(dutycycle&0x00ff);
	SFRS=0;
	PWM0CON0&=~(1<<7);
	PWM0CON0|=(1<<6);
	PWM0CON0|=(1<<7);
}
char checkvolt(unsigned int v) {
    return (v > (lowvolt+15)) && (v < (highvolt-5));
}
/*-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
								                      Aprom read& write routine
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
unsigned char Read_APROM_BYTE(unsigned int code *u16_addr)
{
    //UINT8 rdata;
    rdata = *u16_addr >> 8;
    return rdata;
}
void Write_DATAFLASH_BYTE1(unsigned int u16EPAddr, unsigned char u8EPData)
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
int compare(int comp1, int comp2)
{
	if(comp1>comp2)
		return comp1-comp2;
	
	else
		return comp2-comp1;
	
}
void checkctvalues(unsigned int rct1, unsigned int yct1,unsigned int bct1)
{
	// curret unbalance decide function
	if((start2==0)&&(rctval>=10)&&(volthigh==0)&&(voltlow==0)&&(start1==1)&&(phase==3)&&(restart==0)&&(ovld==0)&&(dry==0))
	{
	unbalancecurrent=0;
	if(compare(rct1,yct1)>(ubcur1*10))
	{
		unbalancecurrent=1;
	}
	
	if(compare(yct1,bct1)>(ubcur1*10))
	{
		unbalancecurrent=1;
	}
		
	if(compare(bct1,rct1)>(ubcur1*10))
	{
		unbalancecurrent=1;
	}
}
	if(rctval<10&&restart==0&&unbalancecurrent==1)
	{
		unbalancecurrent=0;
	}
	
}

void checkvoltvalues(unsigned int ry1, unsigned int yb1,unsigned int br1)
{
	//voltage unbalance decide function
	phaseunbalance=0;
	if(compare(ry1,yb1)>ubvolt1)
	{
		phaseunbalance=1;
	}
	
	if(compare(yb1,br1)>ubvolt1)
	{
		phaseunbalance=1;
	}
		
	if(compare(br1,ry1)>ubvolt1)
	{
		phaseunbalance=1;
	}
	
}

/*void FlashWrite(void)
/***********************************Uart0 functions***************************/
/*void uart0_init(){
SFRS=0;
	
		//P06_PUSHPULL_MODE;
	//P07_INPUT_MODE;
SCON = 0x50;
PCON |= 0x80;
/* Set Timer3 as UART baud rate source */
/*T3CON &= 0xF8;
T3CON |= 0x20;  // Select Timer3 for baud rate
RH3 = 0xFF;
RL3 = 0x98;
T3CON |= 0x08; // Start Timer3
ES=1;	
EA=1;
}	*/
/*void uart_transmit(const char *dt) {
     while (*dt != '\0') { {
        UART_Send(*dt);
			 dt++;
    }
	}
}*/
/* Send a single character over UART */
/*void UART_Send(unsigned char c) {
  TI = 0;
    SBUF = c;
    while (!TI); // Wait until transmission complete
   TI = 0; // Clear TI   
}
/* Reset received data buffer */
/*void reset_recindex(void) {
    unsigned char l;
    for (l = 0; l < BUFFER_SIZE; l++) {
        receivedData[l] = '\0';
    }
   cnt= 0;

}
/*****************************LoRa Functions*******************************************************/
/*void lorareset()  {
	delay(50);
        LORARESET= 0;       
      delay(50);
            LORARESET = 1;
}*/
/*void lora_resposecheck(){
		timeout++;
		if(timeout>6)
        {
			timeout=0;
				
            if(lora_datareceived==1)
            {
						
                lora_datareceived=0;
                if(ATtransmit==1)
                {
                    if(stringcontains(receivedData,loraquery[query_response].response)==1)
                    {
                   
                        responseok = 1;
                        responseerror = 0; 
                        reset_recindex();
                       return;
                    }  
                }

                if(ATtransmit==0&&ack_receive_enable==0)
                {
									if(stringcontains(receivedData,"low")==1){
												id_extractor(18);
										if(value==wlcaddress){
									 wlctop1 = 0;									
                    reset_recindex();
                    responseerror = 0;
										}
                    return;
                }
								if(stringcontains(receivedData,"high")==1){	
										id_extractor(19);	
								if(value==wlcaddress){									
									  wlctop1 = 1;
                    reset_recindex();
									relayoff=1;
                    responseerror = 0;
										}
                    return;
                }
						
							}
									if(ack_receive_enable==1)
								{
								if(stringcontains(receivedData,"ack")==1){
									id_extractor(18);
									
									if(value==wlcaddress)
									{
											ack_receive_enable=0;
										ack_timeout=0;
									newid_ok=1;
									reception_ok=1;
									}
									 reset_recindex();
                    return;
								}
							}	
		    }
            reset_recindex();
		     if(ATtransmit==1)
				   responseerror = 1; 
            return;
    }
}	

/************************String Compare************************************************/

/*int stringcontains(const char *haystack, const char *needle)
{
    if (!*needle)
        return -1; // Empty needle matches at start

    for (; *haystack; haystack++) {
        const char *h = haystack;
        const char *n = needle;


        while (*h && *n && (*h == *n)) {
            h++;
            n++;


            if(*h=='\r')
            h++;
            if(*h=='\n')
            h++;
        }
        if (!*n)
            return 1; // Found match
    }
    return 0; // Not found
}

//**********************************************************************************/

/*int loraATtransmit(char queryresponseindex)
{
    reset_recindex();
    timeout=0;
	ATtransmit=1;	 
	query_response=queryresponseindex;
    responseok = 0;
    responseerror = 0;
	uart_transmit(loraquery[query_response].query);//Target Address.
	while(responseok==0&&responseerror==0);
	responseok=0;
    ATtransmit=0;
    reset_recindex();
    if(responseerror==1)
    {
        responseerror = 0;
        return ERROR;
    }
    return SUCCESS;//String compare(At command successful)
}

int loraTxinit()
{
			loraATtransmit(QUIT);
    errorcount = 0;
    while (errorcount<5)
    {
        errorcount++;
         if(loraATtransmit(TARGET_ADD)==ERROR)
				 {
         continue;
				 }
         if(loraATtransmit(TXLOCAL_ADD)==ERROR)
				 {
         continue;
				 }
         if(loraATtransmit(TX_CONFIG)==SUCCESS)
				 {
				 return 1;	//Tx cconfiguration Successfull.	 
				 }
    }
    return 0;//Tx cconfiguration Unsuccessfull.
}

int loraTxdata(char queryindex)
{
    reset_recindex();
    timeout=0;
	ATtransmit=1;	 
	query_response=queryindex;
    responseok = 0;
    responseerror = 0;
	uart_transmit(loraquery[query_response].query);//Send the string and respective response as declared in the structure.
	while(responseok==0&&responseerror==0);
	responseok=0;
    ATtransmit=0;
    reset_recindex();
    if(responseerror==1)
    {
        responseerror = 0;
        return ERROR;//String compare(At command with data string Unsuccessful)
    }
    return SUCCESS;//String compare(At command with data string successful)
}

int loraRxinit()
{
		loraATtransmit(QUIT);
    errorcount = 0;
    while (errorcount<5)
    {
        errorcount++;
         if(loraATtransmit(RXLOCAL_ADD)==ERROR){
         continue;
				 }
         if(loraATtransmit(RX_CONFIG)==SUCCESS){
				 return 1;				 
				 }
    }
    return 0;
}

void custom_sprintf_ctxaddr(char *buffer, int addr, const char *prefix) {
  char temp[10];
	char value=0;
     i = 0;
    while (*prefix) {
        buffer[i++] = *prefix++;
    }
    // Convert integer to string manually
  value=addr;
    j = 0;
    if (addr == 0) {
        temp[j++] = '0';
    } else {
			
        while (addr > 0) {
					
            temp[j++] = (addr % 10) + '0';
            addr /= 10;
        }
				if(value<10){
				     temp[j++] = '0' ;
					}
    }
    // Reverse the digits
    while (j > 0) {
        buffer[i++] = temp[--j];
    }
    // Append \r\n
    buffer[i++] = '\r';
    buffer[i++] = '\n';
    buffer[i] = '\0';
}
void ack_timerout(){
	if(ack_receive_enable==1){
		ack_timeout++;
		if(ack_timeout>2000){
			ack_timeout=0;
			ack_receive_enable=0;
			reception_ok=1;
			reset_recindex();
		}
	}
}
void id_extractor(char num){
	char index=num;
	value=0;
	while(receivedData[index++]!=':');
	
	
	do{
		value=(value*10)+(receivedData[index++]-0X30);
	}while(receivedData[index]>='0'&&receivedData[index]<='9');
	
}*/