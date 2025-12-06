//BASIC WLC CODE, MODBUS, GSM
//MODES
//STANDALONE, MANUAL, MANUAL RTC, RTC

#include "numicro_8051.h"
#include <math.h> 
#define   CARRY CY
//#define		  buzzer	   P16
sbit	startrly=P3^6;
//sbit relay=P2^1;
#define chip   P02
#define enab   P03
#define ddata  P04
#define dclock P05
#define alpha  0.95

sbit toplevel=P2^0;
sbit SIMRESET=P0^1;
sbit stoprly=P3^7;
sbit crmrly=P1^2;
sbit relay2=P3^5;

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
#define CHIP_RST   TA = 0xAA; TA = 0x55; CHPCON |= 0x80;


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

//#define rtx_ctrl0 P30
#define rtx_ctrl0 P16
#define READ	1
#define WRITE 0

/*-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
							lcd initialisation values routine
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

bdata unsigned char RAMflags _at_ 0x21;
sbit menukey = RAMflags^0;
sbit inckey= RAMflags^1;
sbit deckey = RAMflags^2;
sbit sidekey= RAMflags^3;
sbit memsave = RAMflags^4;
sbit timedisplay = RAMflags^5;
sbit restart = RAMflags^6;
sbit start2 = RAMflags^7;

bdata unsigned char RAMflags1 _at_ 0x22;
sbit  voltlow = RAMflags1^0;
sbit volthigh = RAMflags1^1;
sbit mqqtstatus = RAMflags1^2;
sbit errordisplay = RAMflags1^3;
sbit phaseunbalance = RAMflags1^4;
sbit onttimewrite = RAMflags1^5;
sbit timeseg = RAMflags1^6;
sbit refresh = RAMflags1^7;


bdata unsigned char RAMflags2 _at_ 0x23;
sbit  rtcedit = RAMflags2^0;
sbit manualoff = RAMflags2^1;
sbit  phasereversed= RAMflags2^2;
sbit autosetclrshow = RAMflags2^3;
sbit autosetdone = RAMflags2^4;
sbit smode = RAMflags2^5;
sbit switchset1= RAMflags2^6;

bdata unsigned char RAMflags3 _at_ 0x24;
sbit  turnoff = RAMflags3^0;
sbit  drywrite = RAMflags3^1;
sbit  pumpsensed= RAMflags3^2;
sbit  vshow = RAMflags3^3;
sbit  displayrefresh= RAMflags3^4;
sbit  switchset2= RAMflags3^5;
sbit  switchset3= RAMflags3^6;
sbit  switchset4= RAMflags3^7;

bdata unsigned char RAMflags4 _at_ 0x25;
sbit  switchset6= RAMflags4^0;
sbit  switchset5= RAMflags4^1;
sbit  rtcstart= RAMflags4^2;
sbit  modechangedisp= RAMflags4^3;
sbit  drtstore= RAMflags4^4;
sbit  errorprint= RAMflags4^5;
sbit  checkflag= RAMflags4^6;
sbit  write= RAMflags4^7;

bdata unsigned char RAMflags5 _at_ 0x26;
sbit  rphfail= RAMflags5^0;
sbit  yphfail = RAMflags5^1;
sbit  bphfail = RAMflags5^2;
sbit  switchon= RAMflags5^3;
sbit  oftimestarterror= RAMflags5^4;
sbit  oftimererror= RAMflags5^5;
sbit  ontimererror= RAMflags5^6;
sbit  showdisp= RAMflags5^7;


bdata unsigned char RAMflags6 _at_ 0x27;
sbit   drterror= RAMflags6^0;
sbit  nopump= RAMflags6^1;
sbit  reset1= RAMflags6^2;
sbit  relayoff= RAMflags6^3;
sbit  dry= RAMflags6^4;
sbit  phasefail= RAMflags6^5;
sbit start1= RAMflags6^6;
sbit ondelayok= RAMflags6^7;

bdata unsigned char RAMflags7 _at_ 0x28;
sbit   phasewrite= RAMflags7^0;
sbit  adtransfer= RAMflags7^1;
//sbit  pon= RAMflags7^2;
sbit wlctop1 = RAMflags7^3;
sbit  pumpon= RAMflags7^4;
sbit  startcrmoff= RAMflags7^5;
sbit dscroll= RAMflags7^6;
sbit errorlog= RAMflags7^7;

bdata unsigned char RAMflags8 _at_ 0x29;
sbit moduleReadyFlag=RAMflags8^0;
sbit dtmf_on_Flag=RAMflags8^1;
sbit smsrtconFlag=RAMflags8^2;
sbit smsrtcoffFlag=RAMflags8^3;
sbit dtmf_off_Flag=RAMflags8^4;
sbit sms_off_Flag=RAMflags8^5;
sbit sms_on_Flag=RAMflags8^6;
sbit simnotokFlag=RAMflags8^7;

bdata unsigned char RAMflags9 _at_ 0x2a;
sbit sendSmsflag=RAMflags9^0;
sbit cyclicFlag=RAMflags9^1;
sbit messageFlag=RAMflags9^2;
sbit sensrFlag=RAMflags9^3;
sbit commtFlag=RAMflags9^4;
sbit usrdtFlag=RAMflags9^5;
sbit pumpstatuschange=RAMflags9^6;
sbit simokFlag=RAMflags9^7;

bdata unsigned char RAMflags10 _at_ 0x2b;
sbit okFlag=RAMflags10^0;
sbit callflag=RAMflags10^1;
sbit audioplaystopFlag=RAMflags10^2;
sbit statusdataFlag=RAMflags10^3;
sbit mqttconfigfail=RAMflags10^4;
sbit hvlvFlag=RAMflags10^5;
sbit hvlvFlag2=RAMflags10^6;
sbit drtFlag=RAMflags10^7;

bdata unsigned char RAMflags11 _at_ 0x2c;
sbit smsauto=RAMflags11^0;
sbit smsrtc=RAMflags11^1;
sbit smscyc=RAMflags11^2;
sbit smsreset=RAMflags11^3;
sbit smsaset=RAMflags11^4;
sbit smsaclr=RAMflags11^5;
sbit smssemi=RAMflags11^6;
sbit smsmanual=RAMflags11^7;

bdata unsigned char RAMflags12 _at_ 0x2d;
sbit standalonevalveon=RAMflags12^0;
sbit groupskip=RAMflags12^1;
sbit timercomplete=RAMflags12^2;

xdata const char init2[]={0x38,0xa0,0x38,0x0a,0x08,0x0a,0x01,0x0a,0x06,0x0a,0x0f,0x0a,0x80,0x0a,0x01,0x0a};

xdata const char fline1[]={nu,nu,nu,nu,cs,se,st,nu,sm,so,sd,se,nu,nu,nu,nu};
xdata const char mod[]={cs,st,sd,nu,sa,sl,sn,nu,nu,nu,nu,nu,nu,nu,nu,nu};
xdata const char mod1[]={cm,sa,sn,su,sa,sl,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu};
xdata const char mod2[]={cm,sa,sn,hi,cr,st,sc,nu,nu,nu,nu,nu,nu,nu,nu,nu};
xdata const char mod3[]={nu,cr,ct,cc,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu};
xdata const char mod4[]={nu,cs,se,sm,si,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu};
xdata const char pumpt[]={nu,nu,nu,cp,su,sm,sp,nu,ct,sy,sp,se,nu,nu,nu,nu};
xdata const char pump1[]={nu,co,sp,se,sn,sw,se,sl,sl,'/',cm,so,sn,so,nu,nu};
xdata const char pump2[]={nu,nu,nu,nu,cb,so,sr,se,sw,se,sl,sl,nu,nu,nu,nu};

xdata const char vin[]={cv,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu};
xdata const char pt[]={nu,nu,ac,nu,nu,nu,nu,cp,su,sm,sp,ac,nu,nu,nu,nu};
xdata const char va[]={nu,nu,nu,nu,cv,nu,nu,cp,su,sm,sp,ac,nu,nu,nu,nu};
xdata const char ai[]={ci,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu};
xdata const char am[]={nu,nu,nu,nu,ca,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu,nu};
xdata const char tm[]={ct,'-',nu,nu,ac,nu,nu,nu,cm,so,sd,ac,nu,nu,nu,nu};
xdata const char model[]={cm,so,sd,se,sl,nu,cs,se,sl,se,sc,st,si,so,sn,nu};
xdata const char siph[]={nu,nu,nu,cp,ch,nu,cs,se,sl,se,sc,st,se,sd,nu,nu};
xdata const char proce[]={nu,nu,cp,sr,so,sc,se,ss,ss,si,sn,sg,dp,dp,dp,nu};
xdata const char errorhv[]={nu,nu,hi,ch,si,hi,cv,so,sl,st,ac,nu,nu,nu,cv,nu};
xdata const char errorlv[]={nu,nu,hi,cl,so,hi,cv,so,sl,st,ac,nu,nu,nu,cv,nu};
xdata const char errorovl[]={nu,co,sv,se,sr,sl,so,sa,sd,ac,nu,nu,nu,nu,ca,nu};
xdata const char errordry[]={nu,nu,cd,sr,sy,sr,su,sn,ac,nu,nu,nu,nu,nu,ca,nu};
xdata const char  errorpf[]={nu,hi,cp,sh,sa,ss,se,sf,sa,si,sl,ac,nu,nu,nu,cv};
xdata const char errorph[]={nu,cp,sh,sa,ss,se,nu,cu,sn,sb,sa,sl,sa,sn,sc,se};
xdata const char errorpr[]={nu,cp,sh,sa,ss,se,' ',cr,se,sv,se,sr,ss,se,sd,nu};
xdata const char ron[]={co,sn,nu,nu};
xdata const char rof[]={co,sf,sf,nu}; xdata const char ond[]={co,cn,ct,nu,ac};
xdata const char ofd[]={co,cf,ct,nu,ac};/*xdata const char sc1[]={ct,nu,co,cn};
xdata const char sc2[]={ct,nu,co,cf};*/

xdata const char autos[]={nu,nu,nu,nu,nu,ca,su,st,so,ss,se,st,nu,nu,nu,nu};
xdata const char autoc[]={nu,nu,nu,nu,ca,su,st,so,sc,sl,se,sa,sr,nu,nu,nu};
xdata const char autocom[]={nu,nu,nu,cc,so,sm,sp,sl,se,st,se,sd,nu,nu,nu,nu};

xdata const char select[]={nu,nu,nu,nu,cs,se,sl,se,sc,st,se,sd,nu,nu,nu,nu};
xdata const char drystart[]={nu,cd,cr,ct,ac};
xdata const char  rst[]={nu,nu,cf,sa,su,sl,st,nu,cr,se,ss,se,st,nu,nu,nu};


xdata const char pn[]={nu,cr,su,sn,sn,si,sn,sg,nu,ci,sn,hi,nu,cp,ch,nu};
xdata const char  asetclr[]={nu,cs,st,sa,sr,st,ss,nu,ci,sn,ac,nu,nu,cs,se,sc};
xdata const char  no_pump[]={nu,nu,nu,nu,nu,cn,so,hi,cp,su,sm,sp,nu,nu,nu,nu};
xdata const char  servfail[]={nu,nu,cs,se,sr,sv,se,sr,nu,sf,sa,si,sl,nu,nu,nu};
xdata const char  netfail[]={nu,cn,se,sn,st,sw,so,sr,sk,nu,sf,sa,si,sl,nu,nu};

xdata const char  initialize[]={nu,nu,ci,sn,si,st,si,sa,sl,si,sz,si,sn,sg,nu,nu};
xdata const char con1[]={nu,nu,cc,so,sn,sn,se,st,si,sn,sg,nu,ct,so,nu,nu};
xdata const char con2[]={nu,nu,nu,nu,cn,se,st,sw,so,sr,sk,nu,nu,nu,nu,nu};
xdata const char con3[]={nu,nu,nu,nu,cs,se,sr,sv,se,sr,nu,nu,nu,nu,nu,nu};
xdata const char rssival[]={nu,cs,si,sg,sn,sa,sl,nu,'%',ac,nu,nu,nu,nu,nu,nu};
xdata const char errsc[]={nu,ce,sr,sr,so,sr,':',nu,nu,nu,nu,nu,nu,nu,nu,nu};
xdata const char slavetime[]={nu,nu,cs,sl,sa,sv,se,nu,ct,si,sm,se,so,su,st,nu};
xdata const char unbc[]={nu,cc,su,sr,sr,se,sn,st,nu,cu,sn,sb,sa,sl,nu,nu};
xdata const char airtel[]={nu,nu,nu,nu,nu,ca,si,sr,st,se,sl,nu,nu,nu,nu,nu};
xdata const char jio[]={nu,nu,nu,nu,nu,nu,nu,cj,si,so,nu,nu,nu,nu,nu,nu};
xdata const char vodafone[]={nu,nu,nu,nu,cv,si,nu,ci,sn,sd,si,sa,nu,nu,nu,nu};
xdata const char bsnl[]={nu,nu,nu,nu,nu,nu,cb,cs,cn,cl,nu,nu,nu,nu,nu,nu};
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
						    variable declaration routine
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
volatile unsigned char xdata page_buffer[128];
unsigned char rdata;
xdata unsigned int counter=0,i=0,result1=0,intref=0,result2=0,vref2=0,bufferx=0,buffer=0,rtctime=0,ontime=0,offtime=0,run2=0,ontimeapp=0,offtimeapp=0,drtapp=0;
unsigned char adcome1=0,displayx=0,t3=0,channel=0,k=0,address=0,temp=0,dbuf[7];
char digit1[7],cblink=0;
double resultx1=0;
xdata unsigned int seting_group[25]=0;
xdata unsigned char high=0,low=0;
xdata unsigned int addr=0;
xdata unsigned char datah=0,datal=0,controlstate=0;
xdata unsigned char user_smsdis[15]=0;
xdata unsigned long result=0;//,rtcdate=0,bufferr=0
xdata unsigned char useradd=0,base=0,appreceived=0,appreceived1=0,ibm=0,reg=0,reg7=0,r3=0,h=0,mpress=0,setpress=0,smenu=0,ontimeh=0,ontimel=0,offtimeh=0,offtimel=0,run2l=0,run2h=0;
xdata unsigned char timeron=0,byte=0,bytei=0,resetdata1=0,resetdata=0,limit=0,mode=0,disdat=0,datachang=0,blink=0,chang=0,cursor=0,spress=0,incpress=0,menupress=0,pumptype=0;
xdata unsigned char rtccredit=0,rtcmin=0,rtchour=0,count=0,eemem=0,buff=0,txbuf=0,datai=0,buf=0,timeok=0,curmin=0,curhour=0,runningmin=0,runninghour=0;
xdata unsigned int scan=0,scanx=0,dispr=0,start=0,run7=0,run3=0,resultt=0,combinevalue=0,runningtime=0,runningtimex=0,ontimex=0,offtimex=0,presenttime=0,futuretime=0,ontime1set=0,ontime1setx=0,offtime1setx=0;//y=0,year1=0,year2=0,month2=0,month1=0,yeardays=0,totalpass2=0,totalpass1=0,totalpass3=0,totalpass4=0,day2=0,day1=0,totaldays=0,
xdata unsigned char phscanx=0,phaseselection=0,setmode=0,phscan=0,limitx=0,start4=0,run3h=0,run3l=0,run8=0,flt=0,wtr=0,wtrmr=0,full=0,empty=0;//rtcdate1=0,rtcmonth=0,rtcyear=0,date=0,mon=0,yr=0,dayon=0,
xdata  signed long int resultx=0;
xdata unsigned long int xdata result5=0,result6=0,store1=0;
xdata unsigned int result4=0,icur2=0;
xdata signed int  resultxvolt=0,resultxcurrent=0,resultvolt=0,resultcurrent=0,resultr=0,resulty=0,resultb=0,dispry1=0,dispyb1=0,dispbr1=0;
xdata signed long int  resultry=0,iwatt=0,icur=0,result3=0,volt=0,cur=0;
xdata unsigned char dryrestart=0,sense=0,chkonce=0,autosetcount=0;
xdata unsigned int dat4=0,modechangingsubcount=0,errtime=0,errdate=0,dispcerror=0,errorvalue=0,errorvaluex=0,errorvalue1=0,runningdaymonth=0,msec=0,avgvolt=0,ppdiff=0,ppvolt=0,pppercent=0,rypercent=0,ybpercent=0,brpercent=0,brdiff=0;
xdata unsigned char drth=0,drtl=0,m=0,date=0,mon=0,rtcdate1=0,rtcmonth=0,rtcyear=0,yr=0;
xdata unsigned char disphighh=0,disphighl=0,displowh=0,displowl=0,dryh=0,dryl=0,disperror=0,errt=0,ipress=0;
xdata unsigned char autosetcountsubcount=0,modedisp=0,dryrestartcount=0,twodigitdisp=0,hour=0,minute=0,edate=0,emonth=0,errordisplay1=0;
xdata unsigned char x3=0,countlcd=0,autosetdonesubcount=0,autosetclrshowsubcount=0,autosetclrshowcount=0,autosetclrprocesssubcount=0,autosetclrprocessmain=0;
xdata unsigned int setcurrent=0,setvoltage=0,setcurrent1=0,setvoltage1=0,setvoltagex1=0,setvoltagex2=0,setvoltagex3=0,rphasesetvolt=0,yphasesetvolt=0,bphasesetvolt=0;
xdata unsigned char  eepromtest=0,runningdate=0,runningmonth=0,runningyear=0,curdate=0,curmonth=0,curyear=0;
xdata unsigned char phasenumber=0,errorvalueh=0,errorvaluel=0,oftimestart=0; 
xdata unsigned char switchoffsubcount=0,switchoffcount=0,sidepress=0,setkey=0,incpress1=0,scrolltype=1;
xdata unsigned int y=0,pwmvalue1=0,pwmvalue1x=0;
xdata unsigned char resetx=0,ulimit=0;
xdata unsigned int resetcount=0;
xdata unsigned char crmon=0,adctime=0;
xdata unsigned int resultct=0,resultct1=0,j=0,rampsreference=0,yampsreference=0,bampsreference=0,rctval1=0,yctval1=0,bctval1=0,doad=0,rct1=0,yct1=0,bct1=0;
xdata unsigned int disp4=0,ovtime=0,initialovl=0,dtime=0,brvolt=0,ybvolt=0,ryvolt=0,disp2=0,ovlisl=0,ovlptg=0;
xdata unsigned char pumpsensingcount=0,crmoffcount=0,startrlyoffcount=0,stoprlyoffcount=0,pumpoff=0,address_store=0;
xdata unsigned char phrtime=0,phfail=0,testcount=0,asetpress=0,aclrpress=0,c=0,onrt=0,offrt=0,nokeysubcount=0,nokeys=0,cttime=0,vfault=0,phasenumber1=0;
xdata unsigned int rphvolt1=0,yphvolt1=0,bphvolt1=0,valveoffcount=0;
xdata unsigned int ondelaycheck=0,Nop=0,hvmax=0,hvmin=0,ev=0,timeoutx=0,drt1=0,ovl1=0,dryrun1=0,setreset=0,rvolt=0,yvolt=0,bvolt=0,highvolt=0,lowvolt=0,overload=0,dryx=0;
xdata unsigned char groupok=0,swapgroupvalue=0,errorclearflag=0,dryreadl=0,dryreadh=0,asetkey=0,aclrkey=0,autosetclrprocess=0,amps=0,timesegstart=0,edit=0,switchtime=5,switched=0,relayoffcount=0,relayoncount=0,rphasefailcount=0,yphasefailcount=0,bphasefailcount=0,pumpoffclear=0;
xdata unsigned char autosetclrstart=0,ontoff=0,reenter=0,valve_number=0,usrdtFlag1=0,unbalancecurrent=0,ubc1=0,motoroff=0,powereepromread=0;
xdata uint8_t d1=0,m1=0,y1=0;
xdata unsigned int futuretimedays=0,futuretimehr=0,futuretimemin=0,futuretime1=0;
///dashboard variables
xdata unsigned int maxvalves=0,dispry=0,dispyb=0,dispbr=0,rphvolt=230,yphvolt=230,bphvolt=230,rctval=0,yctval=0,bctval=0;
xdata unsigned char dashboardsignal[55]=0;//phase,BPGsm,GSignal,BPL1 -16,BPS1-16,SSl1-16,pumprunning status

xdata unsigned char programFlag=0,contoserflag=0;
xdata unsigned int cnt=0,stscnt=0,adrs=0;
xdata unsigned char receivedData[710] = {0},schedule_data[31]=0,valve_group[153]=0,group_ontime[68]=0,recnumberon_incoming[11]={0},message[150]={0},MODESTRING[10]={0},inttostring[80]={0},cyc_buf[3]={0},rtc_buf[4]={0},recframe[500]={0},max_liters[4]={0};
xdata unsigned char uart_receive_data=0,v=0,programsendFlag=0;
xdata unsigned char prev_group=0,groupinc=1,calok=0,flag = 0,selectFlag=0,dtmf_verbalvolt_Flag=0,sms_NM1_Flag=0,sms_NM2_Flag=0,sms_NM3_Flag=0,sms_NM4_Flag=0,ovld=0,sms_NM5_Flag=0,
			sms_NM6_Flag=0,semierror=0,rctflag=0,sendsettingflag=0,simcomrstcount=0,netfail1=0,
			user=0,mqttmodrecFlag=0,oneringmode=1,netdisconnected=0,mqphrcheck=0,pofrflag=0,rtcupdateflag=0,mqpofr=0,poffsensed=0,sendsettingsupdated=0,
			poweroff=0,sentpoweroffstatus=0,imeiError=0,mqautosetdone=0,mqautoclrdone=0,user1calldis=1,user2calldis=1,user3calldis=1,user4calldis=1,user5calldis=1,user6calldis=1,
			superFlag=0,maxuser=0,poweroffcount=0,poweroffcount1=0,poff=0, powersensed=0,
			rtcFlag=0,poweronreoot=0,poweroncount=0,powersense=0,netconnected=0,authuser=0, sendonlyonce=0,netstillnotconnected=0,
			nm1_count=0,nm2_count=0,nm3_count=0,nm4_count=0,nm5_count=0,nm6_count=0,a1_count=0,call_recFlag=0,dryrFlag=0,dryrFlag2=0,ovlFlag=0,ovlFlag2=0,dskpFlag=0,intlrefFlag=0,
			netopenFlag=0,mqttsendFlag=0,mqttrecFlag=0,dashboardFlag=0,mqttcompFlag=0,pubcompFlag=0,subcompFlag=0,settingFlag=0,errFlag=0,mqttdataFlag=0,errorFlag=0,statusFlag=0,manualmodeFlag=0,initFlag=0,onlyonceFlag=0
			,smqttmodrecFlag=0,rtdataFlag=0,netdisconnect=0,poweronOnce=0,poweroffOnce=0,powerOn=0;
xdata unsigned char errorpercent=0,MQTT_PHR=0,itr1=0,itr2=0,MQTT_ONTIME=0,MQTT_OFTIME=0,MQTT_POFR=0,MQTT_PHENB=0,MQTT_RESET=0,MQTT_ACLR=0,MQTT_ASET=0,MQTT_SCH1=0,MQTT_SCH2=0,MQTT_SCH3=0,MQTT_SCH4=0,MQTT_SCH5=0,MQTT_SCH6=0,MQTT_P1ON=0,MQTT_P2ON=0,LON=0;
xdata unsigned char pumponflag=0,lightonflag=0,phr=0,manualreset=0,autoset,autoclr=0,twophase=0,prog1=0,prog2=0,prog3=0,prog4=0;
xdata int payloadsize=0,initialize_timeout=2500;
xdata int network_timeout=0;
xdata unsigned char motorstate=0,STAT1=0,STAT2=0,STAT3=0,STAT4=0,MQTTERROR=0,MQTTERROR2=0,MQTTERROR3=0,MQTT_POWER=0;
xdata int idx=0,a=0,x=0,z=0	,itr = 0,q=0,n=0,poweravailable=0;
xdata unsigned char  NM1[12]={0},NM2[12]={0},NM3[12]={0},NM4[12]={0},NM5[12]={0},NM6[12]={0},A1[10]={0},num[10]={0},imei_buf[16]={0};
xdata int phselec=2,ry=320,yb=324,br=328,rn=230,yn=102,bn=115,ra=112,ya=198,ba=109,mqtt_timeout=0;	
xdata int temp1_array[5]={0},temp_array[5]={0},powersensecount=0,temp_timearray[5]=0,pumpstatuschange_timeout=0;
xdata char *numbers[7] = {A1, NM1, NM2, NM3, NM4, NM5, NM6};// Array of pointers to 10-digit strings
xdata int counts[7] = {0};
xdata int users[7] = {10, 1, 2, 3, 4, 5, 6}; 
xdata uint8_t *flags[7] = {&superFlag, &sms_NM1_Flag, &sms_NM2_Flag, &sms_NM3_Flag, &sms_NM4_Flag, &sms_NM5_Flag, &sms_NM6_Flag};
xdata unsigned int values[9];
xdata uint16_t baseAddresses[7] = {62501, 62511, 62521, 62531, 62541, 62551, 62561};
xdata const char *useraddedmessages[7] = {
    "Super admin added", "User 1 added", "User 2 added",
    "User 3 added", "User 4 added", "User 5 added", "User 6 added"
};
xdata unsigned char indexChar = 0,counter1=0,cycle=0,mode2=0,mode3=0,mode4=0,cycle1=0,slaveerror=0,slavenoresponse=0;
xdata int userIndex = 0,digits[3]={0}; 
xdata char valve_state[20]=0,bit_index=0,byte_index=0,ract_str[8]={0},yact_str[8]={0},bact_str[8]={0},statuscnt=0,MQTT_NOPUMP=0,mode1=0;  // For 3-phase formatted strings
xdata unsigned int currunninggrp=0;
xdata char pumponflag1=0,pon=0,valvetrack=0,ic_reset=0,network=0,apnnotdetect=0,pdpnotdetect=0;
//////////////////////////////////modbus///////////////////////////////////////////////////////////////////////

xdata unsigned char primebyte[30],store=0;
xdata char regx,xstartx,data1,get,content,lowCRC,highCRC,transfer=0,startmod=0;
xdata char xstopx,generate,value,slaveaddress,buff1,buff2,store2;
xdata volatile char rxdata;

xdata unsigned char tx_buf[75]={0};
xdata unsigned int mbreaddata = 0;
xdata unsigned int checksum,checksum1;
xdata unsigned long int wait=0;

xdata unsigned int mbdata=5;
xdata unsigned char mbreadwrite=0;
xdata unsigned char datalength=0,errcheckroutine=0;

xdata unsigned char writefailure=0,turnonv=0,slavestat=0,slaveerrornumber=0,slavenumber=0;
xdata char tmpflag=0,divval=0,modval=0,turnoffv=0,writevaluestate=0,groupincwrite=0,cyclestore=0,motoron=0,groupskipwrite=0,groupskipcheck=1;
xdata unsigned int slavedata[16]={0};
xdata unsigned char modechecker1=0,modechecker2=0,modechanged=0,pumpoffclearcount=0,skipstart=0,twobitsend=0,progmstore1=0,progmstore=0,apprespondFlag=0;
xdata unsigned char AKCREG1=0,AKCREG2=0,respondtoappFlag=0,appopen=0,RESENDFRAME1=0,RESENDFRAME2=0,datainvalid=0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
void rtcfunction(void);
void valveonofffunction(void);
void adcconv(void);
void lcdinit(void);
void enabler(char t4);
void switchscan(void);
void Write_DATAFLASH_BYTE1(unsigned int u16EPAddr, unsigned char u8EPData);
unsigned char Read_APROM_BYTE(unsigned int code *u16_addr);
void fline(void);
void sline(void);
void splitl(void);
void valuewritex(void);
void readdata(void);
void datawrite(void);
void split2(void);
void decrout(void);
void display(void);
void pumponoffmessagefunction(void);
void pumponfunction(void);
void pumpoffunction(void);
void twophaseonfunction(void);
void ontimerfunction(void);
void oftimerfunction(void);
void faultresetfunction(void);
void displayscrolltime(void);
void autosetclrfunction(void);
void drtstartfunction(void);
void powersupplysensefunction(void);
void mqqtupdatetime(void);
void dashboardcal(void);
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
void autosetclr(void);
void dataout(void);
void errordisplayprint(void);
void pwm_init(void);
void Pwmduty(unsigned int dutycycle);
void readreference(void);
void ctscan(void);
void curprob(void);
void drycuttoff(void);
void ovlcuttoff(void);
void curdecide(void);
void pwmfunction(void);
void timesegread(void);
void zc(void);
void threedigit(void);
void threedigita(void);
void fourdigit(void);
char checkvolt(unsigned int v);
int compare(int comp1, int comp2);
void checkctvalues(unsigned int rct1, unsigned int yct1,unsigned int bct1);
void checkvoltvalues(unsigned int ry1, unsigned int yb1,unsigned int br1);
void unblanceampscut(void);
void lightonfunction(void);
void valvedelete(void);
void sendSmsToUse(const char *smsData, const char *phoneNumber);
void timeafterset(void);
/////////////////////////////////////////////////////modbus//////////////////////////////////////////////////////////
void modbusUartConfig(void);
void receivedata(void);
unsigned int readMBframe(void);
bit checkRxbuffer(void);
bit CRC16(unsigned char startaddress, unsigned char dataLength,char check);
void readResponse(void);
void delayMs(unsigned int ms);
void modbusCommunication (char readorwrite);

/********************************************************gsm functions**************************************************************************************************/
void gsm_uartinit(void);
void UART_Send_String(const char *dt);
void UART_Send(unsigned char c);
void processData(void);
void reset_recindex(void);
void mqttpush(void);
void value_split(int val);
void readdashboarddata(void);
void check_gsmCommands(void);
void simcom_audio(const char *value);
void sendSms(const char *smsData);
void update_settings(void);
int combine_setting(int dat1,int dat2,int dat3);
void update_staus(void);
void update_error(void);
void gsm_reconnect(void);
void mqtt_reconnect(void);
void modechangemqtt(void);
void checkok_error(void);
void connect_tomqtt(void);
void display_contoserver(void);
void checkfor_authoriseduser(void);
void mqtt_disconnect(void);
void servertocontroller(void);
void checkforlessthansymbol(void);
void sendpumpstatus(void);
void gsm_routine(void);
void display_contointernet(void);
void controllertoserver(void);
void onlywhenmodulereset(void);
void value_split(int val);
void sendStatusBits(int statusReg);
void rst_simcom(void);
void setting_split(void);
void settingschanged(void);
void reset_messageindex(void);
void sendMQTTTopic(const char* topicSuffix);
void sendMQTTPayloadLength(int len);
void int3_to_string(char *dest, int a, int b, int c);
void int2_to_string(char *dest, int a, int b );
int combine2digits(char d1,char d2);
void addUser(uint8_t flagIndex, char *destBuf, uint16_t baseAddr, const char *msg);
void rtcschedulercheck(void);
void allvalveoff(void);
void slavecheck(void);
void addvalveinslave(void);
void valveon(void);
void statusread(void);
void poweronfaultread(void);
void slaveerror_check(void);
void powereepromreadfunction(void);
void SerialPort1_ISR(void) interrupt 15
{
   _push_(SFRS);
   SFRS=0;
   if (RI_1==1)
       {
			uart_receive_data = SBUF_1;
			receivedData[itr] = uart_receive_data;
			processData();	
			if(uart_receive_data == '\n' && receivedData[itr-1] == '\r')
			{
				reset_recindex();//Clear receivedData buffer.
			}
			else 
			{
				itr++;
			}
					
				/*uart_receive_data = SBUF_1;
        receivedData[itr] = uart_receive_data;	
        if(uart_receive_data == '\n' && itr>0 &&receivedData[itr-1] == '\r')
        {
					processData();
          reset_recindex();//Clear receivedData buffer.
        }
        else 
        {
            itr++;
        }*/
        clr_SCON_1_RI_1;  
       }
_pop_(SFRS);
}

void main()
{
init();
nopump=0;
turnoff=1;
wlctop1=0;
phasewrite=1;
//buzzer=0;
edat=0;
eclk=0;
ddata=0;
dclock=0;
startrly=0;
stoprly=1;
crmrly=0;
CTMUXDIS;
adc_init();
pwm_init();
lcdinit();					
timer_init();
delayMs(1000);            //delay given for circuit voltage stabilisation
powereepromreadfunction();//it check device input it battery or EB 
delayMs(1000);            //delay given for circuit voltage stabilisation
if(powereepromread==1)    //delay given for circuit voltage stabilisation
{
delayMs(1000);
delayMs(1000);
}
if(powereepromread==1)  // if input voltage is EB powered then only it will execute 
{
pwmvalue1x=800;         // pwm for white led 
Pwmduty(800);
for(i=0;i<500;i++)      // switchscan for model selection 1 PH or 3 PH 
{
	switchscan();
	if(vref2<530&&vref2>460)
	{
	phaseselection=1;
	}
}

while(phaseselection==1)// this 1 PH or 3 PH model decide Loop
{
		dashboardsignal[0]=3;
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
		if(dispyb1<((dispry1*60)/100))
		{
			dashboardsignal[0]=1;
		}
		else
		{
			dashboardsignal[0]=3;
		}
  		if(phasewrite==1)
  		{
			phasewrite=0;
			IE&=~(1<<7);
			Write_DATAFLASH_BYTE1(61049,dashboardsignal[0]);// model storing in eeprom 
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
			enabler(dashboardsignal[0]+0x30);
			delay(3);
			chip=0;
			enabler(0x0c);
			delay(0x0a);
		}
	}
}
gsm_uartinit();     // uart init 
modbusUartConfig(); // modbus uart configuration  
pwmvalue1x=800;
Pwmduty(800);
if(powereepromread==1)
{
	timeseg=1;         // RTC real time read trigger Flag 
}
fline();           //1st line 
for(z=0;z<16;z++)
{
	enabler(initialize[z]);// initialize display print section 	
}
chip=0;
enabler(0x0c);
delay(1);
chip=1;
rst_simcom();
mqtt_timeout=0;
sline();
while(!moduleReadyFlag && mqtt_timeout<2500 && !simnotokFlag);
if(mqtt_timeout>2450)
{
    simnotokFlag=1;	
	simokFlag=0;
}
mqtt_timeout=0;
lcdinit();
if(simnotokFlag==1)
{
	simokFlag=0;
	EIE1 &= ~(1<<0);			
}
if(simokFlag==1)
{	
	gsm_reconnect();           // gsm connecting to network and server function 
}
counter=0;
displayx=1;
refresh=1;
	powereepromreadfunction(); // again confirm input voltage is battery or EB
	if(powereepromread==1)     // if input is EB powered then only it will enter this loop 
	{
		resultx=0;
		result1=0;
        readdata();//eeprom read 
	    statusread();//status bit read 
        poweronfaultread();// power on fault read 
		if((turnonv==1&&cycle1!=0&&(mode2==1||mode3==1))||(mode1==1&&turnonv==1))// if previously pump is on state then only it will execute 
		{
			pumponflag=1;
			timercomplete=0;
		}
		readreference();// CT reference read 
		if(dashboardsignal[0]==3&&twophase==1)// this loop check input voltage is 3 PH or 2 PH
			{
		for(phscanx=0;phscanx<12;phscanx++)
			{
				adcome1++;  //adcome1 is for phases
				if(adcome1>3)
				adcome1=1;
				ctscan();
			}
				if(rphvolt1>100&&yphvolt1>100&&bphvolt1<50)
				{
					dashboardsignal[1]=1;
					switched=1;
				}
			}
		}
		fline();// 1st line 
		while(1)
		{
		while(smode==0&&errorlog==0)// run mode start  here 
		{	 
		powersupplysensefunction(); //this function check input power supply is EB or battery
		if(poff==1 && powersense==1)
		{
			poff=0;
			powersensed=0;
			CHIP_RST;	 
		}
		if(simokFlag==1)
		{
		gsm_routine();//gsm function 
		}
		if(powersense==1 && poff==0)// if input power is EB then this function will execute  
		{
			if(pon==0)
				stoprly=1;
			switchscan();	
			adcome1++;
			if(adcome1>6)
				adcome1=1;	
			analog();
			ctscan();
			switchscan();		
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
				
			
				
				if(rctval<10)//lesser than 1 A it the device will set zero amps to avoid unwanted data print in display in amps section 
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
				
		if(sense<3)
			sense++;
		pumponoffmessagefunction();
		}
		if(rctval>=10)
			Nop=0;

		if(sense>0&&switched==0)
			phrcheck();
					
		if(sense>1&&switched==0)// once sense the input voltage and current then only this loop will execute due to avoid un wanted trip 
		{
			decide();//voltage related error handling function 
			curdecide();// current related error handling function
			checkctvalues(rctval,yctval,bctval);//current unbalance error handling function 
			vcutoff();//voltage related error cutoff function 
			drycuttoff();//dry error cutoff function
			ovlcuttoff();//ovl error cutoff function
			unblanceampscut();//unbalancecurrent error cutoff function
		}

		if(cnt!=3&&cnt!=6&&cnt!=9)
		{
			timesegread();//RTC time read counter function 
		} 
			////////////////////////////////////////////////error display print section start////////////////////////////////  
		if((((volthigh==1||voltlow==1||phasereversed==1||phaseunbalance==1||phasefail==1)&&(start2==1))||((dry==1||ovld==1||nopump==1||slaveerror==1||unbalancecurrent==1)&&restart==1)))
		{
				errordisplay=1;
				if((errordisplay==1&&dryrestart==0))
				{
					displayx=1;
				}
				if(displayrefresh==0)
				{
					refresh=1;
					displayrefresh=1;
				}
			}
			//////////////////////////////////////error display print section end ///////////////////////////////////////
			////////////////////////////////////constant sting print in display section//////////////////////////////////	
			if(refresh==1)
			{
				refresh=0;
				display();
			}

	///////////////////////////////////volatge- current-time- non constant string printing section/////////////////	
	if(resetx==0&&modechangedisp==0)
         valuewritex();
		
	pwmfunction();         // pwm generate function
    pumponfunction();      // 3&1 PH pump on function
    pumpoffunction();      // 3&1 PH pump off function
    twophaseonfunction();  // 2 PH capacitor on&off function
 	oftimerfunction();     // drt decrement timer function 
    faultresetfunction();  //fault reset function 
	mqqtupdatetime();      //push the data to the server 
    displayscrolltime();   //display scroll function 
    autosetclrfunction();  //auto set/clear function 
	drtstartfunction();    // dry restart timer trigger function
    valveonofffunction();  // valve on and off through the modbus perform this function only 
	lightonfunction();     // light on and off function 
	
	if(switchon==1)        // pwm load for white led glow 
		pwmvalue1=800;

	if(switchon==1)//whiteled ON for menukey press
	{
		   switchoffsubcount++;
		if(switchoffsubcount>20)
		{
			switchoffsubcount=0;
			switchon=0;
		}
	}
      }
     }//runmode ends
	}
}
/*****************************Interrupt*******************************************************/
void Timer() interrupt 3
{
    _push_(SFRS);
 	SFRS=0;
    TF1=0;
    TH1=0xcb;
    TL1=0xea;
	
	//buzzer=!buzzer;

	if(motoroff==1)
		valveoffcount++;

	if(valveoffcount>500)
	{
		motoroff=0;
        valveoffcount=0;
	    turnoffv=1;
	}
	
	if(powersense==1 && poff==1 && powersensed == 0)
	{
		 powersensecount++;
		 if(powersensecount>100)
		 {
			 powersensecount=0;
			 powersensed=1;
		 }
	}
	 
	if(reset1==1&&smode==0&&start2==0&&restart==0&&volthigh==0&&voltlow==0)
		            start4++;
	if(start4>99)
	{
		start4=0;
		ondelaycheck++;
	}
	
	if(ondelaycheck>=(seting_group[13]+7))
	{
		 stoprly=0;
		 ondelaycheck=0;
		 ondelayok=1;
	}
    
	if(rctval>=10||yctval>=10||bctval>=10)//initial don't care
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
	
	if(pon==1&&rctval1<10&&nopump==0&&start2==0&&restart==0&&smode==0&&ondelaycheck==0)
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
			onlyonceFlag=1;
		}
	}
	
	if(timeseg==1&&rtccredit==0)
	{
		timeseg=0;
		timprob();  
		timread();
	}
	
	if(rtccredit==1)
	{
		rtccredit=0;
		rwrbyte1(0x80,0x00);
		rwrbyte1(rtcmin,1);
		rwrbyte1(rtchour,2);
		rwrbyte1(0x00,0x00);
		timprob();
	}
	
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

	if(memsave==1)
	{
		datawrite();
		//sendsettingflag=1;
		memsave=0;
	}
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	if(pofrflag==1)
	{
		errorvalueh=errorvalue1/256;
		errorvaluel=errorvalue1%256;
		Write_DATAFLASH_BYTE1(61040,ovld);
		Write_DATAFLASH_BYTE1(61041,errorvalueh);
		Write_DATAFLASH_BYTE1(61042,errorvaluel);	
		pofrflag=0;
	}

	if(semierror==1)
	{
		semierror=0;
		Write_DATAFLASH_BYTE1(61039,ontoff);
	}

	if(ontimererror==1)
	{
		ontimererror=0;
		run2h=((run2>>8)&(0x00ff));
		run2l=(run2&0x00ff);
		Write_DATAFLASH_BYTE1(61064,run2h);
		Write_DATAFLASH_BYTE1(61065,run2l);
		//Write_DATAFLASH_BYTE1(61003,oftimestart);	
	}

	if(drtstore==1)
	{
		drtstore=0;
		run3h=((drt1>>8)&(0x00ff));
		run3l=(drt1&0x00ff);
		Write_DATAFLASH_BYTE1(61060,run3h);
		Write_DATAFLASH_BYTE1(61061,run3l);
	}

	if(drywrite==1)
	{
		drywrite=0;
		dryreadh=errorvalue1/256;
		dryreadl=errorvalue1%256;
		Write_DATAFLASH_BYTE1(61056,dry);
		Write_DATAFLASH_BYTE1(61062,dryreadh);
		Write_DATAFLASH_BYTE1(61063,dryreadl);
	}

	if(onttimewrite==1)
	{
		onttimewrite=0;
		Write_DATAFLASH_BYTE1(61016,timeron);
	}

	if(drterror==1)
	{
		drterror=0;
		Write_DATAFLASH_BYTE1(61059,dryrestart);
	}

	if(writevaluestate==1)
	{
		writevaluestate=0;
		Write_DATAFLASH_BYTE1(61047,turnonv);
	}

	if(groupincwrite==1)
	{
		groupincwrite=0;
		Write_DATAFLASH_BYTE1(61046,groupinc);
	}

	if(groupskipwrite==1)
	{
		groupskipwrite=0;
		Write_DATAFLASH_BYTE1(61050,groupskip);
	}

	if(cyclestore==1)
	{
		cyclestore=0;
		Write_DATAFLASH_BYTE1(61038,cycle1);
		Write_DATAFLASH_BYTE1(61043,cycle);

	}

	if(resetcount!=0&&resetx==1)
	{
		resetcount--;
	}

	if(smode==1)
		setreset++;

	mqtt_timeout++;
	if(mqtt_timeout>65500){
		mqtt_timeout=0;
	}

	network_timeout++;

	if(network_timeout>65500)
	{
		network_timeout=0;
	}

	_pop_(SFRS);
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

void portinit(void)
{
	SFRS=0;
	P0M1=0x01;
	P0M2=0xFE;
	P0=0X00;
	P1M1=0x12;
	P1M2=0xED;
	P1=0X11;
	
	SFRS=2;
    P2M1=0x27;
    P2M2=0xd8;
    P21_INPUT_MODE;
    P2=0x21;
    SFRS=0;
 
	//enable& buzzer
	SFRS=0;
	P3M1=0x01;
	P3M2=0xFe;
	P3=0x00;
    SFRS=0;
}

void timesegread(void)
{
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
	zc();
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
		if(dashboardsignal[0]==1)
    dispry1=icur/1.099;//1.82;//1.05
		if(dashboardsignal[0]==3)
    dispry1=icur/1.107;//1.82;//1.05
	}
	if(adcome1==2)
	{
  		dispyb1=icur/1.040;//1.839;//1.05
	}
	if(adcome1==3)
	{
  		dispbr1=icur/1.048;//1.842;//1.08
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
			channel=0x0f;
			intref=512;
		}
        if(adcome1==2)
		{
			channel=0x0e;
			intref=512;
		}
        if(adcome1==3)
		{
			channel=0x07;
			intref=512;
		}
		if(adcome1==4)
		{
			RCTMUXDIS;
			intref=rampsreference;
			channel=0x0A;
		}
        if(adcome1==5)
		{
		    YCTMUXDIS;
			intref=yampsreference;
			channel=0x0A;
		}
        if(adcome1==6)
		{
			BCTMUXDIS;
			intref=bampsreference;
			channel=0x0A;
		}
		adcconv();
		if(result1>=intref)
  			result2=result1-intref;
        if(result1<intref)
  			result2=intref-result1;
		timeoutx=((TH0<<8)|TL0);
		if(timeoutx>38869)
		{
			TR0=0;
			TF0=0;
			result2=0;
		}		
	}	
}

void ctscan(void)
{
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
	    if(adcome1==4)
		{
			RCTMUXDIS;
			channel=0x0A;
		}
        if(adcome1==5)
		{
			YCTMUXDIS;
			channel=0x0A;
		}
  		if(adcome1==6)
		{
			BCTMUXDIS;
			channel=0x0A;
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
		rphvolt1=icur2/2.652;
	}
	if(adcome1==2)
	{
		yphvolt1=icur2/2.700;
	}
	if(adcome1==3)
	{
		bphvolt1=icur2/2.440;
	}
	if(adcome1==4)
	{
		rctval1=icur2/1.356;//1.82;//1.05
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
		yctval1=icur2/1.386;//1.82;//1.05
  		if(yctval1<10)
	 		 yctval1=0;		
	}
	if(adcome1==6)
	{
		bctval1=icur2/1.377;//1.82;//1.05
		if(bctval1<10)
			bctval1=0;	
	}
}

void adcconv(void)
{
	ADCCON0=channel;
	for(j=0;j<=10;j++);
	clr_ADCCON0_ADCF;//
	set_ADCCON0_ADCS;//                  
	while(ADCF == 0);//
	result1=(ADCRH<<4)+(ADCRL&0x0F);//
	result1=result1>>2;	
}

void valveonofffunction(void)
{
	//mode1=>standalone
	//mode2=>manual cyclic
	//mode3=>manual RTC
	//mode4=>RTC 
	if(modechanged==1&&motoron==1&&pon==1)// when pump is run & mode is changed this loop will execute
	{
	    relayoff=1;
	    timercomplete=0;
	    //turnoffv=1;
	}

	if(modechanged==1&&pon==0&&motoron==0&&turnoffv==0&&valveoffcount==0)// when pump is off & mode is changed this loop will execute
	{
	    modechanged=0;
	}

	if(pumpoffclear==0&&modechanged==0&&valveoffcount==0)
	{
		///////////////standalone mode function start//////////////////////////////////////////////////////
		if((standalonevalveon==0)&&(mode1==1)&&(pumponflag==1))//valve on when standalone mode is selected based on simcom data
		{
	    	slavecheck();
			valvetrack=0;
			for (byte = 0; byte < 20; byte++) 
			{
				for (bytei = 0; bytei < 8; bytei++) 
				{
					if (valve_state[byte] & (1 << bytei)) 
					{
						valvetrack=1;	
						valve_number = byte * 8 + bytei + 1;  // 1..250  //need to ON only this valve_number remaining all will be 0  					   
						addvalveinslave();  
					}
			    }
		    }
	
/////////valve on////
	motoron=0;
	if(valvetrack==1)
		valveon();
//////set pump on flag
	if(slavenoresponse==1)
		slaveerror_check();
	
	if(slavenumber!=0&&turnonv==0&&slaveerror==0)
	{
			turnonv=1;
			writevaluestate=1;
	}
	//motoron=1; after acknowledgement received
	standalonevalveon=1;
	//no-error
	if(slaveerror==0&&slavenumber!=0)
		     motoron=1;
 	}
	////////////////////standalone mode end ////////////////////////////////////////////

	if((mode1==1&&pumponflag==0&&turnonv==1&&motoron==1)||((mode1==1||mode2==1||mode3==1||mode4==1)&&pumponflag==1&&motoron==1&&valvetrack==0))
	{
		//  there is no valve but user try to turn on the pump this function will block it
		valvedelete();
		relayoff=1;
	}
	//////////////////////////////////////////
	if(mode3==1||mode4==1)
	{
//it perform only RTC based on / off
		rtcschedulercheck();
	}
	////////////////////////////manual cyclic & manual RTC mode function start here//////////////////////////////
	if((((mode2==1||mode3==1))&&(pumponflag==1)&&(cycle1>=1)))
	{
        if(groupskip==1&&timercomplete==0&&groupskipcheck==0)//skip group
		{
			groupinc=seting_group[14];
			groupskip=0;
			groupok=1;
			swapgroupvalue=seting_group[14]=0;
			groupincwrite=1;
			groupskipwrite=1;
			memsave=1;
		}		
		if(timercomplete==0)
		{
		valvedelete();
/////////////slavecheck///////////////////////////
		slavecheck();
////////////////////////////////////////			
		if(run2==0)
		{
				run2=group_ontime[groupinc-1];
				if(schedule_data[1]==1)
					run2=(((group_ontime[groupinc-1]/10)*60)+((group_ontime[groupinc-1]%10)*10));
				ontimererror=1;		 
		}
		if(groupinc<=schedule_data[2])
		{
		  	 valvetrack=0;
   			 for(i=0;i<maxvalves;i++)
  	 		 {
			    byte_index=i/8;
          		bit_index=i%8;
			    if(valve_group[i]==groupinc)
				{
					valvetrack=1;					
					valve_number=i;  // Turn ON valve	
					valve_state[byte_index]|=(1<<bit_index);	
				    addvalveinslave();	
				}		
		     }
	    }
        motoron=0;	
        groupskipcheck=0;	
		timercomplete=1;
		/////////valve on////
		if(valvetrack==1)
		    valveon();
		if(slavenoresponse==1)
			slaveerror_check();
		if(slavenumber!=0&&turnonv==0&&slaveerror==0)
		{
				turnonv=1;
				writevaluestate=1;
		}
		//set pump on flag
		//no-error		
			if(slaveerror==0&&slavenumber!=0)
		  		motoron=1;
		}
		if(start2==0&&restart==0&&rctval>=10)		
		    run7++;

		if(run7>=145)//158
		{
			run7=0;
			run8=run8+1;
		}

		if(run8==3)
		{
			run8=0;
			if(run2>0)
	    		run2=run2-1;
			ontimererror=1;
		}
	    if(run2==0)
	    {
              ontimererror=1;			
			  timercomplete=0;
			  prev_group=groupinc;//move previous valve and increment
			  if(groupskip==0)
	       			groupinc++;               // move to next group for next call
			  groupincwrite=1;
			  if(groupinc>schedule_data[2])
			  {					
					groupinc = 1;
					cycle1=cycle1-1;	//this cycle byte is moved to another variable from mqqt getting 			
					cyclestore=1;					
			  }
				if(cycle1==0)
        {
        //motoron = 0; after acknowledgement received
        for(i=0; i<maxvalves; i++)
        {
            byte_index = i/8;
            bit_index  = i%8;
            valve_state[byte_index] &= ~(1 << bit_index);
        }
		  ///setpumpoff flag//
	         relayoff=1;
        }
				valvedelete();
   	 	}	
		}
////////////////////////////manual cyclic & manual RTC mode function end  here//////////////////////////////
///////////////////////////RTC mode start here/////////////////////////////////////////////////////////////
	if(mode4==1&&pumponflag==1)
	{
	//rtcschedulercheck();
		if(timercomplete==0)
		{
			slavecheck();
			valvetrack=0;
			for(i=0;i<maxvalves;i++)
  			{
			    byte_index=i/8;
          		bit_index=i%8;
				if(valve_group[i]==groupinc)
				{
					valvetrack=1;					
         			valve_number=i;  // Turn ON valve	
         			valve_state[byte_index]|=(1<<bit_index);	
				//////////////////////
				 	addvalveinslave();	
				//////////////////////
				}
			}
	/////////valve on////
    	motoron=0;
	    if(valvetrack==1)
		 valveon();
	    if(slavenoresponse==1)
	     slaveerror_check();
		if(slavenumber!=0&&turnonv==0&&slaveerror==0)
		{
				turnonv=1;
				writevaluestate=1;
		}
				//set pump on flag
		timercomplete=1;	
				//no-error
		if(slaveerror==0&&slavenumber!=0&&valvetrack==1)
				motoron=1;
			}
		}
		////////////////////////////// RTC mode function end here//////////////////////////////
	}
	///after pump off confirmation turn off the all valve here
	if(turnoffv==1)
	{
		valvetrack=0;
		reset1=0;
	 	ondelaycheck=0;
		ondelayok=0;
		valvedelete();
		if(startrly==1)
			startrly=0;
		startrlyoffcount=0;
		allvalveoff();
		if(slavenoresponse==1)
			slaveerror_check();
		turnoffv=0;
		modechanged=0;
		pumpoffclear=1;
	}
	if(pumpoffclear==1)
		pumpoffclearcount++;
	if(pumpoffclearcount>120)
	{
		pumpoffclear=0;
		pumpoffclearcount=0;
		modechanged=0;
	}
}

void readreference(void)
{
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
        	rampsreference=resultct1;
		if(ibm==1)
	        yampsreference=resultct1;
		if(ibm==2)
	        bampsreference=resultct1;
	}
	//EA=1;
}

void unbalcal(void)
{
	ppdiff=ppvolt-avgvolt;
	if(ppdiff>65000)
		ppdiff=65536-ppdiff;
	pppercent=(ppdiff*100)/avgvolt; 
}

void phrcheck(void)
{	
	///phase reversal and phase fail check loop
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
	
	
if(((rphvolt1<80)&&(dashboardsignal[0]==3)&&start2==0)||((rphvolt1<80)&&(dashboardsignal[0]==3)&&(start2==1)&&(phasereversed==1)))//phase fail condition for rphase
{
          rphfail=1;
          phasenumber=1;
	      phaseunbalance=0;		
}
if(((yphvolt1<80)&&(dashboardsignal[0]==3)&&start2==0)||((yphvolt1<80)&&(dashboardsignal[0]==3)&&(start2==1)&&(phasereversed==1)))//phase fail condition for yphase
{
	       
	yphasefailcount=0;
    yphfail=1;
    phasenumber=2;
	phaseunbalance=0;
				
}
if(((bphvolt1<80)&&(dashboardsignal[0]==3)&&(start2==0))||((bphvolt1<80)&&(dashboardsignal[0]==3)&&(start2==1)&&(phasereversed==1)))//phase fail condition for b phase
{
          bphfail=1;
          phasenumber=3;
	      phaseunbalance=0;
}
if((rphfail==1||yphfail==1||(bphfail==1))&&(dashboardsignal[0]==3))
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
if(dashboardsignal[0]==3&&rphfail==0&&yphfail==0&&bphfail==0&&voltlow==0&&volthigh==0&&dashboardsignal[1]==0&&phr==1)
{ 
	if((resultt>675)&&(resultt<1012))
	{
		phasereversed=0;
			phrtime=0;
	}
	if((resultt<675)||(resultt>=1012))
	{
		phrtime++;
		if(phrtime>6)
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
if(phr==0)
phasereversed=0;
}
void autosetclr(void)
{

if(autosetclrstart==1)
{
   testcount++;
	if(autoclr==1)
		{
			///////////////auto clear function start here////////////////////////////
			if(dashboardsignal[0]==1)//single phase 
			{
				seting_group[2]=300;
				seting_group[3]=10;
				seting_group[1]=150;
				seting_group[0]=320;
		    }
			if(dashboardsignal[0]==3)//3 phase
			{
				seting_group[6]=seting_group[2]=320;
				seting_group[7]=seting_group[3]=40;
				seting_group[4]=seting_group[0]=470;
				seting_group[5]=seting_group[1]=285;
			}
		if(slaveerror==0)
				restart=0;
		start2=0;
		volthigh=0;
		voltlow=0;
		ovld=0;
		if(dry==1)
		{
			dry=0;
			drywrite=1;
		}
	    unbalancecurrent=0;
      	ubc1=0;
      	msec=0;
		run7=0;
		if(dryrestart==1)
		{
		     dryrestart=0;
			 drt1=seting_group[8];
			 drtstore=1;
		}
		drterror=1;
		errordisplay=0;
		vfault=0;
		manualoff=0;
		nopump=0;
		if(errorclearflag==1&&slaveerror==0)
		    onlyonceFlag=1;
		 errorclearflag=0;
	}
		if(autoset==1)
		{
			///////////////auto set function
			mqautosetdone=1;
			if(testcount<=40)
			{
				if(dashboardsignal[0]==1)
				{
					setcurrent=rctval1+setcurrent;
					setvoltage=dispry1+setvoltage;
				}
				if(dashboardsignal[0]==3)
				{
					setvoltagex1=dispry1+setvoltagex1;
					setvoltagex2=dispyb1+setvoltagex2;
					setvoltagex3=dispbr1+setvoltagex3;
					setcurrent=rctval1+setcurrent;
				}
			}
			if(testcount==40)
			{
				if(dashboardsignal[0]==1)
				{
					setcurrent1=setcurrent/40;
					setvoltage1=setvoltage/40;
				}
				if(dashboardsignal[0]==3)
				{
					rphasesetvolt=setvoltagex1/40;
					yphasesetvolt=setvoltagex2/40;
					bphasesetvolt=setvoltagex3/40;
					setvoltage1=((rphasesetvolt+yphasesetvolt+bphasesetvolt)/3);
					setcurrent1=setcurrent/40;
				}
			
				setcurrent=0;
				setvoltage=0;
				setvoltagex1=0;
				setvoltagex2=0;
				setvoltagex3=0;
				rphasesetvolt=0;
				yphasesetvolt=0;
				bphasesetvolt=0;

				if(dashboardsignal[1]==0)
					seting_group[0]=(setvoltage1*13)/10;
				if(dashboardsignal[1]==1)
					seting_group[4]=(setvoltage1*13)/10;
				if(seting_group[0]>=275&&dashboardsignal[0]==1)
					seting_group[0]=275;
				if(seting_group[0]>=600&&dashboardsignal[0]==3&&dashboardsignal[1]==0)
					seting_group[0]=600;
				if(dashboardsignal[1]==1&&seting_group[4]>=600)
					seting_group[4]=600;	
				if(dashboardsignal[1]==0)
					seting_group[1]=(setvoltage1*7)/10;	
				if(dashboardsignal[1]==1)
					seting_group[5]=(setvoltage1*7)/10;	
				if(seting_group[1]<=150&&dashboardsignal[0]==1)
					seting_group[1]=150;
				if(seting_group[1]<=285&&dashboardsignal[0]==3&&dashboardsignal[1]==0)
					seting_group[1]=285;
				if(seting_group[5]<=285&&dashboardsignal[1]==1)
					seting_group[5]=285;
				if(dashboardsignal[1]==0)
					seting_group[2]=(setcurrent1*12)/10;
				if(dashboardsignal[1]==1)
					seting_group[6]=(setcurrent1*12)/10;
				if(seting_group[2]>=200&&dashboardsignal[0]==1)
					seting_group[2]=200;
				if(seting_group[2]>=500&&dashboardsignal[0]==3&&dashboardsignal[1]==0)
					seting_group[2]=500;
				if(seting_group[6]>=500&&dashboardsignal[1]==1)
					seting_group[6]=500;
				if(dashboardsignal[1]==0)
					seting_group[3]=(setcurrent1*8)/10;
				if(dashboardsignal[1]==1)
					seting_group[7]=(setcurrent1*8)/10;
					if(seting_group[3]<=10&&dashboardsignal[1]==0)
						seting_group[3]=10;
					
					if(seting_group[7]<=10&&dashboardsignal[1]==1)
					  	seting_group[7]=10;	
			}
			//auto set end here	
		}
	if(testcount>60)
	{
		testcount=0;
		memsave=1;
		autosetclrstart=0;
    autosetdone=1;
    refresh=1;
    displayx=2;
		sense=0;
		adctime=0;
		////autoset and clear end here 
	}
}		
}
void decide(void)
{
	///////////////////it performs all voltage  related error handling
	if(dashboardsignal[1]==0)
	{
		lowvolt=seting_group[1];
	    highvolt=seting_group[0];
	}
	if(dashboardsignal[1]==1)
	{
		lowvolt=seting_group[5];
	    highvolt=seting_group[4];
	}

if(phasefail==0&&phasereversed==0&&phaseunbalance==0  && rphfail==0 && yphfail==0 && bphfail==0)
{
chkonce++; 
if(chkonce>2)//1
{
chkonce=0;
	
if(((dispry<lowvolt||dispyb<lowvolt||dispbr<lowvolt)&&(dashboardsignal[0]==3))||(dispry<lowvolt&&dashboardsignal[0]==1))//295//low voltage condition
	{
voltlow=1;
	if(start2==0&&dashboardsignal[0]==1)
		{
    errorvalue=dispry;
		if(restart==1)
	 {
		 vfault=1;
	 }
		}
if((dashboardsignal[0]==3)&&(dispry<lowvolt)&&(start2==0))
{
errorvalue=dispry;
phasenumber=1;
	if(restart==1)
	 {
		 vfault=1;
	 }
}
if((dashboardsignal[0]==3)&&(dispyb<lowvolt)&&(start2==0))
{
errorvalue=dispyb;
phasenumber=2;
	if(restart==1)
	 {
		 vfault=1;
	 }
}
if((dashboardsignal[0]==3)&&(dispbr<lowvolt)&&(start2==0))
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

if(((((dispry>highvolt)||(dispyb>highvolt)||(dispbr>highvolt))&&dashboardsignal[0]==3))||(dispry>highvolt&&dashboardsignal[0]==1))//high voltage condition
{
	  volthigh=1;
		if(start2==0&&dashboardsignal[0]==1)
		{
    errorvalue=dispry;
		if(restart==1)
	  {
		 vfault=1;
	  }
		}
if((dashboardsignal[0]==3)&&(dispry>highvolt)&&(start2==0))
{
  errorvalue=dispry;
  phasenumber=1;
	if(restart==1)
	 {
		 vfault=1;
	 }
}
if((dashboardsignal[0]==3)&&(dispyb>highvolt)&&(start2==0))
{
errorvalue=dispyb;
phasenumber=2;
	if(restart==1)
	 {
		 vfault=1;
	 }
}
if((dashboardsignal[0]==3)&&(dispbr>highvolt)&&(start2==0))
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
if(voltlow==0&&volthigh==0&&phasefail==0&&phasereversed==0&&dashboardsignal[0]==3&&rphfail==0&&yphfail==0&&bphfail==0)
{
	checkvoltvalues(dispry,dispyb,dispbr);
	 if(phaseunbalance==1)
	 {
	 if(restart==1)
	 {
		 vfault=1;
	 }
 }
 
}                                                    //normal condition
if(mode2==1||mode3==1||mode4==1||mode1==1) 
{
//if((rypercent<4&&ybpercent<4&&brpercent<4)&&(dashboardsignal[0]==3))
//  phaseunbalance=0;
if(((checkvolt(dispry)&&checkvolt(dispyb)&&checkvolt(dispbr))&&(dashboardsignal[0] == 3))||(checkvolt(dispry)&&(dashboardsignal[0]==1)))
{ 
  volthigh=0;
  voltlow=0;
 }
 if((dashboardsignal[0]==3)&&(rphvolt1>80&&yphvolt1>80&&bphvolt1>80))
 {
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
 if((mode2==1||mode3==1||mode4==1||mode1==1)&&voltlow==0&&volthigh==0&&phasefail==0&&start2==1&&phaseunbalance==0&&phasereversed==0)//autonormal for voltage error
 {
if(start2==1)
{
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
	
	if(errorclearflag==1)
		  onlyonceFlag=1;
}
} 
}
void vcutoff(void)
{
	///////it performs if any volatge error occur, it will work as a pump trip function 
	hvmax=highvolt+40;
if(((volthigh==1)||(voltlow==1)||(phaseunbalance==1)||(phasefail==1)||(phasereversed==1))||(vfault==1&&start2==0))
{
	if((restart==0)&&(ovld==1||dry==1))
	{
		dry=0;
		ovld=0;
		restart=0;
	}
	if(((restart==0)&&start2==0)||(vfault==1&&start2==0))
	{
		msec++;
		if((((dispry>highvolt)||(dispry<lowvolt))&&(dashboardsignal[0]==1))||((dashboardsignal[0]==3)&&((dispry||dispyb||dispbr)>highvolt)&&(phaseunbalance==1)))	
		msec=msec;
	
		if(((dispry>hvmax)&&(dashboardsignal[0]==1))||(((dispry>hvmax)||(dispyb>hvmax)||(dispbr>hvmax))&&dashboardsignal[0]==3))
			msec=msec+5;
	}
if((msec>124&&vfault==0)||((phasereversed==1||phasefail==1)&&dashboardsignal[0]==3&&start2==0)||(vfault==1&&msec>100&&start2==0&&(dashboardsignal[0]==3||dashboardsignal[0]==1)))//For disp off and relay on//100
{
	dashboardcal();
	counter1=0;
	cnt=1;
	start2=1;
	 msec=0;
 start4=0;
		ondelaycheck=0;
if(rctval>=10||pon==1)
	relayoff=1;
onlyonceFlag=1;
	 //restart=0;
if(phasereversed==1)
		update_error();
	
}  
} 
}
void unblanceampscut(void)
{
	///unbalancecurrent pump trip  function 
	if(unbalancecurrent==1)
	{
		if(restart==0&&start2==0)
		{
			ubc1++;
		}
		if(ubc1>200)//230-30 sec
		{
	dashboardcal();
	counter1=0;
	cnt=1;
			ubc1=0;
			relayoff=1;
			restart=1;
			onlyonceFlag=1;
		}
	}
}

void drycuttoff(void)
{
	/// dry error pump trip function
if((dry==1))
{
	if((restart==0)&&(start2==0))
	{
		dtime++;
	}
    if((dtime>(seting_group[18]*8)))//For disp off and relay on//28173//40 sec///if(dtime>drycut)
    {
	    if(unbalancecurrent==1)
			unbalancecurrent=0;
	
	dashboardcal();
	counter1=0;
	cnt=1;
	dtime=0;
	relayoff=1;
	restart=1;
    onlyonceFlag=1;
	drywrite=1;
}
}	
}

void ovlcuttoff(void)
{
	//overload error pump trip error 
 if((ovld==1))
 {
    if((restart==0&&start2==0))
	{
	    ovtime++;
	    if(ovld==1)   
		{   
			//initialovl=((150-disp4));
			//initialovl=(32+(initialovl*2));//11264//8448
				initialovl=((seting_group[17]*8)-(((disp4-101)*((seting_group[17]*8)-4))/(200-101)));
		}
	} 
    if(ovtime>initialovl)//For disp off and relay on
    {	
		dashboardcal();
		counter1=0;
		cnt=1;
		if(unbalancecurrent==1)
				unbalancecurrent=0;
		relayoff=1;
		ovtime=0;
		pofrflag=1;
		restart=1;
		onlyonceFlag=1;
		initialovl=0;	
    }   
 }   
} 

void curdecide(void)
{
	// this function handles all current related error 
	if(dashboardsignal[1]==0)
	{
		overload=seting_group[2];
		dryx=seting_group[3];
	}	

	if(dashboardsignal[1]==1)
	{
		overload=seting_group[6];
		dryx=seting_group[7];
	}

    if((start2==0)&&(rctval>=10)&&(volthigh==0)&&(voltlow==0)&&(start1==1))
    {
		  disp2=overload;
		  //if(rctval>=500)
		  //rctval=500;
	      disp4=rctval*100;
	      disp4=disp4/disp2;
	      if(disp4>=150)
            disp4=150;				
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
				dry=1;	
unbalancecurrent=0;
ubc1=0;	
			}
			if(rctval>=overload)
			{
				ovld=1;		
unbalancecurrent=0;
ubc1=0;				
			}
	
  }	
} 
void pwmfunction(void)
{    //this function perform in pwm generation led related function
   if(switchon==0) 	        
	   pwmvalue1=0;

	if(switchon==0)
	{
	if(rctval1>=10)
		 pwmvalue1=200;
 
	
		
  if((volthigh==1||voltlow==1||phasereversed==1||phasefail==1||phaseunbalance==1))
		 pwmvalue1=400;
	
	if((dry==1||ovld==1||slaveerror==1||unbalancecurrent==1||(nopump==1&&start2==0))&&(vfault==0))
		    pwmvalue1=500;
}

	if(pwmvalue1!=pwmvalue1x)
	{
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
	AINDIDS0=0x02;//p11
	
	SFRS=2;
	AINDIDS1=0xC4;//0xc6//p25,p14,p22,P21
	SFRS=0;
	
  ADCCON1=0x31;	
	ADCCON2=0X0E;
}
void adcal(void)
{
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
  TR1=0;
  TMOD|=(1<<4);
  TH1=0xcb;
  TL1=0xea;
  IP&=~(1<<3);
  IPH|=(1<<3);
  IE|=(1<<3);
  IE|=(1<<7);
//	TR1=1;
}
void init(void)
{
 set_clock_source();
 portinit();
}

void lcdinit(void)	
{
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
chip=0;
enabler(0x80);
delay(2);
chip=1;
}
void sline(void)
{
chip=0;
enabler(0xc0);
delay(1);
chip=1;
}

void valuewrite2(void)
{
   chip=0;
   enabler(address);
   delay(3);
   chip=1;

     limit=4;
   if(((((displayx==1)&&amps==1&&autosetclrshow==0)||(autosetclrshow==0&&displayx==2&&timedisplay==1))&&(errorlog==0))||(errorlog==1&&twodigitdisp==0))
     limit=5;     
    if((errorlog==1&&twodigitdisp==1)||((asetkey==1||aclrkey==1)&&(autosetclrprocess==0)))
     limit=3;
    
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
  chip=0;
   enabler(address);
   delay(3);
    chip=1;
   if((errordisplay==1&&smode==0&&errorlog==0&&(volthigh==1||voltlow==1||phasefail==1))||((smode==1&&errorlog==1&&errorprint==1)&&(errordisplay1==1||errordisplay1==2||errordisplay1==6)))
       limit=4;
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
if(autosetclrshow==0&&autosetclrprocess==0&&modedisp==0&&autosetdone==0&&asetkey==0&&aclrkey==0)
  {
if(displayx==4)
{
buffer=dashboardsignal[3];
split2();
address=0x8a;
valuewrite2();
	
	/*buffer=errorpercent;
  split2();
  address=0xc7;
  valuewrite2();*/
}
if(displayx==1)
{
	fline();
  if(dashboardsignal[0]==3)
  {
/*	for(t3=0;t3<8;t3++)
		{
			bufferx=valve_group[t3];
			digit1[1]=(bufferx/10);
      dbuf[1]=digit1[1];
			enabler(dbuf[1]);
      bufferx=bufferx%10;
      digit1[2]=(bufferx);
      dbuf[2]=digit1[2];
			enabler(dbuf[2]);
			enabler(bufferx);
      delay(1);
		}*/         
  buffer=dispry;//dispry	
  split2();
  address=0x82;
  valuewrite2();
	
	buffer=dispyb;//dispyb
  split2();
  address=0x87;
  valuewrite2();

	
	buffer=dispbr;//dispbr
  split2();
  address=0x8c;
  valuewrite2();

//if((errordisplay==0)||(errordisplay==1&&dryrestart==1&&autosetclrprocess==0&&autosetclrshow==0&&mode==3))
if(errordisplay==0)
{
//sline();
/*for(t3=0;t3<8;t3++)
		{
			bufferx=seting_group[t3];
			digit1[1]=(bufferx/10);
      dbuf[1]=digit1[1]+0x30;
			enabler(dbuf[1]);
      bufferx=bufferx%10;
      digit1[2]=(bufferx);
      dbuf[2]=digit1[2]+0x30;
			enabler(dbuf[2]);
      delay(1);
		}*/
	//buffer=dryrun;
	
buffer=rctval;
amps=1;
split2();
address=0xc2;
valuewrite2();
amps=0;

	//buffer=h1on;
buffer=yctval;
amps=1;
split2();
address=0xc7;
valuewrite2();
amps=0;

	//buffer=m1on;
buffer=bctval;
amps=1;
split2();
address=0xcc;
valuewrite2();
amps=0;
}
}
if(dashboardsignal[0]==1)
{ 
	buffer=dispry;//dispry
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
amps=0;
}
}
}
if((displayx==2)&&(dashboardsignal[0]==3))
{
	  /*fline();
		for(t3=0;t3<10;t3++)
		{
			temp=NM3[t3];
			enabler(temp);
      delay(1);
		}
		sline();
		for(t3=0;t3<10;t3++)
		{
			temp=NM4[t3];
			enabler(temp);
      delay(1);
		}*/
  timedisplay=1;
  buffer=runningtimex;
  split2();
  address=0x80;
  valuewrite2();
  timedisplay=0;
  //if((mode==2&&rctval1>7)||(mode==2&&rctval1<7)||(mode==3&&dryrestart==1&&dry==1&&restart==1))
		//if(((mode==2)&&oftimestart==0)||(mode==2&&oftimestart==1)|(dryrestart==1&&dry==1&&restart==1)| (mode==5 && ontoff==0 && dryrestart==0)) 
//if((mode==2&&oftimestart==0)||(mode==2&&oftimestart==1)||(dryrestart==1&&dry==1&&restart==1))
if(dryrestart==1&&dry==1&&restart==1)
  {
 /*if((oftimestart==0&&dryrestart==0)||((mode==5 && ontoff==0 && dryrestart==0)))
 {
 buffer=run2;
 if(manualoff==0)
{
  split2();
  address=0xcc;//cd
  valuewrite2();
	}
 }*/
//if(((rctval1<7&&oftshow==1)&&(mode==2)&&(oftimestart==1)&&(manualoff==0))||(dryrestart==1&&dry==1&&restart==1&&mode==3))
//if(((mode==2)&&(oftimestart==1)&&(manualoff==0)&&(dryrestart==0))||(dryrestart==1&&dry==1&&restart==1))
if(dryrestart==1&&dry==1&&restart==1)
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
if(displayx==3&&dashboardsignal[0]==3)
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
if((displayx==2)&&(dashboardsignal[0]==1))
{
  timedisplay=1;
  buffer=runningtime;
  split2();
  address=0x80;
  valuewrite2();
  timedisplay=0;
}
if(((((displayx==1)||displayx==2)&&(dashboardsignal[0]==1))&&(errordisplay==0))||(((displayx==1&&dryrestart==0)||(displayx==2))&&(dashboardsignal[0]==1)&&(errordisplay==1&&dryrestart==1&&autosetclrprocess==0&&autosetclrshow==0)))
{
  //if(((mode==2&&rctval1>7)||(mode==2&&rctval1<7))||(dryrestart==1&&dry==1&&restart==1&&mode==3))
	if(((mode==2&&oftimestart==0)||(mode==2&&oftimestart==1))||(dryrestart==1&&dry==1&&restart==1)||(mode==5 && ontoff==0 && dryrestart==0))
  {
 if((oftimestart==0)||((mode==5 && ontoff==0 && dryrestart==0)))
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
  valuewrite2();}}
}
}
	}
	if((aclrkey==1||asetkey==1)&&(autosetclrprocess==0&&switchtime!=0))
	{
	buffer=switchtime;
  split2();
  address=0xcb;
  valuewrite2();
	}
}
void splitl(void)
{
    bufferx=buffer;
	
    if(errorlog==0&&errorprint==0)
    {
			if(((asetkey==1||aclrkey==1)&&(autosetclrprocess==0)))
			{
digit1[1]=(bufferx/10);
dbuf[1]=digit1[1]+0x30;
bufferx=bufferx%10;
digit1[2]=(bufferx);
dbuf[2]=digit1[2]+0x30;
			}

if((asetkey==0&&aclrkey==0&&autosetclrprocess==0&&autosetclrshow==0&&smode==0&&displayx==3&&amps!=1&&dashboardsignal[0]==3)||(asetkey==0&&aclrkey==0&&autosetclrprocess==0&&autosetclrshow==0&&smode==0&&displayx==1&&amps!=1)||(asetkey==0&&aclrkey==0&&autosetclrprocess==0&&autosetclrshow==0&&smode==0&&displayx==2&&timedisplay!=1)||(displayx==4))   
		{
		threedigit();
    }

if((smode==0&&timedisplay==1))   
		{
		fourdigit();
    }
    if(amps==1&&autosetclrshow==0&&autosetclrprocess==0)
    {
 threedigita();
    }}
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
      }
      if((((ovld==1||dry==1)&&(vfault==0))&&(smode==0))||(errorlog==1&&(errordisplay1==4||errordisplay1==3)))
      {
				threedigita();
      
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

digit1[3]=dbuf[3]-0x30;

if(((timedisplay==1))&&(autosetclrshow==0))
	digit1[4]=dbuf[4]-0x30; 
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
	
for(x3=0;x3<100;x3++);
x3=t4;
dataout();
enab=0;
for(x3=0;x3<100;x3++);
enab=1;

}

void dataout(void)
{
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
		 result1=0;
		 resultx=0;
   //ENABLE1_ADC_CH15;
  // channel=0x0f;
	YPHASEMUXDIS;
	for(j=0;j<10;j++);
	ADCCON0=0x0A;
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
 if(vref2>995)
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
 

  if((asetkey==1||aclrkey==1)&&(autosetclrprocess==0&&autosetclrstart==0&&autosetdone==0&&autosetclrshow==0))
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

  sidekey=0;
  inckey=0;
  asetkey=0;
	aclrkey=0;
	menupress++;
	sidepress=0;
	incpress1=0;
	setpress=0;
	asetpress=0;
  aclrpress=0;
	//
 if((smode==0)&&(menupress>1)&&(switchset1==1)&&(switchon==0)&&resetx==0)
  {
    //menupress=0;
    //switchset1=0;
    //menukey=1;
		if(modedisp==0&&modechangedisp==0)
		{
		switchon=1;
		}
  }
	
}
if(vref2>370&&vref2<410)//inckey
{
  sidekey=0;
  menukey=0;
  asetkey=0;
	aclrkey=0;
	setkey=0;

  incpress1++;
	if(((start2==1||restart==1)&&(phasereversed==0))&&(incpress1>20&&switchset2==1&&asetkey==0&&aclrkey==0))
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

if(vref2>930&&vref2<990)//run/setmode
{
	inckey=0;
  menukey=0;
  asetkey=0;
	aclrkey=0;
	sidekey=0;

	setpress++;
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
	//aclrkey=0;
	sidekey=0;

	aclrpress=0;
	if(autoclr==0)
	asetpress++;
	if(smode==0&&rctval>=10&&asetpress>5&&switchset5==1&&resetx==0&&autosetclrprocess==0&&autosetdone==0&&autosetclrshow==0&&modedisp==0&&modechangedisp==0)
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
if(vref2>250&&vref2<350)//autoclr
{
	inckey=0;
  menukey=0;
	//asetkey=0;
	//sidekey=0;

if(autoset==0)
	aclrpress++;

     if((switchset6==1)&&(smenu>=1&&smenu<=12)&&(smode==1)&&(aclrpress>5))
     {
      sidekey=1;
      switchset6=0;
			 aclrpress=0;
     }
	if(smode==0&&rctval<10&&aclrpress>5&&switchset6==1&&autosetclrprocess==0&&autosetdone==0&&autosetclrshow==0&&modedisp==0&&modechangedisp==0)
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
lcdinit();
fline();
	if(resetx==0&&modechangedisp==0)
	{
for(t3=0;t3<16;t3++)
{
if(autosetclrprocess==0&&autosetclrshow==0&&modedisp==0&&autosetdone==0&&asetkey==0&&aclrkey==0)
 {
if((displayx==1||displayx==3)&&(dashboardsignal[0]==3))
temp=vin[t3];

if(((displayx==2)&&(dashboardsignal[0]==3))||(displayx==2&&dashboardsignal[0]==1))
temp=pt[t3];

if((displayx==1)&&(dashboardsignal[0]==1))
  temp=va[t3];

if(displayx==3&&dashboardsignal[0]==1)
temp=pumpt[t3];
if(displayx==4)
	temp=rssival[t3];
}
if((autosetclrprocess==1&&autosetclrshow==0&&modedisp==0)||((asetkey==1||aclrkey==1)&&(autosetclrprocess==0)))
{
  if((autoset==1)||(asetkey==1&&autosetclrprocess==0))
    temp=autos[t3];
  if((autoclr==1)||(aclrkey==1&&autosetclrprocess==0))
    temp=autoc[t3];
}
/*if(autosetclrshow==1&&modedisp==0)
    temp=acshowr1[t3];
  
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
} */
enabler(temp);
delay(1);
}

chip=0;
enabler(0x8c);
chip=1;

for(t3=0;t3<3;t3++)
{
if((((displayx==2)&&(dashboardsignal[0]==3))||((displayx==1||displayx==2)&&(dashboardsignal[0]==1)))&&(asetkey==0&&aclrkey==0&&autosetdone==0&&autosetclrprocess==0&&autosetclrshow==0&&modedisp==0))
 {
  if(rctval1>7)
  temp=ron[t3];
  if(rctval1<7)
  temp=rof[t3];
  enabler(temp);
  delay(1);
 }
}

sline();
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
  if(ovld==1&&vfault==0)
  temp=errorovl[t3];
  if(dry==1&&vfault==0)
 temp=errordry[t3];
	if(slaveerror==1&&vfault==0)
		temp=slavetime[t3];
	if(vfault==0&&unbalancecurrent==1)
			temp=unbc[t3];
  if(phaseunbalance==1&&dashboardsignal[0]==3)
  temp=errorph[t3];
	if(phasefail==1&&dashboardsignal[0]==3)
	temp=errorpf[t3];
  if(phasereversed==1&&dashboardsignal[0]==3)
	temp=errorpr[t3];
	if(nopump==1&& start2==0)
		temp=no_pump[t3];
}
  if(modedisp==1)
  temp=select[t3];
    
  enabler(temp);
  delay(1);
}
	
	if(dashboardsignal[0]==1)
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
if(dashboardsignal[0]==3)
{
//if(phasenumber==1&&dashboardsignal[0]==3)
if(phasenumber==1)
{
enabler(cr);
enabler(cy);
delay(1);
}
//if(phasenumber==2&&dashboardsignal[0]==3)
if(phasenumber==2)
{
enabler(cy);
enabler(cb);
delay(1);
}
//if(phasenumber==3&&dashboardsignal[0]==3)
if(phasenumber==3)
{
enabler(cb);
enabler(cr);
delay(1);
}
}
  }
if(phasefail==1&&dashboardsignal[0]==3)
{
//if(phasenumber==1&&phasefail==1&&dashboardsignal[0]==3)
	if(phasenumber==1)
enabler(cr);

//if(phasenumber==2&&phasefail==1&&dashboardsignal[0]==3)
	if(phasenumber==2)
enabler(cy);

//if(phasenumber==3&&phasefail==1&&dashboardsignal[0]==3)
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
  if((ovld==1||dry==1)&&(vfault==0))	
  {  
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
    enabler(temp);
    delay(1);
    }
}
if((errordisplay==0&&autosetclrprocess==0&&autosetdone==0&&autosetclrshow==0&&modedisp==0&&asetkey==0&&aclrkey==0)||(asetkey==0&&aclrkey==0&&errordisplay==1&&dry==1&&dryrestart==1&&autosetclrprocess==0&&autosetdone==0&&autosetclrshow==0&&modedisp==0&&displayx!=1))
{
for(t3=0;t3<16;t3++)
{
  if((displayx==1)&&(dashboardsignal[0]==3))
         temp=ai[t3];
	
	if((displayx==3)&&(dashboardsignal[0]==3)){
			 if(netconnected==1&&netopenFlag==1){  //////phase flag & net flag
				temp=pn[t3]; 
			 }
			  if(netconnected==1&&netopenFlag==0){  //////phase flag & net flag
				temp=servfail[t3]; 
			 }
			  if(netconnected==0){  //////phase flag & net flag
				temp=netfail[t3]; 
			 }
   
	}
  if(displayx==1&&dashboardsignal[0]==1)
  temp=am[t3];

  if(((displayx==2)&&(dashboardsignal[0]==3))||((displayx==2&&dashboardsignal[0]==1)))
  {
   if(mode1==1)
   temp=mod[t3];
   if(mode2==1)
   temp=mod1[t3];
   if(mode3==1)
   temp=mod2[t3];
   if(mode4==1)
   temp=mod3[t3];
	 //if(mode==5)
   //temp=mod4[t3];
   }
   if(displayx==3&&dashboardsignal[0]==1)//////phase flag & net flag
   {
		if(netconnected ==1 && netopenFlag==1){
    if(pumptype==1)
    temp=pump1[t3];

    if(pumptype==2)
    temp=pump2[t3];
	}
			  if(netconnected ==1 &&  netopenFlag==0){  //////phase flag & net flag
				temp=servfail[t3]; 
			 }
			  if(netconnected ==0 ){  //////phase flag & net flag
				temp=netfail[t3]; 
			 }		 
   }
if(displayx==4)
{
	if(network<=0)
	  temp=' ';
	if(network==1)
		temp=airtel[t3];
	if(network==2)
		temp=jio[t3];
	if(network==3)
		temp=vodafone[t3];
	if(network==4)
		temp=bsnl[t3];
	
}
	//temp=errsc[t3];
enabler(temp);
delay(1);
}
if(dashboardsignal[0]==3&&displayx==3  && netconnected ==1 && netopenFlag==1 )//////phase flag & net flag
{
	chip=0;
	enabler(0xcc);
	chip=1;
	if(dashboardsignal[1]==1)
		temp='2';
	if(dashboardsignal[1]==0)
		temp='3';
	
enabler(temp);
delay(1);	
}
chip=0;
if((mode4!=1)||(mode4==1&&dryrestart==1))
enabler(0xc7);
if(mode4==1&&dryrestart==0)
enabler(0xc8);
chip=1;
if(((displayx==2)&&(dashboardsignal[0]==3))||(((displayx==1||displayx==2)&&(dashboardsignal[0]==1))))
//if((displayx==2)&&(dashboardsignal[0]==3))
{
//	if(mode4==4&&dryrestart==0)
	if(mode4==1&&dryrestart==0)
        limitx=4;
	
  //if((manualoff==0&&(mode==2))||(mode==3)||(dryrestart==1&&mode==4)||(mode==5&&ontoff==0&&dryrestart==0))
	if((mode2==1||mode3==1||mode4==1)&&(dryrestart==1))
                  limitx=5;
	

  
	
for(t3=0;t3<limitx;t3++)
{
    temp=' ';
if((mode==2)&&(oftimestart==0)&&(dryrestart==0)|| (mode==5 && ontoff==0 && dryrestart==0))
{
    temp=ond[t3];
	
}

if((mode==2)&&(manualoff==0)&&(oftimestart==1)&&(dryrestart==0))
{
      temp=ofd[t3];
}

if(restart==1&&dry==1&&dryrestart==1)
      temp=drystart[t3];

/*if((mode==4)&&(timeron==1||timeron==3||timeron==5||timeron==7)&&(dryrestart==0))
 temp=sc1[t3];
 if((mode==4)&&(timeron==2||timeron==4||timeron==6||timeron==8)&&(dryrestart==0))
 temp=sc2[t3];*/
 
 
enabler(temp);
delay(1);
}
}
if(((displayx==2)&&(dashboardsignal[0]==3))||(((displayx==1||displayx==2)&&(dashboardsignal[0]==1))))
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
}
	if(resetx==1&&modechangedisp==0 && contoserflag==0)
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

chip=0;
enabler(0x0c);
delay(0x0a);
}
void datawrite(void)
{
 		 Write_DATAFLASH_BYTE1(61048,10);//for eepromtest
	   addr=61500;
	   Write_DATAFLASH_BYTE1(addr++, maxvalves  & 0xFF);       // low byte
     Write_DATAFLASH_BYTE1(addr++, (maxvalves  >> 8) & 0xFF); // high byte
	
	  for (i = 0; i < 19; i++) 
		{
    Write_DATAFLASH_BYTE1(addr++, seting_group[i] & 0xFF);       // low byte
    Write_DATAFLASH_BYTE1(addr++, (seting_group[i] >> 8) & 0xFF); // high byte
    }///start 61001 to 61026
		
		 
    for(i=0;i<64;i++)
		{
        //group_ontime[v] //group ontimer
			Write_DATAFLASH_BYTE1(addr++,group_ontime[i] & 0xFF);
		}
		
		for(i=0;i<29;i++)
		{
         //schedule_data[v] //
			Write_DATAFLASH_BYTE1(addr++,schedule_data[i] & 0xFF);
		}		
		 for(i=0;i<3;i++)
		 {
			 Write_DATAFLASH_BYTE1(addr++,max_liters[i] & 0xFF);//order changed in app 123456 like this....
		 }
    for(i=0;i<150;i++)
		{
        //valve_group[v] //valves group
			Write_DATAFLASH_BYTE1(addr++,valve_group[i] & 0xFF);
		}   
   
		for (i = 0; i < 14; i++) 
	  {
		Write_DATAFLASH_BYTE1(addr++, user_smsdis[i+1] & 0xFF);
    }///start 61027 to 61039
	 
    Write_DATAFLASH_BYTE1(addr++, STAT1);
		Write_DATAFLASH_BYTE1(addr++, STAT2);
		Write_DATAFLASH_BYTE1(addr++, STAT3);
		Write_DATAFLASH_BYTE1(addr++, STAT4);
		
		for (i = 0; i < 20; i++) 
	  {
		Write_DATAFLASH_BYTE1(addr++, valve_state[i]);
    }
     	
	 
		/*if(mqpofr>1)
	  {
		mqpofr=0;
		}
		Write_DATAFLASH_BYTE1(61044,mqpofr);
	
		if(mqphrcheck>1)
		{
		mqphrcheck=0;
		}
		Write_DATAFLASH_BYTE1(61043,mqphrcheck);*/
		
	
      /*if(mode>5)
		  {
		  // mode=5;
	    }
      Write_DATAFLASH_BYTE1(61001,mode);*/
      
		
      
			
        /*ontimeh=ontime/256;
	      ontimel=ontime%256;
	      Write_DATAFLASH_BYTE1(61006,ontimeh);
				Write_DATAFLASH_BYTE1(61007,ontimel);

        
        run2h=run2/256;
	      run2l=run2%256;
	      Write_DATAFLASH_BYTE1(61008,run2h);
	      Write_DATAFLASH_BYTE1(61009,run2l);



        run3h=run3/256;
	      run3l=run3%256;
	      Write_DATAFLASH_BYTE1(61014,run3h);
	      Write_DATAFLASH_BYTE1(61015,run3l);
        
				 Write_DATAFLASH_BYTE1(61046,twophase);*/

   //combine();
}
void readdata(void)
{
	
	 Read_APROM_BYTE(61016);
	 timeron=rdata;
	  if(timeron>10)
		  timeron=0;
	
	     if(rdata==10)
			 eepromtest=0;
	     eepromtest=1;
	     Read_APROM_BYTE(61048);
	     if(rdata==10)
			 eepromtest=0;
	      
	
	      Read_APROM_BYTE(61049);
        dashboardsignal[0]=rdata;
        if(dashboardsignal[0]>5)
	      dashboardsignal[0]=3;
				
		
			 
			 Read_APROM_BYTE(61050);
			 if(rdata>1)
			groupskip=rdata=0;
			 groupskip=rdata;
			 
			Read_APROM_BYTE(61056);
			 dry=rdata;
			 if(dry!=1)
				 dry=0;
		Read_APROM_BYTE(61059);
	  dryrestart=rdata;
			 if(dryrestart>1)
				 dryrestart=0;
				
	      addr = 61500;//start address in flash
			 
			   Read_APROM_BYTE(addr++);
         low = rdata;
         Read_APROM_BYTE(addr++);
         high = rdata;
         maxvalves = ((high << 8) | low);
				 if(maxvalves>250)
         maxvalves=150;
				 
				 
        for (i = 0; i < 19; i++) 
        {
		    Read_APROM_BYTE(addr++);
        low = rdata;
        Read_APROM_BYTE(addr++);
        high = rdata;
        seting_group[i] = ((high << 8) | low);
        }	
		    for(i=0;i<64;i++)
		    {
			  Read_APROM_BYTE(addr++);
        low = rdata;
			  if(low>=255)
				low=0;
			  group_ontime[i]=low;
			  }
		   for(i=0;i<29;i++)
		   {
			 Read_APROM_BYTE(addr++);
        low = rdata;
			 if(low>=255)
				low=0;
			 schedule_data[i]=low;
		   }
		for(i=0;i<3;i++)
		 {
			 Read_APROM_BYTE(addr++);
        low = rdata;
			 if(low>=255)
				low=0;
			 max_liters[i]=low;
		 }
		   for(i=0;i<150;i++)
		    {
				Read_APROM_BYTE(addr++);
        low = rdata;
			  if(low>=255)
				low=0;
			  valve_group[i]=low;
	      }
				
		    for(i = 0; i < 14; i++) 
        {
		    Read_APROM_BYTE(addr++);
        user_smsdis[i+1] = rdata;
				if(user_smsdis[i]>2)
					 user_smsdis[i]=0;
        }
				
		    Read_APROM_BYTE(addr++);
        STAT1 = rdata;
				
				Read_APROM_BYTE(addr++);
        STAT2 = rdata;
				
				Read_APROM_BYTE(addr++);
        STAT3 = rdata;
				
				Read_APROM_BYTE(addr++);
        STAT4 = rdata;
				if(eepromtest==1)
				{
					STAT1=0;
					STAT2=0;
					STAT3=0;
					STAT4=0;
				}
		    for(i=0;i<20;i++)
		    {
				Read_APROM_BYTE(addr++);
        low = rdata;
			  if(eepromtest==1)
				low=0;
			  valve_state[i]=low;
	      }
		
if(dashboardsignal[0]==1)
{
if(seting_group[0]>320)
	 seting_group[0]=260;
if(seting_group[1]>230)
	 seting_group[1]=180;
if(seting_group[2]>500)
	 seting_group[2]=80;
if(seting_group[3]>((seting_group[2]*90)/10))
   seting_group[3]=10;
if(seting_group[4]>650)
	 seting_group[4]=650;//460
if(seting_group[4]<400)
	 seting_group[4]=400;				  
if(seting_group[5]>370)
	 seting_group[5]=370;//280
if(seting_group[5]<280)
	 seting_group[5]=280;//280			 
if(seting_group[6]>500)
	 seting_group[6]=80;
if(seting_group[7]>500)
   seting_group[7]=70;
if(seting_group[7]<10)
   seting_group[7]=((seting_group[6]*90)/10);	
}	
if(dashboardsignal[0]==3)
{
if(seting_group[0]>1000)
	 seting_group[0]=450;//460
if(seting_group[1]>1000)
	 seting_group[1]=300;//280 
if(seting_group[2]>500)
	 seting_group[2]=120;
if(seting_group[3]>500)
   seting_group[3]=70;
if(seting_group[3]<10)
   seting_group[3]=((seting_group[2]*90)/10);
if(seting_group[4]>650)
	 seting_group[4]=650;//460
if(seting_group[4]<400)
	 seting_group[4]=400;				  
if(seting_group[5]>370)
	 seting_group[5]=370;//280
if(seting_group[5]<280)
	 seting_group[5]=280;//280			 
if(seting_group[6]>500)
	 seting_group[6]=80;
if(seting_group[7]>500)
   seting_group[7]=70;
if(seting_group[7]<10)
   seting_group[7]=((seting_group[6]*90)/10);					
}
if((seting_group[8]>999)||(seting_group[8]<=0))
	seting_group[8]=10;

if(seting_group[9]>10)
	seting_group[9]=0;

if(seting_group[10]>10)
	seting_group[10]=0;


if((seting_group[11]>99)||(seting_group[11]<=0))
	seting_group[11]=5;

if((seting_group[12]>99)||(seting_group[12]<=0))
	seting_group[12]=5;

if((seting_group[13]>99)||(seting_group[13]<=0))
	     seting_group[13]=5;

if(seting_group[14]>64)
	seting_group[14]=0;
swapgroupvalue=seting_group[14];
				
	if(seting_group[15]>100)
	seting_group[15]=50;	

if(seting_group[16]>10)
	seting_group[16]=10;		
	
if((seting_group[17]>999)||(seting_group[17]<=0))
	seting_group[17]=30;

if((seting_group[18]>999)||(seting_group[18]<=0))
	seting_group[18]=20;
	
for ( q = 0; q < 7; q++) 
{
     adrs = baseAddresses[q];
    for ( n = 0; n < 10; n++)
		{
        Read_APROM_BYTE(adrs);  // sets rdata
        numbers[q][n] = rdata;

        // Validate digit
        if (numbers[q][n] > '9' || numbers[q][n] < '0')
				{
            numbers[q][n] = '0';
        }

        adrs++;
    }
    numbers[q][10] = '\0'; // Null-terminate the number string
}
					
Read_APROM_BYTE(61077);	
maxuser=rdata;
if(maxuser>7){
	maxuser=0;
}

Read_APROM_BYTE(61039);	
ontoff=rdata;
if(ontoff>2){
	ontoff=1;
}

Read_APROM_BYTE(61038);
 cycle1=rdata;
if(cycle1>=255)
	cycle1=cycle;

Read_APROM_BYTE(61043);	
cycle=rdata;
if(cycle>=250){
	cycle=0;
}


Read_APROM_BYTE(61046);
   groupinc=rdata;
if(groupinc>=255)
	groupinc=1;

Read_APROM_BYTE(61047);
   turnonv=rdata;
if(turnonv>1)
	turnonv=0;

if((groupinc==seting_group[14])&&(groupskip==1))
{
	  groupskipcheck=0;
	
}



  Read_APROM_BYTE(61064);           
  run2h=rdata;
	Read_APROM_BYTE(61065);
	run2l=rdata;
	run2=((run2h<<8)|run2l);
if(run2>999)
	run2=0;

progmstore1=progmstore=schedule_data[3];

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
int combine2digits(char d1,char d2)
{
return(d1-'0')*10+(d2-'0');
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
/*variablecombine(schedule_data[7],schedule_data[9]);
ontime1set=combinevalue;
variablecombine(schedule_data[8],schedule_data[10]);	
offtime1set=combinevalue;
	
variablecombine(schedule_data[11],schedule_data[13]);
ontime2set=combinevalue;
variablecombine(schedule_data[12],schedule_data[14]);
offtime2set=combinevalue;
	
variablecombine(schedule_data[15],schedule_data[17]);
ontime3set=combinevalue;
variablecombine(schedule_data[16],schedule_data[16]);
offtime3set=combinevalue;
	
variablecombine(schedule_data[19],schedule_data[21]);
ontime4set=combinevalue;
variablecombine(schedule_data[20],schedule_data[22]);
offtime4set=combinevalue;

variablecombine(schedule_data[23],schedule_data[25]);
ontime5set=combinevalue;
variablecombine(schedule_data[24],schedule_data[26]);
offtime5set=combinevalue;

variablecombine(schedule_data[27],schedule_data[29]);
ontime6set=combinevalue;
variablecombine(schedule_data[28],schedule_data[30]);
offtime6set=combinevalue;*/
	

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
	runningtimex=runningtime;
rtctime=runningtime;

  rread1(4);
	curdate=datai;
  bcdtodecimal(curdate);
	runningdate=buf;
	
	rread1(5);
	curmonth=datai;
  bcdtodecimal(curmonth);
	runningmonth=buf;
	
	variablecombine(runningdate,runningmonth);
	runningdaymonth=combinevalue;
	rread1(6);
	curyear=datai;
  bcdtodecimal(curyear);
	runningyear=buf;

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

void pwm_init(void)
{
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
    return (v > (lowvolt+5)) && (v < (highvolt-5));
}


void gsm_uartinit(void)
{
//SFRS=0;
SFRS=2;
AUXR2 |=0X0F;
SFRS=0;
P00_INPUT_MODE;
P10_PUSHPULL_MODE;
SCON_1 = 0x52; //UART1 Mode1,REN_1=1,TI_1=1
T3CON = 0xF8; //T3PS2=0,T3PS1=0,T3PS0=0(Prescale=1),
RH3 = 0XFF;
RL3 = 0X98;           //Trigger Timer3 
T3CON|= 0x08;
//P0|=0X01;
EIE1 |= (1<<0);
IE |=(1<<7);
TR1=1;
}

void gsm_reconnect(void)
	{	
		///////this loop perform like imei number read , sim detected or not like that 	
display_contointernet();		
UART_Send_String("ATS0=006\r");
checkok_error();
UART_Send_String("AT+SIMEI?\r");
checkok_error();
if(okFlag==0){
imeiError=1;
	}
UART_Send_String("AT+CSPN?\r");
checkok_error();

UART_Send_String("AT+CGATT?\r");
checkok_error();
	
	UART_Send_String("AT+CGACT?\r");
checkok_error();
	}
		
void processData(void)
	{
		////////this function performs all UART received data from the GSM 
		if( receivedData[itr-3] == 'S' && receivedData[itr-2] == 'A' && receivedData[itr-1] == '+')
    {   //NB:ReceiveData buffer have 10-digits of number 2 in i=0 to i=9 & i=10,11,12,13 have "_2MN" format.
         x=13;
		     for(z=0;z<10;z++)
			   {
             num[z]=receivedData[itr-x];
             x--;
         }
				  //maxuser=1;
				 *flags[0] = 1;
    }
		
    if(receivedData[itr-18] == 'P' && receivedData[itr-19] == 'I'  && receivedData[itr-20] == 'L' && receivedData[itr-21] == 'C')
    {   
          x=11;
      for(z=0;z<10;z++)
			{
      recnumberon_incoming[z]=receivedData[itr-x];
      x--;
      }
	    messageFlag=1;
			calok=1;
    }
		
		if(receivedData[itr-21] == '+' && receivedData[itr-20] == 'C' && receivedData[itr-19] == 'M' && receivedData[itr-18] == 'T' && receivedData[itr-17] == ':')
    {
			x=11;
			n=0;
      for( n=0;n<10;n++)
      {
       recnumberon_incoming[n]=receivedData[itr-x];
       x--;
      }
      messageFlag=1;
		}
		
/*if((receivedData[itr - 10] == ' ') &&(receivedData[itr - 12] == 'M') &&(receivedData[itr - 13] == 'N')) 
{
          userIndex = receivedData[itr - 11] - '0';  // Convert '1'..'6' to 1..6
	     if(userIndex>1&&userIndex<5)
			 {
				 useradd=1;
			 }
       
       *flags[userIndex]=1;
       x = 9;
       for(z = 0; z < 10; z++) 
			 {
             num[z] = receivedData[itr - x];
             x--;
       }                
}*/	

if(receivedData[itr - 1] == 'N' &&receivedData[itr - 2] == 'O' &&receivedData[itr - 3] == 'D' &&receivedData[itr - 5] == 'B' &&receivedData[itr - 6] == 'P') 
		{
        moduleReadyFlag = 1;
    }
if(receivedData[itr - 1] == 'D' &&receivedData[itr - 2] == 'A' &&receivedData[itr - 3] == 'E' &&receivedData[itr - 7] == 'N' &&receivedData[itr - 8] == 'I' &&receivedData[itr - 9] == 'P') 
		{
        simokFlag = 1;
    }
if(receivedData[itr - 1] == 'M' &&receivedData[itr - 2] == 'E' &&receivedData[itr - 3] == 'R' &&receivedData[itr - 5] == 'M' &&receivedData[itr - 6] == 'I' &&receivedData[itr - 7] == 'S') 
		{
        simnotokFlag = 1;
    }	
		
		///
		if(receivedData[itr - 1] == 'l'&&receivedData[itr - 2] == 'e'&&receivedData[itr - 3] == 't'&&receivedData[itr - 4] == 'r'&&receivedData[itr - 5] == 'i'&&receivedData[itr - 6] == 'a')
		{
			network=1;
		}
		if(receivedData[itr - 1] == 'o'&&receivedData[itr - 2] == 'i'&&receivedData[itr - 3] == 'J')
		{
			network=2;
		}
		if(receivedData[itr - 1] == 'a'&&receivedData[itr - 2] == 'i'&&receivedData[itr - 3] == 'd'&&receivedData[itr - 4] == 'n'&&receivedData[itr - 5] == 'I'&&receivedData[itr - 7] == 'i'&&receivedData[itr - 8] == 'V')
		{
			network=3;
		}
		if(receivedData[itr - 1] == 'L'&&receivedData[itr - 2] == 'N'&&receivedData[itr - 3] == 'S'&&receivedData[itr - 4] == 'B')
		{
			network=4;
		}
		if(receivedData[itr - 1] == '0'&&receivedData[itr - 4] == 'T'&&receivedData[itr - 5] == 'T'&&receivedData[itr - 6] == 'A'&&receivedData[itr - 7] == 'G'&&receivedData[itr - 8] == 'C')
		{
			apnnotdetect=1;
		}
		if(receivedData[itr - 1] == '1'&&receivedData[itr - 4] == 'T'&&receivedData[itr - 5] == 'T'&&receivedData[itr - 6] == 'A'&&receivedData[itr - 7] == 'G'&&receivedData[itr - 8] == 'C')
		{
			apnnotdetect=0;
		}
		if(receivedData[itr - 1] == '1'&&receivedData[itr - 3] == '1'&&receivedData[itr - 6] == 'T'&&receivedData[itr - 7] == 'C'&&receivedData[itr - 8] == 'A'&&receivedData[itr - 9] == 'G'&&receivedData[itr - 10] == 'C')
		{
			pdpnotdetect=0;
		}
		if(receivedData[itr - 1] == '0'&&receivedData[itr - 3] == '1'&&receivedData[itr - 6] == 'T'&&receivedData[itr - 7] == 'C'&&receivedData[itr - 8] == 'A'&&receivedData[itr - 9] == 'G'&&receivedData[itr - 10] == 'C')
		{
			pdpnotdetect=1;
		}
		///
if(receivedData[itr-1] == 'K' && receivedData[itr-2] == 'O')
    {
       okFlag=1;
    }
if(receivedData[itr-1] == 'G'  && receivedData[itr-2] == 'E' && receivedData[itr-3] == 'B' )
    {   
       // callflag=1;
    }
if(receivedData[itr-1] == 'o' &&receivedData[itr-2] == 't'  && receivedData[itr-3] == 's')
    {   
         audioplaystopFlag=1;
    }
if(receivedData[itr-1] == '1' && receivedData[itr-7] == 'D' /*&& dtmf_Modeselect_Flag == 0 */&& receivedData[itr-4] == 'F' )
    {
       //dtmf_on_Flag=1;
    }
if(receivedData[itr-1] == '0' && receivedData[itr-7] == 'D' /*&& dtmf_Modeselect_Flag == 0 */&& receivedData[itr-4] == 'F' )
    {
       //dtmf_off_Flag=1;
    }
if(receivedData[itr-1] == '3' && receivedData[itr-7] == 'D' && receivedData[itr-4] == 'F' )
    {
       dtmf_verbalvolt_Flag=1;
    }
		    /**************************************SMS FUNCTIONS*********************************************/
if(receivedData[itr-1] == '>' || receivedData[itr] == '>')//Checks receive buffer have data to send as SMS.
    {
       sendSmsflag=1;
    } 
if(receivedData[itr-5] == 'E' && receivedData[itr-3] == 'R' && receivedData[itr-2] == 'O' && receivedData[itr-1] == 'R')
    {
			errorFlag=1;
		}
if(receivedData[itr-21] == 'I' && receivedData[itr-20] == 'M' && receivedData[itr-19] == 'E' && receivedData[itr-18] == 'I')
    {
		  x=15;
			for(z=0;z<15;z++)
			{
				 imei_buf[z]=((receivedData[itr-x]));
         x--;	
			}
			 imei_buf[15]='\0';
		
		}
if(receivedData[itr-9] == 'C' && receivedData[itr-8] == 'S' && receivedData[itr-7] == 'Q')
    {
		  x=4;
			for(z=0;z<4;z++)
			{
				 cyc_buf[z]=((receivedData[itr-x])-'0');
      x--;	
			}
			errorpercent=cyc_buf[3];
		  dashboardsignal[3]=cyc_buf[0];
			dashboardsignal[3]=(dashboardsignal[3]*10)+cyc_buf[1];
			dashboardsignal[3] = (dashboardsignal[3] * 100) / 31;
		}
		
if(receivedData[itr-6] == 'A' && receivedData[itr-7] == 'D' && receivedData[itr-6] == 'C')
    {
		  x=4;
			for(z=0;z<4;z++)
			{
				 rtc_buf[z]=((receivedData[itr-x])-'0');
         x--;	
			}
		 /* dashboardsignal[3]=rtc_buf[0];
			dashboardsignal[3]=(dashboardsignal[3]*10)+rtc_buf[1];
			dashboardsignal[3]=(dashboardsignal[3]*10)+rtc_buf[2];
			dashboardsignal[3]=(dashboardsignal[3]*10)+rtc_buf[3];
			dashboardsignal[3] = (dashboardsignal[3] - 3600) * 100 / (4100 - 3600);*/	
			}
		
if(receivedData[itr-10] == 'N' && receivedData[itr-9] == 'E'  && receivedData[itr-8] == 'T' &&  receivedData[itr-1] == '0')
	       {
							netopenFlag=1;
				 }
if(receivedData[itr-9] == 'N' && receivedData[itr-8] == 'E'  && receivedData[itr-7] == 'C' &&  receivedData[itr-6] == 'T')
	{
							if(receivedData[itr-3] == '0' && receivedData[itr-1] == '0')
							{
								netopenFlag=1;
							}
	}
	///status bit///from App to controller
if( receivedData[itr-6] == '/' && receivedData[itr-5] == 'R'  && receivedData[itr-4] == 'T' &&  receivedData[itr-3] == 'D' &&  receivedData[itr-2] == 'A' &&  receivedData[itr-1] == 'T'  )
					{	 
						 
						 rtdataFlag=1; 							 
					}
					///sensor flag
if(receivedData[itr-6] == '/' && receivedData[itr-5] == 'S'  && receivedData[itr-4] == 'E' &&  receivedData[itr-3] == 'N' &&  receivedData[itr-2] == 'S' &&  receivedData[itr-1] == 'R'  )
					{						 
						 sensrFlag=1; 							 
					}					
					///user addition
if(receivedData[itr-6] == '/' && receivedData[itr-5] == 'U'  && receivedData[itr-4] == 'S' &&  receivedData[itr-3] == 'R' &&  receivedData[itr-2] == 'D' &&  receivedData[itr-1] == 'T'  )
					{ 
						usrdtFlag=1; 							 
					}
					////SMS enable disable APP to controller///
if(receivedData[itr-6] == '/' && receivedData[itr-5] == 'C'  && receivedData[itr-4] == 'O' &&  receivedData[itr-3] == 'M' &&  receivedData[itr-2] == 'M' &&  receivedData[itr-1] == 'T'  )
					{						 
						 commtFlag=1; 							 
					}
if(receivedData[itr-6] == '/' && receivedData[itr-5] == 'P'  && receivedData[itr-4] == 'R' &&  receivedData[itr-3] == 'O' &&  receivedData[itr-2] == 'G' &&  receivedData[itr-1] == 'M'  )
					{ 
						programFlag=1; 							 
					}				////Settings from APP to controller
				////Settings from APP to controller
if(receivedData[itr-6] == '/' && receivedData[itr-5] == 'W'  && receivedData[itr-4] == 'R' &&  receivedData[itr-3] == 'I' &&  receivedData[itr-2] == 'T' &&  receivedData[itr-1] == 'E'  )
					{
            appreceived=1;						
						mqttmodrecFlag=1; 
					}
if(receivedData[itr-6] == '/' && receivedData[itr-5] == 'A'  && receivedData[itr-4] == 'P' &&  receivedData[itr-3] == 'R' &&  receivedData[itr-2] == 'E' &&  receivedData[itr-1] == 'S'  )
					{						 
						 		apprespondFlag=1;					 
					}
					//+CMQTTRXPAYLOAD
if(receivedData[itr-17] == 'R' && receivedData[itr-16] == 'X' && receivedData[itr-14] == 'A' && receivedData[itr-13] == 'Y' && receivedData[itr-12] == 'L' && receivedData[itr-11] == 'O'  && receivedData[itr-10] == 'A' &&  receivedData[itr-9] == 'D')
					{
					mqttrecFlag=1;
					mqttdataFlag=0;
					a=0;			
					}
					
if(receivedData[itr-16] == 'R' && receivedData[itr-15] == 'X' && receivedData[itr-13] == 'A' && receivedData[itr-12] == 'Y' && receivedData[itr-11] == 'L' && receivedData[itr-10] == 'O'  && receivedData[itr-9] == 'A' &&  receivedData[itr-8] == 'D')
					{
					mqttrecFlag=1;
					mqttdataFlag=0;
					a=0;					
					}
					
if(mqttrecFlag==1)
					{
					if(receivedData[itr]>='0'&&receivedData[itr]<='9')
					{						
					recframe[a]=receivedData[itr];
					a++;
					}
					}					
					
if(receivedData[itr-6] == 'R' && receivedData[itr-5] == 'X' && receivedData[itr-4] == 'E' && receivedData[itr-3] == 'N'  && receivedData[itr-2] == 'D')
					{
						mqttcompFlag=1;
						mqttrecFlag=0;
			      initFlag=1;
						a=0;
					}					
if(receivedData[itr-8] == 'P' && receivedData[itr-7] == 'U'  && receivedData[itr-6] == 'B' && receivedData[itr-1] == '0')
					{
						pubcompFlag=1;
					}
if(receivedData[itr-8] == 'S'  && receivedData[itr-7] == 'U' &&  receivedData[itr-6] == 'B')
	       {
					if(receivedData[itr-3] == '0' && receivedData[itr-1] == '0')
							{
								subcompFlag=1;
							}
					}
					///status flag to send message
if(receivedData[itr-6] == 'S' && receivedData[itr-5] == 'T' && receivedData[itr-4] == 'A' && receivedData[itr-3] == 'T'  && receivedData[itr-2] == 'U' &&  receivedData[itr-1] == 'S')
					{
						statusFlag=1;
					}
/*if(receivedData[itr-6] == 'M' && receivedData[itr-5] == 'A' && receivedData[itr-4] == 'N' && receivedData[itr-3] == 'U'  && receivedData[itr-2] == 'A' &&  receivedData[itr-1] == 'L')
					{
						smsmanual=1;
						
					}
					
if( receivedData[itr-3] == 'A' && receivedData[itr-2] == 'U' && receivedData[itr-1] == 'T' )
	{
		       smsauto=1;
		
	}
if( receivedData[itr-3] == 'R' && receivedData[itr-2] == 'T' && receivedData[itr-1] == 'C' && receivedData[itr] == '\r')
	{
		      smsrtc=1;
		
	}
if( receivedData[itr-3] == 'C' && receivedData[itr-2] == 'Y' && receivedData[itr-1] == 'C' )
	{
		     smscyc=1;
		
	}
if(receivedData[itr-5] == 'R' && receivedData[itr-4] == 'E' && receivedData[itr-3] == 'S' && receivedData[itr-2] == 'E' && receivedData[itr-1] == 'T')
	{
		     smsreset=1;
		
	}*/
	if(receivedData[itr] == 'T'  && receivedData[itr - 2] == 'S'  &&  receivedData[itr - 3] == 'R' && itr < 4) //RS T -ic reset command 
	  {
              ic_reset = 1;
    }
if(receivedData[itr] == 'M'  && receivedData[itr - 2] == 'N'  &&  receivedData[itr - 3] == 'O' && itr < 4) 
	  {
              sms_on_Flag = 1;
    }
if(receivedData[itr] == 'M' && receivedData[itr - 2] == 'F'  && receivedData[itr - 3] == 'F' && receivedData[itr - 4] == 'O' && itr < 5) 
	  {
        //sms_off_Flag = 1;
    }
		
if(receivedData[itr-5] == 'V'  && receivedData[itr-4] == 'C' && itr<7)
		{
			   x=3;
		for(z=0;z<3;z++)
		{
			temp1_array[z]=((receivedData[itr-x]));
       x--;
		}
		temp1_array[3]=receivedData[itr-6];
		hvlvFlag=1;
		temp1_array[4]=	combine_setting(temp1_array[0],temp1_array[1],temp1_array[2]);
    }
							
if(receivedData[itr-7] == '2'  && receivedData[itr-5] == 'V'  && receivedData[itr-4] == 'C' )
	{
			x=3;
			for(z=0;z<3;z++)
			{
				 temp1_array[z]=((receivedData[itr-x]));
         x--;
			}
			temp1_array[3]=receivedData[itr-6];
			hvlvFlag2=1;
			temp1_array[4]=	combine_setting(temp1_array[0],temp1_array[1],temp1_array[2]);
			
 }
		
if(receivedData[itr-7] == '2' && receivedData[itr-6] == 'O' && receivedData[itr-5] == 'V' && receivedData[itr-4] == 'L'  )
	{
		x=3;
		for(z=0;z<3;z++)
		{
		temp1_array[z]=((receivedData[itr-x]));
		x--;
		}
		ovlFlag2=1;
		
	}

if( receivedData[itr-6] == 'O' && receivedData[itr-5] == 'V' && receivedData[itr-4] == 'L' && itr<7 )
	{
			x=3;
			for(z=0;z<3;z++)
			{
			temp1_array[z]=((receivedData[itr-x]));	
       x--;
			}
			ovlFlag=1;
			
	}
	
if(receivedData[itr-6] == 'D' && receivedData[itr-5] == 'R' && receivedData[itr-4] == 'Y' && itr<7)
	{
			x=3;
			for(z=0;z<3;z++)
			{
				 temp1_array[z]=((receivedData[itr-x]));
         x--;
			}
      dryrFlag=1;
			
	}
	
if(receivedData[itr-7] == '2' &&  receivedData[itr-6] == 'D' && receivedData[itr-5] == 'R' && receivedData[itr-4] == 'Y' )
	{
					x=3;
			for(z=0;z<3;z++)
			{
			temp1_array[z]=((receivedData[itr-x]));
       x--;
			}
    dryrFlag2=1;
			
			
  }
	
if( receivedData[itr-6] == 'D' && receivedData[itr-5] == 'R' && receivedData[itr-4] == 'T' && itr<7)
	{
			x=3;
			for(z=0;z<3;z++)
			{
				 temp1_array[z]=((receivedData[itr-x]));
				 x--;
			}
   drtFlag=1;
	 
	}
if(receivedData[itr-6] == 'O'  && receivedData[itr-4] == 'T'  && itr<9)
	{
		  x=3;
			for(z=0;z<3;z++)
			{
				 temp1_array[z]=((receivedData[itr-x]));	
         x--;
			}
			temp1_array[5]=receivedData[itr-7];
			temp1_array[3]=receivedData[itr-5];
			cyclicFlag=1;
			
			if(cyclicFlag==1 && (temp1_array[5] == 67 || temp1_array[5] == 83))
			{
					temp1_array[4]=	combine_setting(temp1_array[0],temp1_array[1],temp1_array[2]);
		}
  }

/*if(receivedData[itr-5] == '2' && receivedData[itr-4] == 'P' && receivedData[itr-3] == 'H'  && receivedData[itr-1] == 'B')
	{
if(receivedData[itr-2] == 'E')
{
	twophase=1;
}	
if(receivedData[itr-2] == 'D')
{
	twophase=2;
}
	memsave=1;
  sendsettingsupdated=1;
	}*/
if(receivedData[itr-4] == 'A' && receivedData[itr-3] == 'S' && receivedData[itr-2] == 'E' && receivedData[itr-1] == 'T' )
	{
		smsaset=1;

	}
if(receivedData[itr-4] == 'A' && receivedData[itr-3] == 'C' && receivedData[itr-2] == 'L' && receivedData[itr-1] == 'R' )
	{
		smsaclr=1;
	}
if(receivedData[itr-4] == 'S' && receivedData[itr-3] == 'E' && receivedData[itr-2] == 'M' && receivedData[itr-1] == 'I' )
	{
		smssemi=1;
}
if(receivedData[itr-10] == 'M' && receivedData[itr-9] == 'E' && receivedData[itr-7] == 'P' && receivedData[itr-6] == 'D' && receivedData[itr-5] == 'N' && receivedData[itr-3] == 'A' &&  receivedData[itr-2] == 'C' &&receivedData[itr-1] == 'T' )
	{
		netfail1=0;
    netconnected=1;
	}
if(
				receivedData[itr-13] == 'M' && 
				receivedData[itr-12] == 'E' &&
				receivedData[itr-10] == 'P' && 
				receivedData[itr-9] == 'D' &&
		    receivedData[itr-7] == 'N' && 
			  receivedData[itr-5] == 'D' && 
		    receivedData[itr-4] == 'E' &&  
		    receivedData[itr-3] == 'A' &&  
		    receivedData[itr-2] == 'C' &&
		    receivedData[itr-1] == 'T' )
	{
		netfail1=1;
netdisconnected=1;
netconnected=0;
network_timeout=0;
	
	}
				if(
				receivedData[itr-9] == 'M' && 
				receivedData[itr-8] == 'Q' &&
		    receivedData[itr-7] == 'T' && 
			  receivedData[itr-5] == 'N' && 
		    receivedData[itr-4] == 'O' &&  
		    receivedData[itr-3] == 'N' &&  
		    receivedData[itr-2] == 'E' &&
		    receivedData[itr-1] == 'T' )
	{
netdisconnected=1;
netconnected=0;
network_timeout=0;
	
	}
			if(
				receivedData[itr-10] == 'C' && 
				receivedData[itr-9] == 'O' &&
				receivedData[itr-8] == 'N' && 
				receivedData[itr-7] == 'N' &&
		    receivedData[itr-6] == 'L' && 
			  receivedData[itr-5] == 'O' && 
		    receivedData[itr-4] == 'S' &&  
		    receivedData[itr-3] == 'T' &&  
			receivedData[itr-2] == ':' )

	{
netdisconnected=1;
netconnected=0;
network_timeout=0;
	
	}
	
	
	
}
void reset_recindex(void) 
	{// received array clearing function 
    unsigned int l;
    for (l = 0; l < itr+1; l++) {
        receivedData[l] = 0;
    }
    itr = 0;
   
  }
void checkok_error(void){
	//AT ok response check function
	okFlag=0;
	errorFlag=0;
	mqtt_timeout=0;
  while (!okFlag && !errorFlag && mqtt_timeout <= 400);
	mqtt_timeout=0;
}
void checkforlessthansymbol(void)
{
	  errorFlag=0;
    mqtt_timeout=0;	
		sendSmsflag=0;
    while((!sendSmsflag && !errorFlag) && mqtt_timeout <= 400);
	  sendSmsflag=0;
}

void check_gsmCommands(void)
{
/*if(oneringmode == 0)
	{
  if(callflag ==1)
  { 
  simcom_audio("G");//g
  callflag =0;
  }
		
  if(dtmf_on_Flag==1)
  {
	if(mode==1 && rctval1<10)
	{
	MQTT_P1ON=1;
	reset1=1;
	}
	if(mode==5)
	{
	ontoff=0;
	semierror=1;
	reset1=1;
	refresh=1;
	}
  simcom_audio("MO");
  dtmf_on_Flag=0;
  }
  if( dtmf_off_Flag==1)
  {
	 relayoff=1;
   simcom_audio("MF");   				
   dtmf_off_Flag=0;
  }
	}*/
     //oneringmode=1;
		 oneringmode=0;
		if(oneringmode == 1 && callflag==1)// IVRS turn and turn off the pump 
		{
			if((mode==1)||(mode==5))
			{
			motorstate = !motorstate;
			if(motorstate==1)
			{
				if(mode==1 && rctval1<10)
				{
	      MQTT_P1ON=1;
		    reset1=1;	
	      }
	    if(mode==5)
			  {
	     ontoff=0;
	     semierror=1;
	     reset1=1;
	     refresh=1;
	      }
				simcom_audio("MO");
			}
			if(motorstate==0)
			{
				relayoff=1;
        simcom_audio("MF");				
			}
		  }
			callflag=0;
		  UART_Send_String("AT+CHUP\r");
      checkok_error();		
	  }
	if(ic_reset==1)// IC software reset using Massgae
		{
	poff=0;
	powersensed=0;
	CHIP_RST;	 
	ic_reset=0;
		}
	if(sms_on_Flag==1)// turn on the pump to the message 
     {
			if(mode2==1 && rctval<10)
				{
	      MQTT_P1ON=1;
		    pumponflag=1;
	      timercomplete=0;	
	      }
	    /*if(mode==5)
			  {
	     ontoff=0;
	     semierror=1;
	     reset1=1;
	     refresh=1;
	      }	*/
			sms_on_Flag=0;
    }
		 
if(sms_off_Flag==1)// turn off the pump to the message 
{  
		relayoff=1;
		sms_off_Flag=0;
}
if(hvlvFlag2==1)// hv message settings 
	{
		if(temp1_array[3]=='H')
				{
		     seting_group[4]=temp1_array[4];
				}
			  if(temp1_array[3]=='L')
				{
				seting_group[5]=temp1_array[4];
				}
     settingschanged();
			hvlvFlag2=0;	
}
		if(hvlvFlag==1)//lv message settings
			{
		if(temp1_array[3]=='H')
		{
		 seting_group[0]=temp1_array[4];
		 }
		if(temp1_array[3]=='L')
		 {
			seting_group[1]=temp1_array[4];
		 }
      settingschanged();
			hvlvFlag=0;	
			}	
     if(ovlFlag==1)//ovl message settings
			{
				seting_group[2]=	combine_setting(temp1_array[0],temp1_array[1],temp1_array[2]);
			settingschanged();	
			ovlFlag=0;	
			}	
			 if(ovlFlag2==1)
			{
				seting_group[6]=	combine_setting(temp1_array[0],temp1_array[1],temp1_array[2]);
			settingschanged();	
			ovlFlag2=0;	
			}
			if(dryrFlag==1)//dry message settings
			{
				seting_group[3]=	combine_setting(temp1_array[0],temp1_array[1],temp1_array[2]);
	    settingschanged();	
			dryrFlag=0;	
			}	
			if(drtFlag==1){
			seting_group[8]=	combine_setting(temp1_array[0],temp1_array[1],temp1_array[2]);
	    settingschanged();	
			drtFlag=0;	
			}	
			if(dryrFlag2==1){
			seting_group[7]=	combine_setting(temp1_array[0],temp1_array[1],temp1_array[2]);
			settingschanged();	
			dryrFlag2=0;	
			}
    if(cyclicFlag==1 && (temp1_array[5] == 67 || temp1_array[5] == 83))//cyclic mode ont and off timer settings not used in valve controller 
			{
			/*if(temp1_array[3]=='N')
			{
		    ontime=temp1_array[4];
			//	run2=ontime;
      }
			if(temp1_array[3]=='F')
			{
        offtime=temp1_array[4];
				run3=offtime;
			}
			settingschanged();	*/
			cyclicFlag=0;	
			/*if(mode==2 && oftimestart==1)
				{
			oftimestart=0;
			oftimestarterror=1;
		    }*/
			}	
if(smsmanual==1)//smsmanual mode setting for gsm
{
	mode=1;
	modechangemqtt();
	sendsettingsupdated=1;
	smsmanual=0;
}
if(smsauto==1)//sms auto mode setting for gsm
		{
     mode=3;
     modechangemqtt();
		 sendsettingsupdated=1;
			smsauto=0;
		}
	if(smssemi==1)//sms semi  mode setting for gsm
		{
	mode=5;
	modechangemqtt();	
	sendsettingsupdated=1;
		smssemi=0;
	}
		if(smsaclr==1)// sms auto clear 
		{
if(rctval<7)
	{
	autoclr=1;
	autosetclrprocess=1;
	autosetclrstart=1;
	MQTT_ACLR=1;
	sendsettingsupdated=1;
	}
	smsaclr=0;
}
		if(smsaset==1)// sms auto set 
		{
	if(rctval>7)
	{
	autoset=1;
	autosetclrprocess=1;
	autosetclrstart=1;
	MQTT_ASET=1;
	sendsettingsupdated=1;
	}
	smsaset=0;
}
  if(smsreset==1)// sms fault reset 
	{
	resetx=1;
	resetcount=200;	
  sendsettingsupdated=1;
	smsreset=0;
	}	
	if(smscyc==1)//cyclic mode change 
		{
    mode=2;
    modechangemqtt();
		sendsettingsupdated=1;
		smscyc=0;
		}
		if(smsrtc==1)//rtc mode change 
		{
    mode=4;
    modechangemqtt();
		sendsettingsupdated=1;
		smsrtc=0;
		}
///user addition///
	if(useradd==1)
	{
		// user adding  loop
	if((user==10)||(user==1))//admin or user1 will add 
	{ 
  if(*flags[2] == 1) 
	{
    addUser(2, NM2, 62521, "User 2 added");
  }
  if(*flags[3] == 1) 
	{
    addUser(3, NM3, 62531, "User 3 added");
  }
  if(*flags[4] == 1) 
	{
    addUser(4, NM4, 62541, "User 4 added");
  }
  }
    useradd=0;
   *flags[userIndex]=0;  
	}			
		
			if(sendsettingsupdated==1)
			{
				settingschanged();	
				sendsettingsupdated=0;
			}
		
if(statusFlag==1)
{
//user=9;
	if(motorstate==1)
	{
		// if motor is turned on it will send a message to mobile 
						//user=9;
			if(dashboardsignal[0]==3){
			sprintf(message, "pump on %dr,%dy,%db\nRA:%sA YA:%sA BA:%sA\r",
                    dispry, dispyb, dispbr,
                    ract_str, yact_str, bact_str);
			
			}
			if(dashboardsignal[0]==1)
			{
				 sprintf(message, "pump on %dr\n%sA\r", dispry, ract_str);
			}
		  sendSms(message);
			
	}
		if(motorstate==0)
		{
		// if motor is turned off it will send a message to mobile
		// if there is any error it will send a message to mobile
		if(dashboardsignal[0]==3)
		{
    		 sprintf(message, "pump off %dry,%dyb,%dbr\r",dispry,dispyb,dispbr);
		}
		if(dashboardsignal[0]==1)
		{
			 sprintf(message, "pump off %dr\n%sA\r", dispry, ract_str);
		}
			sendSms(message);
			if(start2==1)
			{
				if(volthigh==1)
				{
					sendSms("high voltage error");	
				}
				if(voltlow==1)
				{
					sendSms("low voltage error");	
				}
				if(phasefail==1)
				{
					if(rphfail==1)
					{
						sendSms("R - phasefail ");	
					}
					if(yphfail==1)
					{
						sendSms("Y - phasefail ");
					}
					if(bphfail==1)
					{
						sendSms("B - phasefail ");
					}
				}
				if(phasereversed==1)
				{
					sendSms("phase reverse error");	
				}
				if(phaseunbalance==1)
				{
					 sendSms("phase unbalance error");	
				}
			}
		}
		statusFlag=0;
	}		
}

void simcom_audio(const char *value)
	{
		// audio play function
		sprintf(inttostring, "AT+CCMXPLAY=\"c:/%s.amr\",1,0\r", value);
    UART_Send_String(inttostring);
		while(!audioplaystopFlag);
    audioplaystopFlag=0; 
  }
	
void sendSms(const char *smsData)
{
	////send sms to the stored users 
	numbers[0] = A1;
	numbers[1] = NM1;
	numbers[2] = NM2;
	numbers[3] = NM3;
	numbers[4] = NM4;
	numbers[5] = NM5;
	numbers[6] = NM6;
	user_smsdis[0]=1;
    for(n = 0; n < maxuser; n++) 
    {
        if (user_smsdis[n] == 1 && numbers[n] != NULL && numbers[n][0] != '\0')
        {
            sprintf(inttostring, "AT+CMGS=\"%s\"\r", numbers[n]);
            UART_Send_String(inttostring);
            checkforlessthansymbol();
            UART_Send_String(smsData);
            UART_Send_String("\x1a\r");
            checkok_error();
        }
    }
}
	
int combine_setting(int dat1,int dat2,int dat3)
 {
	// convert integer to char 
	dat1=dat1-48;
	dat2=dat2-48;
	dat3=dat3-48;
	
  //dat1=(dat1*10)+dat2;
  //dat1=(dat1*10)+dat3;
	dat4=(dat1*100)+(dat2*10)+dat3;
	 
	return dat4;
 }
 
void update_staus(void)
{
	// using this function you update the status off the device to the app 
STAT1=0;
STAT2=0;
STAT3=0;
STAT4=0;
STAT1 |= (pumponflag1  & 0x01) << 0;//1
STAT1 |= (lightonflag & 0x01) << 1;//2
STAT1 |= (autoset&0x01) << 2;//3
STAT1 |= (resetx & 0x01)<< 3;//4
STAT1 |= (phr & 0x01) << 4;//5
STAT1 |= (mqpofr & 0x01) << 5;//6
STAT1 |= (twophase & 0x01) << 6;//7(i == 0) 
STAT1 |= ((schedule_data[3]==1)?1:0) << 7;//2ph enable//8
		
STAT2 |= ((schedule_data[3]==2)?1:0) << 0;//9
STAT2 |= ((schedule_data[3]==3)?1:0) << 1;//10
STAT2 |= ((schedule_data[3]==4)?1:0) << 2;//11
STAT2 |= (mode1 & 0x01) << 3;//12
STAT2 |= (mode2 & 0x01) << 4;//13
STAT2 |= (mode3 & 0x01) << 5;//14
STAT2 |= (mode4 & 0x01) << 6;//15
STAT2 |= ((slavestat&(1<<0))?1:0) << 7;//1//16
	

STAT3 |= ((slavestat&(1<<1))?1:0) << 0;//17
STAT3 |= ((slavestat&(1<<2))?1:0) << 1;//18
STAT3 |= ((slavestat&(1<<3))?1:0) << 2;//19
STAT3 |= ((slavestat&(1<<4))?1:0) << 3;//20
STAT3 |= ((slavestat&(1<<5))?1:0) << 4;//21
STAT3 |= ((slavestat&(1<<6))?1:0) << 5;//22
STAT3 |= ((slavestat&(1<<7))?1:0) << 6;//23
STAT3 |= (0 & 0x01) << 7;//24

STAT4 |= (0 & 0x01) << 0;//25
STAT4 |= (0 & 0x01) << 1;//26
STAT4 |= (0 & 0x01) << 2;//27
STAT4 |= (0 & 0x01) << 3;//28
STAT4 |= (0 & 0x01) << 4;//29
STAT4 |= (0 & 0x01) << 5;//30
STAT4 |= (0 & 0x01) << 6;//31
STAT4 |=(autoclr&0x01)<<7;//32



/*void set_valve_state(void)
{
    int byteIndex = (valve_number - 1) / 8;
    int bitIndex  = (valve_number - 1) % 8;
    valve_state[byteIndex] |= (1 << bitIndex);
}*/
memsave=1;
statusdataFlag=1;
mqttpush();
statusdataFlag=0;
}   

void update_error(void)
{
		// if there is any error sending message to the mobile 
		if(onlyonceFlag==1)
		{
			MQTTERROR=0;
			MQTTERROR2=0;
			
			if(volthigh==1){
				MQTTERROR |= (1 & 0x01) << 0;
				//user=9;
				sendSms("high voltage");
			}
			
			if(voltlow==1){
				MQTTERROR |= (1 & 0x01) << 1;
				//user=9;
				sendSms("low voltage");
					
		    }
			if(ovld==1){
				 MQTTERROR |= (1 & 0x01) << 2;
								//user=9;
				sendSms("overload");
			}
			if(dry==1){
				 MQTTERROR |= (1 & 0x01) << 3;
				sendSms("dryrun");
			}	
		
			if(phaseunbalance==1){
				MQTTERROR |= (1 & 0x01) << 4;
				//user=9;
				sendSms("Phase unbalance");
			}
		
			if(phasefail==1)
	  		{
				MQTTERROR |= (1 & 0x01) << 5;
						//user=9;
				sendSms("phasefail");
			}
			if(phasereversed==1){
				MQTTERROR |= (1 & 0x01) << 6;
			//user=9;
				sendSms("phase reversed");
		   }
			if(slaveerror==1)
			{
				MQTTERROR |= (1 & 0x01) << 7;
			//user=9;
				sendSms("slave timeout");
			}
			if(nopump==1)
			{
				MQTTERROR2 |= (1 & 0x01) << 0;
			//user=9;
				sendSms("No pump");
			}
			if(unbalancecurrent==1)
			{
				MQTTERROR2 |=(1 & 0x01) << 1;
				sendSms("current unbalance");
			}
			
	  MQTTERROR2 |=(0 & 0x01) << 2;
		MQTTERROR2 |=(0 & 0x01) << 3;
		MQTTERROR2 |=(0 & 0x01) << 4;
		MQTTERROR2 |=(0 & 0x01) << 5;
		MQTTERROR2 |=(0 & 0x01) << 6;
		MQTTERROR2 |=(0 & 0x01) << 7;
		errFlag=1;
		mqttpush();
    errFlag=0;
		onlyonceFlag=0;
			errorclearflag=1;
		if(volthigh==0&&voltlow==0&&dry==0&&ovld==0&&phasefail==0&&phasereversed==0&&slaveerror==0&&nopump==0&&unbalancecurrent==0&&phaseunbalance==0)
			                        errorclearflag=0;
	}
	}   
	void modechangemqtt(void)
	{
		///if mode changed through the app it will perform only in gsm not in valve 
			if(mode==5)
		  {
			ontoff=1;
			semierror=1;
		  }
			if(mode==2 && oftimestart==1)
		  {
			oftimestart=0;
			oftimestarterror=1;
		  }
			//if(mode==3)
			//wlctop1=1;
		  if(mode ==4 && rtcstart==1)
		  {
			//rtcstart=0;
		  }
		      if(rctval>=10)
					relayoff=1;
		      refresh=1;
		      memsave=1;
	}

void display_contointernet(void)
{
	//connecting to network display 
fline();
for(z=0;z<16;z++)
{
enabler(con1[z]);
}
sline();
for(z=0;z<16;z++)
{
enabler(con2[z]);
}

}
void display_contoserver(void){
	//connecting to server display 
	sline();
for(z=0;z<16;z++)
{
enabler(con3[z]);
}
}
void connect_tomqtt(void)
{
	// connecting to mqtt server function 
display_contoserver();	
UART_Send_String("AT+CMQTTSTART\r");
checkok_error();
UART_Send_String("");
sprintf(message,"AT+CMQTTACCQ=0,\"%s\"\r",imei_buf);
UART_Send_String(message);
checkok_error();
//buzzer=0;
//UART_Send_String("AT+CMQTTCONNECT=0,\"tcp://broker.hivemq.com:1883\",60,1\r");
	//UART_Send_String("AT+CMQTTCONNECT=0,\"tcp://aicon.evolve.ind.in:8883\",60,1\r");
	UART_Send_String("AT+CMQTTCONNECT=0,\"tcp://uat-aicon.evolve.ind.in:8883\",60,1\r");
errorFlag=0;
netopenFlag=0;
mqtt_timeout=0;
mqttconfigfail=0;
while((!netopenFlag && !errorFlag) && mqtt_timeout<4000);
if(mqtt_timeout>2000 )
{
	mqttconfigfail=1;
	netdisconnect=1;
	mqtt_timeout=0;
	contoserflag=0; 
}
if(errorFlag==1)
{
	mqttconfigfail=1;
	netdisconnect=1;
	mqtt_timeout=0;
	contoserflag=0;
}

if(netopenFlag==1)
{
	// topic supscription loop 
for(n=0;n<7;n++)
	{
	  UART_Send_String("AT+CMQTTSUB=0,21,1\r\n");
    checkforlessthansymbol();
		mqtt_timeout=0;
	 if(n==0)
	{
    sprintf(message,"%s/WRITE\r",imei_buf);
	}
	if(n==1)
	{
		sprintf(message,"%s/RTDAT\r",imei_buf);
	}
	if(n==2)
	{
		sprintf(message,"%s/SENSR\r",imei_buf);
	}
	if(n==3)
	{
		sprintf(message,"%s/COMMT\r",imei_buf);
	}
	if(n==4)
	{
		sprintf(message,"%s/USRDT\r",imei_buf);
	}
	if(n==5)
	{
		sprintf(message,"%s/PROGM\r",imei_buf);
	}
	if(n==6)
	{
	 sprintf(message,"%s/APRES\r",imei_buf);
	}
	UART_Send_String(message);
  mqtt_timeout=0;
	while(!subcompFlag && mqtt_timeout<900);
  subcompFlag=0;
}
}


chip=0;
enabler(0x01);
chip=1;
refresh=1;
displayx=1;
}
void checkfor_authoriseduser(void)
{
	// check incoming message came from authorised user or not 
    int i,j;
    authuser = 0;
    user = 0;
    for (i = 0; i < 7; i++) 
		{
       int count = 0;
       for (j = 0; j < 10; j++) 
			 {
       if (recnumberon_incoming[j] == numbers[i][j])
           count++;     
       }
       if (count > 9) 
			 {
            user = (i == 0) ? 10 : i; 
            authuser = 1;
				    if(powersense==0)
					  sendonlyonce=0;
       }
    }
    if(!authuser) 
		{
        UART_Send_String("AT+CHUP\r");
        while (!okFlag);
        okFlag = 0;
    }
}

void twophaseonfunction(void)
{
	// two phase starting capacitor relay turn on and off  
	if(dashboardsignal[1]==1&&switched==1)
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
		if(relayoffcount>seting_group[11])
		{
			relayoffcount=0;
			relay2=0;
			switched=0;
		}
	}
}
}
void pumpoffunction(void)
{
	 if(relayoff==1)
      {
				pon=0;
				
				relayoff=0;
				reset1=0;
				ondelaycheck=0;
				ondelayok=0;
				Nop=0;
				
      if(((mode4==1)&&(timeron==1||timeron==3||timeron==5||timeron==7)&&(rtcstart==1))||(mode3==1&&cycle1==0&&rtcstart==1))
                           rtcstart=0;
			run7=0;
			run8=0;
		if(dashboardsignal[0]==1&&pumptype==1)
		{
			crmrly=0;
			refresh=1;	
		}
			if(dashboardsignal[0]==1&&pumptype==2)
			{
				stoprly=1;
				//pumpoff=1;
			}
			if(dashboardsignal[0]==3)
			{
				stoprly=1;
				//pumpoff=1;
			}
      start1=0;
			motorstate=0;
			pumpstatuschange=1;
		  pumpstatuschange_timeout=0;
			
			if((turnonv==1)&&((mode1==1&&pumponflag==0)||((mode2==1||mode3==1)&&cycle1==0)||((mode4==1)&&(rtcstart==0))||(slaveerror==1||modechanged==1||valvetrack==0)))
				{
					///valve turn off command execute only in this loop 
					turnonv=0;
					writevaluestate=1;
					motoroff=1;
					pumponflag=0;
					motoron=0;
					currunninggrp=0;
					pumponflag1=0;
						if((cycle1==0&&mode1!=1)||(modechanged==1))
							{
							cycle1=cycle;
							cyclestore=1;
								if(rtcstart==1)
								{
									rtcstart=0;
								}
							}
				}
			
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
				/*if((turnonv==1)&&((mode1==1&&pumponflag==0)||(mode2==1&&cycle1==0)||((mode3==1||mode4==1)&&(rtcstart==0))))
				{
					turnonv=0;
					writevaluestate=1;
					turnoffv=1;
					pumponflag=0;
					motoron=0;
				}*/
			}
}
void drtstartfunction(void)
{
if(dry==1&&restart==1&&errordisplay==1&&dryrestart==0&&vfault==0&&seting_group[8]>=1&& mode1!=1)
	{
		   dryrestartcount++;
		if(dryrestartcount>90)
		{
			dryrestartcount=0;
			dryrestart=1;
			drt1=seting_group[8];
			run3=drt1;
			drterror=1;
			/*if(mode==2)
			{
				oftimestart=0;
				run2=ontime;
				ontimererror=1;
		    oftimestarterror=1;
			}	*/
      refresh=1;
		}
	}
}

void autosetclrfunction(void)
{
  if((asetkey==1||aclrkey==1)&&(autoset==0&&autoclr==0&&autosetclrprocess==0)&&(autosetdone==0)&&(autosetclrshow==0))
  {
      autosetcountsubcount++;      //loop delay
    if(autosetcountsubcount>13)
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
      refresh=1;
    }
  }
	if(autosetclrprocess==1&&autosetdone==0&&autosetclrshow==0&&autosetclrstart==0)
	{
		autosetclrprocesssubcount++;//loop delay//to withstand the display for some time
		if(autosetclrprocesssubcount>3)
		{
			autosetclrprocesssubcount=0;
			autosetclrprocessmain++;
		}
		if(autosetclrprocessmain>1)
		{
			autosetclrprocessmain=0;
			autosetclrstart=1;
		}
	}
     autosetclr();
     if(autosetdone==1)
     {
      autosetdonesubcount++;
      if(autosetdonesubcount>10)
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
	  mqqtstatus=0;
      displayx=1;
      refresh=1;
      update_staus();
	  setting_split();
    }
  }
}

void mqqtupdatetime(void)
{
	//device data updating to the app using this loop
	if(simokFlag==1)
	{
		if(appopen==1)// if app is open then only it will execute 
		{
			counter1++;
			if(counter1>80)
			{
				counter1=0;
				cnt++;
				if(cnt>2)
				{
					 dashboardcal();
                     statuscnt=0;
					 cnt=0;
				}
			}
		}
	}
	if(netconnected==1 && simokFlag==1&&respondtoappFlag==1)
    {/////// if app is request to give device data then only this loop will execute 
	 if(AKCREG2&(1<<4))
     {
	  appopen=1;
	  datainvalid=1;
      mqttpush();
     }
     if((AKCREG2&(1<<4))==0)
     {
	   if(appopen)
	   {
	      appopen=0;
		  datainvalid=1;
          mqttpush();
	      counter1=0;
	   }
     }
	if(AKCREG1&(1<<4))//prgramx
    {
	 programsendFlag=1;
	 mqttpush();
    }
    if(AKCREG1&(1<<5))//settings
    {
	  //settingFlag=1;
	 setting_split();
    }
    if(AKCREG1&(1<<6))//communic
    {
	 commtFlag=1;
     mqttpush();
    }
    if(AKCREG1&(1<<7))//userdetl
    {
	 usrdtFlag1=1;
	 mqttpush();
    }
    if(AKCREG2&(1<<0))//sensor dat
    {
	 sensrFlag=1;
	 mqttpush();
    }
    if(AKCREG2&(1<<1))//dta
    {
	 dashboardcal();
     statuscnt=0;
    }
 if(AKCREG2&(1<<2)) //statregx
 {
	update_staus();	
 statusdataFlag=0;
 statuscnt=1;
 mqqtstatus=0;
 }
 if(AKCREG2&(1<<3))//errorlog
 {
	 onlyonceFlag=1;
	 update_error();
 }
 AKCREG2=0;
 AKCREG1=0;
	 respondtoappFlag=0;
 }
	/*if(simokFlag==1)
	{
	counter1++;
	if(counter1>160)//20sec
	{
  if(cnt==0)
	{
   setting_split();
		onlyonceFlag=1;
		update_error();
		programsendFlag=1;
		mqttpush();
  }		
		cnt++;
    counter1=0;
	}
	}
 if(netconnected==1 && simokFlag==1)
 {
 if((cnt==3)&&(statuscnt==0))// 
 {		
 update_staus();	
 statusdataFlag=0;
 statuscnt=1;
 mqqtstatus=0;
 }
 if(cnt>=4)//
 {
 dashboardcal();
 cnt=1;
 statuscnt=0;
 }
 }*/		
}

void dashboardcal(void)
{
	///dta frame reading and send to the app funcion 
	UART_Send_String("AT+CSQ\r");
    checkok_error(); 
    UART_Send_String("AT+CADC2=2\r");
    checkok_error(); 
    readdashboarddata();
}

void displayscrolltime(void)
{
	if(((autosetclrprocess==0))||(scrolltype==1&&dryrestart==1&&restart==1&&autosetclrprocess==0))
	{
		counter++;
	}
	switchscan();
	if(counter>40)//5sec
	{
		refresh=1;
		if(nopump==0)
			displayx++;

		if(errordisplay==1&&dryrestart==0)
			displayx=1;
		if(pumpsensed==0&&dashboardsignal[0]==1&&displayx==3)
			displayx=4;	
		counter=0;
		if(displayx>4)
		{
			displayx=1;
		}  
	}	
}

void faultresetfunction(void)
{
	/////// fault reset function 
	if(resetx==1&&resetcount==0)
	{
		//if(mode1==1)
		//{
		  start2=0;
		  volthigh=0;
		  voltlow=0;	
		  rphfail=0;
		  yphfail=0;
		  bphfail=0;
		  phaseunbalance=0;			 
		//}
		  resetx=0;
		  start2=0;
    	    if(slaveerror==0)
	    	  restart=0;
        unbalancecurrent=0;
        ubc1=0;
		 //ovld=0;
		if(dry==1)
		{
			dry=0;
			drywrite=1;
		}
		 //dry=0;
		 phasefail=0;
		 displayx=1;
		 counter=0;
		 errordisplay=0;
		 refresh=1;
		 start1=0;
		 start=0;

		 if(dryrestart==1)
		 {
		     dryrestart=0;
			 drt1=seting_group[8];
			 drtstore=1;
			 drterror=1;
		 }
		 if(ovld==1)
		 {
			 ovld=0;
			 pofrflag=1;
		 }
		/* if(slaveerror==1)
		 {
			 pumponflag=0;
			 timercomplete=0;
			 slaveerror=0;
			slaveerrornumber=0;
		 }*/
		 vfault=0;
		 displayrefresh=0;
		 sense=0;
		//adctime=0;
		 pon=0;
		 drterror=1;
		 nopump=0;
		// slaveerror=0;
		 if(errorclearflag==1)
		    onlyonceFlag=1;
		 mqqtstatus=0;
	 }
}

void oftimerfunction(void)
{	
	////////dry restart timer decrement function 
   if(((dry==1)&&(rctval1<10)&&(restart==1)&&(dryrestart==1)))//oftime
   {
	  run7++;
	  if(run7>=145)//158
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
			ontoff=0;
			dry=0;
			restart=0;
			dryrestart=0;
			errordisplay=0;
			displayx=2;
			counter=0;
			run3=seting_group[8];
			drt1=seting_group[8];
			sense=0;
			adctime=0;
			start1=0;
			refresh=1;
			drterror=1;
			drtstore=1;
		}
	//	if(restart==0)
	 // run3=offtime;
		//if(restart==0)
		//oftimererror=1;
		counter=0;
		refresh=1;
	  }
	}
}

void powereepromreadfunction(void)
{
	/// this function only use to identify the input voltage is EB line or battery 
	// If input voltage comes from EB then only the device will read eeprom and ct refrence and voltage scan and other while(1) functions
	// we implemented this funcion to avoid unwanted error behaviour 
	resultx=0;
  //ADCCON0=0X09;
	ADCCON0=0X01;
	
  for(scan=0;scan<250;scan++)
  {
	  clr_ADCCON0_ADCF;
      set_ADCCON0_ADCS;                  
      while(ADCF == 0);
      result1=(ADCRH<<4)+(ADCRL&0x0F);
	  result1=result1>>2;
	  resultx=result1+resultx;
  }
    poweravailable=resultx/250;
	if(poweravailable>=800)
	{
		powereepromread=1;
	}
	if(poweravailable<800)
	{
		powereepromread=0;
	}
	
}
void powersupplysensefunction(void)
{
	/// this function only use to identify the input voltage is EB line or battery 
	resultx=0;
  //ADCCON0=0X09;
	ADCCON0=0x01;
  for(scan=0;scan<250;scan++)
	{
	  clr_ADCCON0_ADCF;
    set_ADCCON0_ADCS;                  
    while(ADCF == 0);
    result1=(ADCRH<<4)+(ADCRL&0x0F);
		result1=result1>>2;
		resultx=result1+resultx;
	}
    poweravailable=resultx/250;
  if(poweravailable>=800)
	{
			powersense=1;
			reenter=0;	
		  MQTT_POWER=1;
	}
	if(poweravailable<800)
	{
	powersense=0;
	poff=1;
	MQTT_POWER=0;
	if(reenter==0)
	{
  SFRS=0;
	P0M1=0XFF;
	P0M2=0X00;
	P01_PUSHPULL_MODE;
	
	SFRS=0;
	P1M1=0xFE;
	P1M2=0X01;
	//P16_PUSHPULL_MODE;
	P1=0x01;
		
	/*SFRS=2;when enabling the timer getting disabled
	P2M1=0xFF;
	P2M2=0X00;
	SFRS=0;*/
	P3M1=0xFF;
	P3M2=0X00;
	SFRS=0;
  reenter=1;
	}		
	}
}

void pumponoffmessagefunction(void)
{
	//Format all currents as XX.X (divide by 10)
	sprintf(ract_str, "%d.%d", rctval / 10, rctval % 10);
	sprintf(yact_str, "%d.%d", yctval / 10, yctval % 10);
	sprintf(bact_str, "%d.%d", bctval / 10, bctval % 10);
    
	if(pumpstatuschange == 1)
	{
      if((rctval >= 10)&&(start1==1)&&(pon==1)) // Pump ON
      {
		reset_messageindex();
        user=9;
        if (dashboardsignal[0]== 3)
        {
            sprintf(message, "pump on %dr,%dy,%db\nRA:%sA YA:%sA BA:%sA\r",
                    dispry, dispyb, dispbr,
                    ract_str, yact_str, bact_str);
        }
        else if (dashboardsignal[0]== 1)
        {
            sprintf(message, "pump on %dr\n%sA\r", dispry, ract_str);
        }
		sendSms(message);
        pumpstatuschange = 0;
        update_staus();
		dashboardcal();
      } 
      if((rctval<10)&&(pon==0)) // Pump OFF
      {
        if (dashboardsignal[0]== 3)
        {
             sprintf(message, "pump off %dry,%dyb,%dbr\nRA:%sA YA:%sA BA:%sA\r",
             dispry, dispyb, dispbr,
             ract_str, yact_str, bact_str);
        }
        else if(dashboardsignal[0]== 1)
        {
            sprintf(message,"pump off %dr\n%sA\r",dispry,ract_str);
        }
				sendSms(message);
                pumpstatuschange = 0;
                update_staus();
				dashboardcal();
      }
	} 
}	

/*void ontimerfunction(void)
{
if(((mode==5 && ontoff==0)&&(rctval1>7)&&(restart==0)))//ontime
{
	    run7++;
	  if(run7>=145)//158
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
		if(mode==2)//cyclic
    oftimestart=1;	
	  relayoff=1;
	  run2=ontime;
		ontimererror=1;
		if(mode==2)
		{
		run3=offtime;
		oftimererror=1;
		}
		if(mode==5)
		{
		ontoff=1;
		semierror=1;
		}
		rctflag=1;
		refresh=1;
	  }
}	
}*/
void pumponfunction(void)
{
	//pump on trigger function 
//if((rctval1<10)&&(pon==0)&&(autosetclrprocess==0)&&(sense>2)&&(volthigh==0)&&(voltlow==0)&&(restart==0)&&(start2==0)&&(phasereversed==0)&&(motoron==1))
 if((pon==0)&&(autosetclrprocess==0)&&(volthigh==0)&&(voltlow==0)&&(restart==0)&&(start2==0)&&(phasereversed==0)&&(motoron==1)&&(mode3==1||mode4==1||mode2==1||mode1==1))
	{	
	reset1=1;
	pon=1;
  }
if(reset1&&ondelayok) 
{
		pumponflag1=1;
        reset1=0;
		ondelayok=0;
        if(dashboardsignal[0]==1) 
		{
           crmrly=1;
           crmon=1;
        } 
        else if(dashboardsignal[0]==3)
		{
          startrly=1;
          pumpon=1;
        }
					motorstate=1;
			pumpstatuschange=1;
		  pumpstatuschange_timeout=0;
	    	pon=1;
		    
}

if(crmon&&dashboardsignal[0]==1) 
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
if(motoron==1&&mode1!=1&&pon==1)
 currunninggrp=groupinc;
}

void mqttpush(void)
{
	// this function push the all data to the mobile app 
if(netconnected==1 || mqttconfigfail==0)
{
UART_Send_String("AT+CMQTTTOPIC=0,24\r");
checkforlessthansymbol();
	
if(datainvalid==1)
{
    RESENDFRAME1|=(appopen<<5);
	sendMQTTTopic("RESPONSE");
    sendMQTTPayloadLength(32);
	sendStatusBits(RESENDFRAME1);
    sendStatusBits(RESENDFRAME2);
	checkok_error();
	delayMs(5000);
	RESENDFRAME1=0;
	RESENDFRAME2=0;
	datainvalid=0;
}
if(errFlag==1)
{
sendMQTTTopic("ERRORLOG");
sendMQTTPayloadLength(32);
sendStatusBits(MQTTERROR);
sendStatusBits(MQTTERROR2);
checkok_error();
delayMs(5000);
errFlag=0;
}

if(mqttdataFlag==1)
{
	sendMQTTTopic("DTAFRAME");
	sendMQTTPayloadLength(256);//252
    values[0] = dispry;
    values[1] = dispyb;
    values[2] = dispbr;
    values[3] = rphvolt;
    values[4] = yphvolt;
    values[5] = bphvolt;
    values[6] = rctval;
    values[7] = yctval;
    values[8] = bctval;
    for(v =0; v <9; v++) 
	{
	   temp_array[0]=values[v]/100;
	   temp_array[1]=(values[v]%100)/10;
	   temp_array[2]=(values[v]%100)%10;
	   int3_to_string(message, temp_array[0], temp_array[1], temp_array[2]);
       UART_Send_String(message);
    }
	////////////we implement in future these value we are manually set some to test the app
		dashboardsignal[2]=30;//t-level//3signal,4 battery
		dashboardsignal[5]=80;//1
		dashboardsignal[6]=70;//2
		dashboardsignal[7]=90;//3
		dashboardsignal[8]=80;//4
		dashboardsignal[9]=60;//5
		dashboardsignal[10]=80;//6
		dashboardsignal[11]=80;//7
		dashboardsignal[12]=50;//8
		dashboardsignal[13]=81;//9
		dashboardsignal[14]=80;//10
		dashboardsignal[15]=80;//11
		dashboardsignal[16]=80;//12
		dashboardsignal[17]=50;//13
		dashboardsignal[18]=81;//14
		dashboardsignal[19]=93;//15
		dashboardsignal[20]=98;//16
		
		dashboardsignal[21]=80;//1
		dashboardsignal[22]=70;//2
		dashboardsignal[23]=90;//3
		dashboardsignal[24]=80;//4
		dashboardsignal[25]=60;//5
		dashboardsignal[26]=80;//6
		dashboardsignal[27]=80;//7
		dashboardsignal[28]=50;//8
		dashboardsignal[29]=81;//9
		dashboardsignal[30]=80;//10
		dashboardsignal[31]=80;//11
		dashboardsignal[32]=80;//12
		dashboardsignal[33]=50;//13
		dashboardsignal[34]=81;//14
		dashboardsignal[35]=93;//15
		dashboardsignal[36]=98;//16
		
		
		dashboardsignal[37]=80;//1
		dashboardsignal[38]=70;//2
		dashboardsignal[39]=90;//3
		dashboardsignal[40]=80;//4
		dashboardsignal[41]=60;//5
		dashboardsignal[42]=80;//6
		dashboardsignal[43]=80;//7
		dashboardsignal[44]=50;//8
		dashboardsignal[45]=81;//9
		dashboardsignal[46]=80;//10
		dashboardsignal[47]=80;//11
		dashboardsignal[48]=80;//12
		dashboardsignal[49]=50;//13
		dashboardsignal[50]=81;//14
		dashboardsignal[51]=93;//15
		dashboardsignal[52]=98;//16
		//////////////////////////////////////////////////////
    for(v =0; v <53; v++)//53 
	{
	   temp_array[0]=dashboardsignal[v]/100;//dashboardsignal[0]= phase,1=twophaseslected,2=RSSI,3=batterypercent,16-lora Batterypercent 1-16,16- battery percent solar 1-16,16-signal strenght loara 1-16;
	   temp_array[1]=(dashboardsignal[v]%100)/10;
	   temp_array[2]=(dashboardsignal[v]%100)%10;
	   int3_to_string(message, temp_array[0], temp_array[1], temp_array[2]);
       UART_Send_String(message);
    }	
		
	   temp_array[0]=run2/100;
	   temp_array[1]=(run2%100)/10;
	   temp_array[2]=(run2%100)%10;
	   int3_to_string(message, temp_array[0], temp_array[1], temp_array[2]);
       UART_Send_String(message);
	   temp_array[0]=currunninggrp/100;
	   temp_array[1]=(currunninggrp%100)/10;
	   temp_array[2]=(currunninggrp%100)%10;
	   int3_to_string(message, temp_array[0], temp_array[1], temp_array[2]);
       UART_Send_String(message);
    
       checkok_error();
	   delayMs(5000);
       mqttdataFlag=0;
}
if(statusdataFlag == 1) 
{
              reset_recindex();
			  sendMQTTTopic("STATREGX");
              sendMQTTPayloadLength(370);
			  sendStatusBits(STAT1);
			  sendStatusBits(STAT2);
			  sendStatusBits(STAT3);
			  sendStatusBits(STAT4);
			  UART_Send((MQTT_POWER+'0'));
			  UART_Send(',');
			  for(v=0;v<19;v++)
			  {
                    sendStatusBits(valve_state[v]);
			  }
              checkok_error();
			  delayMs(5000);
              statusdataFlag = 0;
}
		
if(programsendFlag==1)
{
	//slaveerrornumber=2;
    reset_recindex();
	sendMQTTTopic("PROGRAMX");
	reset_recindex();
	sendMQTTPayloadLength(742);//739
	
	temp_array[0]=maxvalves/100;
	temp_array[1]=(maxvalves%100)/10;
	temp_array[2]=(maxvalves%100)%10;
	int3_to_string(message, temp_array[0], temp_array[1], temp_array[2]);
    UART_Send_String(message);
	 
 for(z = 0; z < 64; z ++) 
  {
	temp_array[0]=group_ontime[z]/10;
	temp_array[1]=group_ontime[z]%10;
	int2_to_string(message, temp_array[0],temp_array[1]);
    UART_Send_String(message);
  }
 for(z = 0; z < 29; z ++)//16 
  {
	temp_array[0]=schedule_data[z]/10;
	temp_array[1]=schedule_data[z]%10;
	int2_to_string(message, temp_array[0],temp_array[1]);
    UART_Send_String(message);
	}
	
	for(z = 0; z < 3; z ++)//16 
  {
	temp_array[0]=max_liters[z]/10;
	temp_array[1]=max_liters[z]%10;
	int2_to_string(message, temp_array[0],temp_array[1]);
  UART_Send_String(message);
	}
	
 for (z = 0; z <150; z ++)//450
  {
	temp_array[0]=valve_group[z]/10;
	temp_array[1]=valve_group[z]%10;
	int2_to_string(message, temp_array[0],temp_array[1]);
    UART_Send_String(message);
  }	
    checkok_error();
	delayMs(5000);
    programsendFlag=0;
}	
if(settingFlag==1)
{
	reset_recindex();
	sendMQTTTopic("SETTINGS");
	reset_recindex();
	sendMQTTPayloadLength(75);//59//75
	for(v=0;v<19;v++) //19
	  {
	   temp_array[0]=seting_group[v]/100;
	   temp_array[1]=(seting_group[v]%100)/10;
	   temp_array[2]=(seting_group[v]%100)%10;
	   int3_to_string(message, temp_array[0], temp_array[1], temp_array[2]);
       UART_Send_String(message);
    }
checkok_error();
delayMs(5000);
settingFlag=0;	
}
if(sensrFlag==1)
{
	sendMQTTTopic("SENSRDAT");
	sendMQTTPayloadLength(24);
	q=0;
    for(z = 0; z <12; z++)	
	{
		message[q++]='0';
		message[q++]=',';
    
    }
	UART_Send_String(message);
  /*for(z = 0; z <= 24; z += 2) 
	{
  sprintf(message, "%c,", recframe[z]);
  UART_Send_String(message);
  }*/
  checkok_error();
	delayMs(5000);
  sensrFlag=0;
}
if(usrdtFlag1==1)
{
	sendMQTTTopic("USERDETL");
	sendMQTTPayloadLength(44);

for( q = 1; q <= 4; q++) 
{
    for ( n = 0; n < 10; n++)
  	{
        inttostring[0] = numbers[q][n];  // Assign character directly
        inttostring[1] = '\0';               // Null-terminate the string
        UART_Send_String(inttostring);
    }
    UART_Send_String(",");//Add comma after each number
}
checkok_error();
delayMs(5000);
usrdtFlag1=0;
}

if(commtFlag==1)
{
sendMQTTTopic("COMMUNIC");
sendMQTTPayloadLength(26);//26
reset_messageindex();
for(z = 0; z <13; z ++)	
	{
		message[q++]=(user_smsdis[z+1]+'0');
		message[q++]=',';
   // sprintf(message,"%d,",user_smsdis[q+1]);
    
  }
	UART_Send_String(message);
    checkok_error();
	delayMs(5000);
    commtFlag=0;
}

		UART_Send_String("AT+CMQTTPUB=0,2,60\r");
		pubcompFlag=0;
		mqtt_timeout=0;
		while(!pubcompFlag && mqtt_timeout<800);
		mqtt_timeout=0;
	
}
}

void mqtt_disconnect(void)
{
	while(!pubcompFlag&& mqtt_timeout<800);
	pubcompFlag=0;
	UART_Send_String("AT+CMQTTDISC=0,5\r");
	checkok_error();
	UART_Send_String("AT+CMQTTREL=0\r");	
	checkok_error();
	UART_Send_String("AT+CMQTTSTOP\r");	
	checkok_error();	
}
             
void gsm_routine(void)
{
	if(powersense==1)
	{
	switchscan();
	if((netdisconnected==1 || mqttconfigfail==1) && simcomrstcount<4)
		{
		simcomrstcount++;
    rst_simcom();	
	  netdisconnected=0;
		mqttconfigfail=0;
    }
	if(simcomrstcount>3 && network_timeout>30000)
		{
	  simcomrstcount=0;	
		network_timeout=0;
	  }
	if(sendsettingflag==1 && netconnected==1)
	{
		setting_split();
		sendsettingflag=0;
	}
	update_error();
	servertocontroller();
	//update_settings();
temp_timearray[0]=runningtime/1000;
temp_timearray[1]=(runningtime%1000)/100;
temp_timearray[2]=(runningtime%100)/10;
temp_timearray[3]=(runningtime%100)%10;
}
	
if(*flags[0] == 1) 
	{
    addUser(0, A1, 62501, "Super admin added");
  }
if(*flags[1] == 1) 
	{
    addUser(1, NM1, 62511, "User 1 added");
  }
	
if(messageFlag==1)
{
user=0;
checkfor_authoriseduser();
messageFlag=0;
}

if(user==0 && authuser==0)
{
sms_on_Flag=0;
sms_off_Flag=0;
statusFlag=0;
smsmanual=0;
smsauto=0;
smsrtc=0;
smscyc=0;
smsreset=0;
hvlvFlag=0;
hvlvFlag2=0;
ovlFlag2=0;
ovlFlag=0;
dryrFlag=0;
dryrFlag2=0;
drtFlag=0;
cyclicFlag=0;
smsaset=0;
smsaclr=0;
smssemi=0;
}
///////////////////////////////////check the received commands if the number is from a authorised user
if(user>0 && authuser==1)
//if(user>0 || authuser==1)
{
if(powersense==0 && sendonlyonce==0)
{
UART_Send_String("AT\r");
checkok_error();
sprintf(message,"power off\r");
sendSms(message);
UART_Send_String("AT+CHUP\r");
checkok_error();
sendonlyonce=1;		
}
if(powersense==1)
{				
check_gsmCommands();
}
}
onlywhenmodulereset();
}

void addUser(uint8_t flagIndex, char *destBuf, uint16_t baseAddr, const char *msg)
{
    uint8_t z;
	  IE &= ~(1 << 7);
	
	  if((destBuf[0] < 1)||(destBuf[0] > 9))  
    {
        maxuser++;  // Increment only once for empty slot
        Write_DATAFLASH_BYTE1(61077, maxuser);
    }
    for(z = 0; z < 10; z++) 
		{
       destBuf[z] = num[z];
       Write_DATAFLASH_BYTE1(baseAddr + z, num[z]);  
    }
    IE |= (1 << 7);
    destBuf[10] = '\0';
		sendSmsToUse(msg, destBuf);
    //sendSms(msg);
    reset_recindex();
    *flags[flagIndex]=0;
}

void sendSmsToUse(const char *smsData, const char *phoneNumber)
{
	if(phoneNumber != NULL && phoneNumber[0]!='\0')
	{
		sprintf(inttostring, "AT+CMGS=\"%s\"\r",phoneNumber);
		UART_Send_String(inttostring);
		checkforlessthansymbol();
		UART_Send_String(smsData);
		UART_Send_String("\x1a\r");
		checkok_error();
	}
}
	
void onlywhenmodulereset(void)
{
	if(moduleReadyFlag==1)        
  {
  moduleReadyFlag=0;
  gsm_reconnect();
	mqtt_timeout=0;
	while(!netconnected && mqtt_timeout<1500);
	mqtt_timeout=0;
		if(apnnotdetect==1&&pdpnotdetect==1)
		{
			if(network==1)
			{
                UART_Send_String("AT+CGDCONT=1,\"IPV4V6\",\"airtelgprs.com\"\r");
			}
			if(network==2)
			{
				UART_Send_String("AT+CGDCONT=1,\"IPV4V6\",\"jionet\"\r");
			}
			if(network==3)
			{
				UART_Send_String("AT+CGDCONT=1,\"IPV4V6\",\"www\"\r");
			}
			if(network==4)
			{
				UART_Send_String("AT+CGDCONT=1,\"IP\",\"bsnlnet\"\r\n");
			}
            checkok_error();
			UART_Send_String("AT+CGACT=1,0\r");
			checkok_error();
			UART_Send_String("AT+CGACT=1,1\r");
			checkok_error();
			UART_Send_String("AT+CGPADDR=1\r");
			checkok_error();
			
	/*poff=0;
	powersensed=0;
	CHIP_RST;	 */
		}
	if(netconnected==1)
	{   
  connect_tomqtt();
	}   
	if(powersense==0&&poweroffcount1==0)
  {     
  //user=10;					
  sprintf(message,"power off\r");
  sendSms(message);	
	if(netconnected==1&&netopenFlag==1)
	{          
	//MQTT_POWER=0;   
	update_staus();   
  mqtt_disconnect();       
	}
	poweroffcount1=1;
  }     
	                       
if(powersense==1 && poweroncount==0)
{
if(powersense==1)
{ 	
//user=9;	
sprintf(message,"power resumed %d%d:%d%d\n\r",temp_timearray[0],temp_timearray[1],temp_timearray[2],temp_timearray[3]);
sendSms(message);	
if(netconnected==1 && netopenFlag==1)
{
	  //MQTT_POWER=1;
	  update_staus();	
}
}
poweroncount=1;
}
}
}
     
void servertocontroller(void)
{
	//  this topic we are newly added for apres topic 
	//  it checks which topic data mobile app need from the device 
	
if(apprespondFlag==1&&initFlag==1)
{
	AKCREG1=0;
	AKCREG2=0;
    AKCREG1|=(((recframe[0] == '1') ? 1 : 0)<<0);
	AKCREG1|=(((recframe[1] == '1') ? 1 : 0)<<1);
    AKCREG1|=(((recframe[2] == '1') ? 1 : 0)<<2);
	AKCREG1|=(((recframe[3] == '1') ? 1 : 0)<<3);
	AKCREG1|=(((recframe[4] == '1') ? 1 : 0)<<4);
	AKCREG1|=(((recframe[5] == '1') ? 1 : 0)<<5);
	AKCREG1|=(((recframe[6] == '1') ? 1 : 0)<<6);
	AKCREG1|=(((recframe[7] == '1') ? 1 : 0)<<7);
	AKCREG2|=(((recframe[8] == '1') ? 1 : 0)<<0);
	AKCREG2|=(((recframe[9] == '1') ? 1 : 0)<<1);
	AKCREG2|=(((recframe[10] == '1') ? 1 : 0)<<2);
	AKCREG2|=(((recframe[11] == '1') ? 1 : 0)<<3);
	AKCREG2|=(((recframe[12] == '1') ? 1 : 0)<<4);
	apprespondFlag=0;
	respondtoappFlag=1;
	initFlag=0;
}
if(sensrFlag==1)
{
//mqttpush();
	     sensrFlag=0;
	     RESENDFRAME2=0;
		 RESENDFRAME1|=(1<<4);
		 datainvalid=1;
	     mqttpush();
}

if(initFlag==1 && programFlag==1)
{
	///////////program topic receiving topic
	   idx=0;
       maxvalves = combine_setting(recframe[idx], recframe[idx+1],recframe[idx+2]);//max valves
	   idx=3;
	   for(v = 0; v < 64; v++) //192   129
	   {
           group_ontime[v] = combine2digits(recframe[idx], recframe[idx+1]);///here group_ontime[0] has 1st group ontimer ,like so on..
           idx += 2;
       }
	   for(v = 0; v < 29; v++) //48   32
	   {
          schedule_data[v] = combine2digits(recframe[idx], recframe[idx+1]);///data like schedule timer 6 schedule max group like that
          idx += 2;
       }
	   for(v=0;v<3;v++)
	   {
		  max_liters[v] = combine2digits(recframe[idx], recframe[idx+1]);//max liters
		  idx += 2;
	   }
       for(v =0; v <150; v++) //450   300
	   {
         valve_group[v] = combine2digits(recframe[idx], recframe[idx+1]);//////here valve _group has 1st valve group number ,like so on
         idx += 2;
     }	
	
/*maxvalves=schedule_data[0];		 
maxgroups=schedule_data[1];
programnumber=schedule_data[2];
valvemode=schedule_data[3];
roundrobin=schedule_data[4];
mode=schedule_data[5];
cyc=schedule_data[6];
h1on=schedule_data[7];
h1off=schedule_data[8];
m1on=schedule_data[9];
m1off=schedule_data[10];
h2on=schedule_data[11];
h2off=schedule_data[12];
m2on=schedule_data[13];
m2off=schedule_data[14];
h3on=schedule_data[15];
h3off=schedule_data[16];
m3on=schedule_data[17];
m3off=schedule_data[18];
h4on=schedule_data[19];
h4off=schedule_data[20];
m4on=schedule_data[21];
m4off=schedule_data[22];
h5on=schedule_data[23];
h5off=schedule_data[24];
m5on=schedule_data[25];
m5off=schedule_data[26];
h6on=schedule_data[27];
h6off=schedule_data[28];
m6on=schedule_data[29];
m6off=schedule_data[30];*/

//if((mode2==1)||(mode3==1))
//{
    cycle=schedule_data[4];
	if(cycle1!=cycle)
	{
		cycle1=schedule_data[4];
		cyclestore=1;
	}
	progmstore=schedule_data[3];
	if(progmstore!=progmstore1)
	{
		progmstore1=progmstore;
		if(motoron==1&&mode1!=1)
		{
	modechanged=1;
	groupinc=1;
    groupincwrite=1;
	rtcstart=0;
	if(groupskip==1)
	{
	seting_group[14]=0;
	groupskip=0;	
    groupskipcheck=0;		
	groupskipwrite=1;
	}
}
}
//}
     memsave=1;
	 initFlag=0;
	 programFlag=0;
     //i command 6445
	 //programsendFlag=1;
	 //mqttpush();
     RESENDFRAME2=0;
	 RESENDFRAME1|=(1<<6);
	 datainvalid=1;
	 mqttpush();
}
if(commtFlag==1 && initFlag==1)
{
	///commt topic receive loop
	for (i = 0; i < 13; i++) 
	{
  		  user_smsdis[i+1] = (recframe[i] == '1') ? 1 : 0;
    }
/*	
user1calldis=1;
if(recframe[6]=='1')
user1calldis=0;

user2calldis=1;
if(recframe[7]=='1')
user2calldis=0;

user3calldis=1;
if(recframe[8]=='1')
user3calldis=0;

user4calldis=1;	
if(recframe[9]=='1')
user4calldis=0;

user5calldis=1;	
if(recframe[10]=='1')
user5calldis=0;

user6calldis=1;	
if(recframe[11]=='0')
user6calldis=0;

oneringmode=0;
if(recframe[12]=='1')
oneringmode=1;
*/
memsave=1;
/*for(q=0;q<14;q++)
{
user_smsdis[i+1]
int_txbuf[q]=recframe[q]-'0';	
}*/
initFlag=0;
mqttdataFlag=0;
settingFlag=0;
statusdataFlag=0;
// i command 6489
//mqttpush();
commtFlag=0;
RESENDFRAME2=0;
RESENDFRAME1|=(1<<2);
datainvalid=1;
mqttpush();
}
if(usrdtFlag==1&&initFlag==1)
{
	//usrdt receive loop  
    for (i = 0; i < 4; i++)// Loop for 4 sets: 0, 10, 20, 30
     {
       base = i * 10;  // Starting index of each 10-byte block
       //Check if first byte in each 10-byte block is not empty
       //if(recframe[base] != 0 && recframe[base] != ' ')
			 if(recframe[base] != ' ')
        {
        for (j = 0; j < 10; j++)
        {
          num[j] = recframe[base + j];// Copy 10 bytes to num[]
        }
        *flags[i + 1] = 1;//Set corresponding flag
				if(*flags[1] == 1) 
	      {
        addUser(1, NM1, 62511, "User 1 added");
        }
				if(*flags[2] == 1) 
	      {
        addUser(2, NM2, 62521, "User 2 added");
        }
				if(*flags[3] == 1) 
	      {
        addUser(3, NM3, 62531, "User 3 added");
        }
				if(*flags[4] == 1) 
	      {
        addUser(4, NM4, 62541, "User 4 added");
        }
		    }    
    }		   		    
usrdtFlag=0;
initFlag=0;
mqttdataFlag=0;
settingFlag=0;
statusdataFlag=0;
		//i command 6534
//usrdtFlag1=1;		
//mqttpush();
		usrdtFlag1=0;
		RESENDFRAME2=0;
		 RESENDFRAME1|=(1<<3);
		datainvalid=1;
		mqttpush();
		
}

if(initFlag==1  &&  mqttmodrecFlag==1 &&  rtdataFlag==0 )
{	
	///settings topic receive loop 
    idx=0;
      for(v =0; v <20; v++) 
	  {
			   if(v!=19)
				 {
                     seting_group[v] = combine_setting(recframe[idx], recframe[idx+1],recframe[idx+2]);
                     idx += 3;
				 }
				 if(v==19)
				 {
					 seting_group[v]=((recframe[idx]-48)*1000)+((recframe[idx+1]-48)*100)+((recframe[idx+2]-48)*10)+((recframe[idx+3]-48));
				     idx += 4;
				 }
      }
		 rtcmin=((((seting_group[19]%100)/10)*0x10)|((seting_group[19]%100)%10));
		 rtchour=((((seting_group[19]/100)/10)*0x10)|((seting_group[19]/100)%10));
         rtccredit=1;
			if(seting_group[14]!=swapgroupvalue)
			{	
				swapgroupvalue=seting_group[14];
				if((seting_group[14]>64)||((seting_group[14]>schedule_data[2])&&(mode4!=1))||(mode4==1&&seting_group[14]>6))
				{
					seting_group[14]=0;
				}
				
				if(seting_group[14]!=0)
				  groupskip=1;
				else
					groupskip=0;
				
				groupskipwrite=1;
				
			}
			if(seting_group[13]>99||seting_group[13]<=0)
			{
				seting_group[13]=5;
			}
    memsave=1;
    rtcupdateflag=1;
	errFlag=0;
	mqttdataFlag=0;
			//i command 6576
  //setting_split();
	initFlag=0;
	mqttmodrecFlag=0;
			
				//write->
		RESENDFRAME2=0;
	    RESENDFRAME1|= (1<<0);
		datainvalid=1;
		mqttpush();
	//valvecontrollerFlag=1;

}

if(initFlag==1  &&  rtdataFlag==1)
{ 
		//Rtdat topic receive loop 
		pumponflag   = (recframe[0] == '1');//this 1 expression is equal to if (recframe[0] == '1')pumponflag = 1; else pumponflag = 0;
		lightonflag  = (recframe[1] == '1');
		if(mqqtstatus==0)
		{
			autoset      = (recframe[2] == '1');
			autoclr      = (recframe[3] == '1');
			//resetx       = (recframe[4] == '1');
			resetx=(((start2==1)||(restart==1))&&(recframe[4]=='1'));
		}
		phr          = (recframe[5] == '1');
		mqpofr       = (recframe[6] == '1');
		twophase     = (recframe[7] == '1');
		mode1        = (recframe[8] == '1');	
		mode2        = (recframe[9] == '1');
		mode3        = (recframe[10] == '1');
		mode4        = (recframe[11] == '1');

		for(i=0;i<150;i++)  //valve start
		{
			byte_index = i / 8;
			bit_index  = i % 8;
			
			if(recframe[i + 12]=='1')   
				valve_state[byte_index]|=(1<< bit_index);
			else
				valve_state[byte_index]&=~(1<< bit_index);
		}
    standalonevalveon=0;
    modechecker1=0;
    modechecker1|=(mode1<<0);
	modechecker1|=(mode2<<1);
	modechecker1|=(mode3<<2);
	modechecker1|=(mode4<<3);

	if(modechecker2!=modechecker1)
	{
		modechecker2=modechecker1;
		modechanged=1;
		groupinc=1;
		groupincwrite=1;
		rtcstart=0;
		if(groupskip==1)
		{
			seting_group[14]=0;
			groupskip=0;	
			groupskipcheck=0;		
			groupskipwrite=1;
		}
		run2=0;
		ontimererror=1;
		//update_staus();   
	}

//memsave=1;
refresh=1;
rtdataFlag=0;
initFlag=0;

if(resetx==1&&(start2==1||restart==1)&&(autosetclrprocess==0))
	 resetcount=200;

	 if(autoset==1&&rctval>=10&&autosetclrprocess==0&&resetx==0)
	{
		autosetclrprocess=1;
		autosetclrstart=1;
	}

	if(autoclr==1&&rctval<10&&autosetclrprocess==0&&resetx==0)
	{
		autosetclrprocess=1;
		autosetclrstart=1;
	}
	// i command 6657
	update_staus();
	if((resetx==1||autoset==1||autoclr==1)&&(mqqtstatus==0))
	{
		mqqtstatus=1;
	}
	datainvalid=1;
	RESENDFRAME2=0;
	RESENDFRAME1|=(1<<1);
	mqttpush();
//if(pumponflag==1&&mode==2&&timercomplete==1&&pon==0&&motoron==0)
	//  timercomplete=0;
	}
}

void readdashboarddata(void)
{
	mqttdataFlag=1;
	mqttpush();
	mqttdataFlag=0;
}

void value_split(int val)
{
	temp_array[0]=val/100;
	temp_array[1]=(val%100)/10;
	temp_array[2]=(val%100)%10;
}

void setting_split(void)
{// settings send to the app 
	errFlag=0;
	mqttdataFlag=0;
	settingFlag=1;
	mqttpush();
	settingFlag=0;
	initFlag=0;
	mqttmodrecFlag=0;
}

void UART_Send_String(const char *dt)
{
    while (*dt != '\0') 
	{ 
		{
        	  UART_Send(*dt);
			  dt++;
    	}
	}
}
	
void UART_Send(unsigned char c)
{
    SFRS = 0;
    TI_1 = 0;
    SBUF_1 = c;
    while (!TI_1); // Wait until transmission complete
    TI_1 = 0; // Clear TI
}

void sendStatusBits(int statusReg) 
{
    reset_messageindex();
    for (z = 0; z <= 7; z++) 
	  {
        message[q++] = ((statusReg >> z) & 1) + '0';
        message[q++] = ',';
			
    }
    UART_Send_String(message);
}

void rst_simcom(void)
{
	SIMRESET=1;
	for(z=0;z<7;z++)
	{
		delay(100);//per value what is the time in us
	}
	SIMRESET=0;	
}

void settingschanged(void)
{
	memsave=1;
	//user=9;
	sendSms("settingsupdated");
	reset_recindex();
}

void reset_messageindex(void)
{
	for (z = 0; z <= 120; z++) 
	{
	    message[z] = 0;  // Add '0' or '1'                      // Add comma
  	}
	q=0;	
}

void sendMQTTTopic(const char* topicSuffix) 
{
    sprintf(message, "%s/%s\r", imei_buf, topicSuffix);
    UART_Send_String(message);
    checkok_error();
}

void sendMQTTPayloadLength(int len) 
{
    sprintf(inttostring, "AT+CMQTTPAYLOAD=0,%d\r", len);
    UART_Send_String(inttostring);
    checkforlessthansymbol();
}

void int3_to_string(char *dest, int a, int b, int c )
{
    dest[0] = a + '0';  // Convert int to char
    dest[1] = b + '0';
    dest[2] = c + '0';
    dest[3] = ',';      // Add comma
    dest[4] = '\0';     // Null terminator
}

void int2_to_string(char *dest, int a, int b )
{
    dest[0] = a + '0';  // Convert int to char
    dest[1] = b + '0';
    dest[2] = ',';      // Add comma
    dest[3] = '\0';     // Null terminator
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
	// unbalance current error decide function 
	if((start2==0)&&(rctval>=10)&&(volthigh==0)&&(voltlow==0)&&(start1==1)&&(dashboardsignal[0]==3)&&(restart==0)&&(ovld==0)&&(dry==0))
	{
		unbalancecurrent=0;
		if(compare(rct1,yct1)>(seting_group[16]*10))
		{
			unbalancecurrent=1;
		}
		
		if(compare(yct1,bct1)>(seting_group[16]*10))
		{
			unbalancecurrent=1;
		}
			
		if(compare(bct1,rct1)>(seting_group[16]*10))
		{
			unbalancecurrent=1;
		}
    }
	if(rctval<10&&restart==0&&unbalancecurrent==1&&pon==0)
	{
		unbalancecurrent=0;
	}
	
}

void checkvoltvalues(unsigned int ry1, unsigned int yb1,unsigned int br1)
{
	/// unbalance voltage decide function 
	phaseunbalance=0;
	if(compare(ry1,yb1)>seting_group[15])
	{
		phaseunbalance=1;
	}
	
	if(compare(yb1,br1)>seting_group[15])
	{
		phaseunbalance=1;
	}
		
	if(compare(br1,ry1)>seting_group[15])
	{
		phaseunbalance=1;
	}
}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
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

void modbusUartConfig(void)
{
	slaveaddress = 1;
	/*P30_PUSHPULL_MODE;
	P30 = 0;*/
	P16_PUSHPULL_MODE;
	P16 = 0;
	P06_PUSHPULL_MODE;
	P07_INPUT_MODE;
	/*Tx pin-0.6 and Rx pin-0.7 config							*/
	AUXR1 &= 0xFB;
	/*Tx-pin output on															*/
	P06 = 1;
	/*Mode-1 selected for asynchronous							*/
	SCON = 0x50;
	/*Double baudrate enable												*/
	PCON |= 0x80;
	/*Timer-3 selected															*/
	T3CON &= 0xF8; 
    T3CON |= 0x20; 
	/*Baudrate value as per datasheet								*/
	RH3 = 0xff;
    RL3 = 0x98;
	/*Timer3 start run to generate baudrate					*/
	T3CON |= 0x08;
	/*Uart Interrupt enabled												*/
	EA = 1;
	//ES = 1;
}

void receivedata(void)
{
    get=readMBframe();
// delay(20);
	if(get==0x10 || get==0x03)
	{
		readResponse();
		
	}
	if(get!=0x01&&get!=0x03)
	{
		//writefailure=1;
	}
//writefailure=get;
    REN=1;
    EA=1;
}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
			   MODBUS MASTER IMPLEMENTATION
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void writeResponse(void)
{
	REN=0;
	if(mbreadwrite==WRITE)
	{
		tx_buf[30]=slaveaddress;//Slave address.
		tx_buf[31]=0x10;//Function Code.
		tx_buf[32]=0x00;//start address msb
		tx_buf[33]=0x00;//start address lsb 
		tx_buf[34]=0x00;//quantity of registers msb
		tx_buf[35]=0x01;//quantity of registers lsb  
		tx_buf[36]=0x02;//byte count
		tx_buf[37]=(mbdata&0xFF00)>>8;//output values msb
		tx_buf[38]=(mbdata&0x00FF);//output values lsb
		
		CRC16(30,39,0);
		datalength = 39;
		tmpflag=1;
		writefailure=1;
	}
	if(mbreadwrite==READ)
	{
		tx_buf[30] = slaveaddress; // slave address
		tx_buf[31] = 0x03; // Function Code
		tx_buf[32] = 0x00; // Start address msb
		tx_buf[33] = 0x00; // Start address lsb
		tx_buf[34] = 0x00; // Number of Register msb.
		tx_buf[35] = 0x01; // Number of Registers lsb.
		CRC16(30,36,0);
		datalength = 36;
	}
	
 rtx_ctrl0=1;
 delayMs(5);
 for(startmod=30;startmod<datalength;startmod++)
 {
      SBUF=tx_buf[startmod];
      while(!TI);
	  TI=0;
	  delayMs(5);
 }
 SBUF=lowCRC;
 while(!TI);
 TI=0;
 delayMs(5);
 SBUF=highCRC;
 while(!TI);
 TI=0;
// delayMs(5);
 rtx_ctrl0=0;
 RI=0;
 REN=1;
}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
			   MODBUS SLAVE IMPLEMENTATION
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void readResponse(void)//response
{
    REN=0;
    rtx_ctrl0=0;
	if(mbreadwrite==WRITE)
	{
		CRC16(30,36,0);
		if(tx_buf[6] != lowCRC &&  tx_buf[7] != highCRC)
		{
			 //modbus error
			//slavebadresponse=1;
			 //restart=1;
			//relayoff=1;
			 //onlyonceFlag=1;
			 slavenoresponse=1;
		}
		if(tx_buf[6] == lowCRC &&  tx_buf[7] == highCRC)
		{
		slaveerrornumber&=~(1<<byte);
		}	 
	}
	if(mbreadwrite==READ)
	{
		for(store=3;store<15;store++)
		{
			store2=store-3;
			primebyte[store2]=tx_buf[store];
		}
		mbreaddata=((primebyte[0]<<8)&0xFF00)|primebyte[1];		
	}

 
}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
			   MODBUS MASTER IMPLEMENTATION
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
bit checkRxbuffer(void)
{
  wait=0;
  while((!RI) && (wait < 1540)) //1540
  wait++;//wait 5ms
  return RI;
}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
			   MODBUS MASTER IMPLEMENTATION
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

unsigned int readMBframe(void)
{
 char received=0;
 store2=0;
 rtx_ctrl0 = 0;   // DE=0, RE=0 ? enable receiver
 //REN = 1;         // Enable UART receiver
 //RI = 0; 
 rxdata = 0;
 xstopx = 0;
 while (!xstopx) 
 if(!checkRxbuffer()){
 	xstopx = 1;
 }
 else 
 {
  RI=0;
  if (rxdata<=20)
  { 
 	received=SBUF;		
    tx_buf[rxdata] = received;
    store2=tx_buf[rxdata];
    rxdata++;
  }
  REN=1;//need to check
 }
					
 if (rxdata > 17) 
 	return 2; //to many bytes in frame       

 //if (rxdata < 17) 
// return 0; //to few bytes frame

 if (tx_buf[0] != slaveaddress)
 	return 9; //wrong address

 if ((tx_buf[1] != 0x03 && tx_buf[1] != 0x10))
 	return 1; //wrong modbus function

 if(CRC16(0,(rxdata - 2),1)) 
 	return tx_buf[1]; //OK
 else
 	return 0;   //bad CRC
}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
			   MODBUS MASTER IMPLEMENTATION
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
bit  CRC16(unsigned char startaddress, unsigned char dataLength,char check) //CRC 16 for modbus checksum
{
 checksum=0xffff;
 lowCRC=0;
 highCRC=0;

 for (generate=startaddress; generate<dataLength; generate++)
 {
  checksum = checksum^(unsigned int)tx_buf[generate];
  for(value=8;value>0;value--)
   
   if((checksum)&0x0001)
   		checksum = (checksum>>1)^0xa001;
   else
   		checksum>>=1;
 }	
 highCRC = checksum>>8;
 checksum<<=8;
 lowCRC = checksum>>8;
 
 if (check==1)
 {	
 if ( (tx_buf[dataLength+1] == highCRC) && (tx_buf[dataLength] == lowCRC ))
   return 1;
 else
   return 0;
 }
 return 0;
}

void delayMs(unsigned int ms) {
    while (ms--) {
        int j = 120; // Approximate delay for 1ms
        while (j--);
    }
}

void modbusCommunication (char readorwrite)
{
		EA=0;
		// Master write response :
		mbreadwrite = readorwrite;
		writeResponse();
		while((!RI) && (wait < 5000))//wait for receive Interrupt for 50ms, now Data is for 500ms(147058)
		{
			 wait++;
		}
		if(wait>=5000)//30000
		{
		     wait=0;
		     slavenoresponse=1;
		}
		else
		{
		    wait=0;
			receivedata();//to check received data buffer before read response
		}
	    EA=1;
}

void rtcschedulercheck(void)
{
	// rtc schedule for manual RTC and RTC mode 
	// In manual-RTC mode we only read 1st schedule only 1st switch case only 
	if(start2==0&&restart==0&&volthigh==0&&voltlow==0)
	{
       //timercomplete=1;
       for (c=1;c<=6;c++) 
	   {
            ontimex=0,offtimex=0;
            switch(c)
			{
               case 1:
                  ontimex=((schedule_data[5]*100)+(schedule_data[6]));//56
							 if(mode3!=1)
							 {
               				     offtimex=((schedule_data[7]*100)+(schedule_data[8]));//78
							 }
							 if(mode3==1&&mode4!=1)
							 {
								 timeafterset();
							 }
                    break;
													 
               case 2:
                    ontimex=((schedule_data[9]*100)+(schedule_data[10]));//9 10
                    offtimex=((schedule_data[11]*100)+(schedule_data[12]));//11 12
                    break;
														
               case 3:
                    ontimex=((schedule_data[13]*100)+(schedule_data[14]));//13 14
                    offtimex=((schedule_data[15]*100)+(schedule_data[16]));//15 16             
                    break;
																
               case 4:
                    ontimex=((schedule_data[17]*100)+(schedule_data[18]));//17 18
                    offtimex=((schedule_data[19]*100)+(schedule_data[20]));//19 20
                    break;
															 
				case 5:
                    ontimex=((schedule_data[21]*100)+(schedule_data[22]));//21 22
                    offtimex=((schedule_data[23]*100)+(schedule_data[24]));//23 24              
					break;
																 
								
				case 6:
                    ontimex=((schedule_data[25]*100)+(schedule_data[26])); //25 26
                    offtimex=((schedule_data[27]*100)+(schedule_data[28]));//27 28
                    break;
               }

            if(ontimex>0&&offtimex>0)
			{
                presenttime=ontimex;
                futuretime=offtimex;
                timeok=0;
                timecompare();

                 onrt=1+(c-1)*2;
                 offrt=onrt+1;

               // if (timeok==1&&rctval1<10)
								if(timeok==1)// time is reaches the schedule on time this loop will execute
								{
                   						 	timeron=onrt;
                    						if(rtcstart==0)
											{
											//if(wlctop1==0)
											//{
                       						// reset1=1;
											//pon=1;
											if(mode4==1)
											{
												if(timeron%2==1) 
                      								groupinc=(timeron+1)/2;
											}
											if(mode4==1&&groupskip==1&&(seting_group[14]==groupinc))
											{
												groupskip=0;
												swapgroupvalue=seting_group[14]=0;
												//groupincwrite=1;
												groupskipwrite=1;
												memsave=1;
												skipstart=0;
											}
											if((mode3==1)||(mode4==1&&skipstart==0))
											{
												pumponflag=1;
												timercomplete=0;
											}
                        					rtcstart=1;
                        					onttimewrite=1;
                      }
                  }
                if(timeok==0&&timeron==onrt&&mode3!=1) // time is reaches the schedule off  time this loop will execute
				{
                    rtcstart=0;
                    timeron=offrt;
			    	onttimewrite=1;
                    relayoff=1;
					pon=0;
										pumponflag=0;
				            timercomplete=1;
										motoron=0;
										reset1=0;
								/*	if(mode==3)
									{
									groupinc=1;
									groupincwrite=1;
                  }*/
                  }
            }
							if(mode3==1&&c==1)
							{
								c=7;
							}
        }
				if(mode4==1&&groupskip==1&&rtcstart==0&&skipstart==0)// only execute when group skip enabled 
		                skipstart=1;
    }
			
}

void timeafterset(void)
{
    futuretimedays=((schedule_data[5]*60)+(schedule_data[6]))+5;
	if(futuretimedays>=1440)
			futuretimedays=futuretimedays-1440;
	futuretimehr=futuretimedays/60;
	futuretimemin=futuretimedays%60;
    futuretime1=((futuretimehr*100)+futuretimemin);
	//futuretime=futuretime1;
	offtimex=futuretime1;
}

void allvalveoff(void)
{
	// all turned on valves only turn off
	slaveerrornumber=slavenumber;
	//while(groupincwrite==1||memsave==1||groupskipwrite==1||writevaluestate==1||cyclestore==1||onttimewrite==1);
	for(byte=0;byte<15;byte++)
	{
 //if((slavenumber&(1<<byte)))
	   if(((slavenumber&(1<<byte))&&(slavenoresponse==0))||((slaveerrornumber&(1<<byte))&&(slavenoresponse==1)))
       {
				slaveaddress=byte+1;
				mbdata=0x00;
				modbusCommunication(WRITE); 
	   }
//slavedata[byte]=0x00;				
	}
}

void valveon(void)
{
	//giving valves only turned on not turn on for all valves 
	//while(groupincwrite==1||memsave==1||groupskipwrite==1||writevaluestate==1||cyclestore==1||onttimewrite==1);
    for(byte=0;byte<15;byte++)
    {
         if(((slavenumber&(1<<byte))&&(slavenoresponse==0))||((slaveerrornumber&(1<<byte))&&(slavenoresponse==1)))
         {
		    slaveaddress=byte+1;
            mbdata=slavedata[byte];
		    modbusCommunication(WRITE); 
	 	}
    }
}

void slavecheck(void)
{
	// this loop only perform slave error, if any slave error happens, itb writes 
	// 15 time even the device didn't get any response from the server then only it put the 
	// slave time out error 
	slavenumber=0;
    for(byte=0;byte<15;byte++)
    {
        if(slavedata[byte]>0)
        {
          slavenumber|=(1<<byte);
        }
    }
	for(byte=0;byte<15;byte++)
	{
		  slavedata[byte]=0;
	}
}

void addvalveinslave(void)
{
	// this function decides which valves will turn on 
    if(mode1!=1)
           valve_number=valve_number+1;
	
    divval=(((valve_number)-1)/10);		
    modval=10;
	if(valve_number%10!=0)
		   modval=valve_number%10;
    slavenumber|=(1<<divval);
	slaveerrornumber=slavenumber;
    slavedata[divval]|=(1<<(modval-1));	
}

void valvedelete(void)
{
	// when pump on we manually turn off all slaves 
    	for(i=0; i<maxvalves; i++)
        {
            byte_index = i/8;
            bit_index  = i%8;
            valve_state[byte_index] &= ~(1 << bit_index);
        }		
 }

void statusread(void)
{
	//every time device turn on after power cut that time this function will execute, to retain previous flags and data 
	//pumponflag=(STAT1&(1<<0));
	lightonflag=((STAT1>>1)&1);
	//STAT1 |= (0 & 0x01) << 2;
	//STAT1 |= (manualreset & 0x01)<< 3;
	phr=((STAT1>>4)&1);
	mqpofr=((STAT1>>5)&1);
	twophase=((STAT1>>6)&1);
	prog1=((STAT1>>7)&1);

	prog2=(STAT2&(1<<0));	
	prog3=((STAT2>>1)&1);
	prog4=((STAT2>>2)&1);
	mode1=((STAT2>>3)&1);
	mode2=((STAT2>>4)&1);
	mode3=((STAT2)>>5&1);
	mode4=((STAT2>>6)&1);
    if(mode1==0&&mode2==0&&mode3==0&&mode4==0)
		    mode3=1;
	modechecker1|=(mode1<<0);
	modechecker1|=(mode2<<1);
	modechecker1|=(mode3<<2);
	modechecker1|=(mode4<<3);
	modechecker2=modechecker1;
	if(groupskip==1&&mode4==1)
	{
			skipstart=1;
	}
}


void lightonfunction(void)
{
	//light on function 
	if(lightonflag==1&&dashboardsignal[0]==3)
	{
		if(crmrly==0)
		{
		    crmrly=1;
			update_staus();
		}
	}

	if(lightonflag==0&&dashboardsignal[0]==3)
	{
		if(crmrly==1)
		{
			crmrly=0;
			update_staus();
		}
	}
}


void poweronfaultread(void)
{
	// this loop execute when pump on after power cut and power on fault reset is enabled to retain the previous over load or dry error
	// retain means if pump turned off before due to ovl or dry and user didn't reset and device also turned off due to EB cut 
	if(mqpofr==1&&dryrestart==1)
	{
		dry=1;
	}
	if(mqpofr==1)
	{
		MQTT_POFR=1;
		Read_APROM_BYTE(61040);
		ovld=rdata;
		if(ovld>1)
		{
			ovld=0;
		}
		Read_APROM_BYTE(61041);
		errorvalueh=rdata;
		Read_APROM_BYTE(61042);
		errorvaluel=rdata;
		errorvalue1=((errorvalueh<<8)|errorvaluel);
		if(errorvalue1>1000){
			errorvalue1=0;
		}
    }
	
	if((dryrestart==1 && mqpofr==1&&mode1!=1)||(mode1==1&&dry==1&&mqpofr==1))
	{
	 if(mode1==1)
	 {
		 dryrestart=0;	
	 }
	 Read_APROM_BYTE(61060);           
     run3h=rdata;
	 Read_APROM_BYTE(61061);
	 run3l=rdata;
	 drt1=((run3h<<8)|run3l);
	 if(drt1>=1000)
     {
	  	drt1=seting_group[8];
     }
	 dry=1;
	 restart=1;
	 errordisplay=1;
	 displayx=1;
	 run3=drt1;
		
	 Read_APROM_BYTE(61062);
     dryreadh=rdata;
     Read_APROM_BYTE(61063);
     dryreadl=rdata;
     errorvalue1=((dryreadh<<8)|dryreadl);
     if(errorvalue1>999)
    	   errorvalue1=0;
	 }
	 if(ovld==1 && mqpofr==1)
	 {
		restart=1;
		errordisplay=1;
		refresh=1;
	 }
	 if(mqpofr==0)
	 {
		errorvalue1=0;
		if(dryrestart==1||dry==1)
		{
			dry=0;
			dryrestart=0;
			drterror=1;
			drywrite=1;
		}
		if(ovld==1)
		{
			ovld=0;	
			pofrflag=1;
		}
	}
}

void slaveerror_check(void)
{
	//if slave error happens this function ensure that error is there or not 
	errcheckroutine=0;
	
	for(errcheckroutine=0;errcheckroutine<10;errcheckroutine++)
	{
		if(slaveerrornumber!=0)
		{
		  if(turnoffv==0)
		  {
			valveon();
		  }
		  if(turnoffv==1)
		  {
			allvalveoff();
		  }
	    }
	  	if(slaveerrornumber==0)
			 errcheckroutine=11;
    }
				
	slavestat=slaveerrornumber;
	if(slaveerrornumber==0&&slavenoresponse==1)
		slavenoresponse=0;
	if(slavenoresponse==1&&slaveerrornumber!=0)
	{
		slavestat=slaveerrornumber;
		slaveerror=1;
		update_staus();
		restart=1;
		onlyonceFlag=1;
		if(pon==1)
			relayoff=1;
//if(turnonv==1)
	//relayoff=1;
//if(turnoffv==1)
	//turnoffv=0;
	}
}

