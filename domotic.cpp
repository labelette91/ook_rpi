#if _MSC_VER 
#include "debug\vspde.h"
#endif

//#include "arduino.h"
#include "types.h"
#include "domotic.h"

#include "oregon.h"
#include "DecodeHomeEasy.h"

#include "print.h"

extern "C" unsigned int millis();

tRBUF Cmd ;
unsigned long  lastTime;  
byte    receiveLength ;
byte    LastZoneValue[4];

byte    LastSensorValue[256] ;

byte    Seqnbr ;

//true if packet as been received and not treated
bool    DomoticPacketReceived;

typedef union tPAR {
byte Byte[4] ;
unsigned long Long;
}_tPAR;

unsigned long getLightingId (){
tPAR v  ;
v.Byte[0]=Cmd.LIGHTING2.id4 ;
v.Byte[1]=Cmd.LIGHTING2.id3 ;
v.Byte[2]=Cmd.LIGHTING2.id2 ;
v.Byte[3]=Cmd.LIGHTING2.id1 ;
return v.Long;
}

void resetLastSensorValue()
{
	memset(LastSensorValue,0,sizeof(LastSensorValue));
}
inline  void SetLastSensorValue(byte SensorId, byte sensorValue)
{
	LastSensorValue[SensorId]=sensorValue;
}
inline  byte GetLastSensorValue(byte SensorId)
{
	return LastSensorValue[SensorId];
}
inline  bool AsSensorValueChanged(byte SensorId, byte sensorValue)
{
	if ( LastSensorValue[SensorId]!=sensorValue)
	{
			LastSensorValue[SensorId]=sensorValue;
			return true;
	}
	return false;
}


void DomoticInit(){
lastTime =  0;
receiveLength = 0;
Cmd.LIGHTING2.packetlength  =0;
LastZoneValue[0];
LastZoneValue[1];
LastZoneValue[2];
LastZoneValue[3];
Seqnbr=0;
DomoticPacketReceived = false;

}
//reception d'une commande de send domotic
void DomoticReceive()
{
	//si paquet en cours de traitement 
	if (DomoticPacketReceived )
		return;

	byte nbCar = Serial.available();
	while (nbCar != 0)
	{
		{
			byte b = Serial.Read();
			if ((millis() - lastTime) > 10000) {
				receiveLength = 0;
			}
			lastTime = millis();
			if (receiveLength >= sizeof(_tRBUF)) {
				receiveLength = 0;
			}
			Cmd.Buffer[receiveLength++] = b;

			//if packet len not valid , wait valid
			if ((Cmd.LIGHTING2.packetlength != 13)
				&& (Cmd.LIGHTING2.packetlength != 11)
				)
				receiveLength = 0;

			/* si paquet recu en entier */
			if (receiveLength >= Cmd.LIGHTING2.packetlength + 1) {
				DomoticPacketReceived = true;
				receiveLength = 0;
			}
			else
				DomoticPacketReceived = false;

			nbCar--;

		}
	}
}    

tRBUF Send ;

//envoi temperature : humudit a domotic
//, byte bateryLevel : 0..15 0 = batteri low
void reportDomoticTemp ( int temp , byte id1 , byte id2 ,  byte bateryLevel){

		Send.Temp.packetlength = 8;
		Send.Temp.packettype   = 0x50;
		Send.Temp.subtype      = 05  ;
		Send.Temp.seqnbr       = Seqnbr++;
		Send.Temp.id1          = id1 ;
		Send.Temp.id2          = id2 ;
		
		if (temp>=0)
			Send.Temp.tempsign     = 0   ;
		else {
			Send.Temp.tempsign = 1;
			temp = -temp;
		}
		Send.Temp.temperatureh = temp >> 8;
		Send.Temp.temperaturel   = temp & 0xff;
		Send.Temp.battery_level  = bateryLevel ;
		Send.Temp.rssi           = 9 ;
            
    Serial.Write((byte*)&Send.Temp,9);
}
void reportDomoticTemp ( const byte* data){
	reportDomoticTemp ( temperatureint(data) , 0x48  , 0x00 , battery(data) );
	
}

//temp = temperature * 10 
void reportDomoticTempHum ( int temp , byte hum , byte id1 , byte id2, byte bateryLevel){
	
	
	//test if as changed
//	if (AsSensorValueChanged(temp,id1))
	{
  Send.Temp_Hum.packetlength = sizeof(Send.Temp_Hum)-1;
  Send.Temp_Hum.packettype   = 0x52;   //pTypeTEMP_HUM
  Send.Temp_Hum.subtype      = 01  ;   //sTypeTH1 0x1  //THGN122/123,THGN132,THGR122/228/238/268
  Send.Temp_Hum.seqnbr       = Seqnbr++;
  Send.Temp_Hum.id1          = id1;
  Send.Temp_Hum.id2          = id2;
  
  if (temp>=0)
  	Send.Temp_Hum.tempsign     = 0   ;
  else {
	Send.Temp_Hum.tempsign = 1;
	temp = -temp;
  }

  Send.Temp_Hum.temperatureh = temp >> 8;
  Send.Temp_Hum.temperaturel   = temp & 0xff;
  Send.Temp_Hum.battery_level  = bateryLevel ;
  Send.Temp_Hum.rssi           = 9 ;
  Send.Temp_Hum.humidity       = hum ;
  Send.Temp_Hum.humidity_status= 0  ;
  
  
  Serial.Write((byte*)&Send.Temp_Hum,sizeof(Send.Temp_Hum));
	}
              
}
void reportDomoticPower (  byte* data, int size ){
int  tint  ;
unsigned long  tlong ;

tint = getPower(data ) ;
tlong= getTotalPower(data);

  
  Send.ENERGY.packetlength = sizeof(Send.ENERGY)-1;
  Send.ENERGY.packettype   = pTypeENERGY; 
  Send.ENERGY.subtype      = sTypeELEC2 ; //0x1   //CM119/160 
  Send.ENERGY.seqnbr       = Seqnbr++;
  Send.ENERGY.id1          = data[0];
  Send.ENERGY.id2          = data[1];
  
 	Send.ENERGY.count=1;     
	Send.ENERGY.instant1=0;
	Send.ENERGY.instant2=0;
	Send.ENERGY.instant3=(tint>>8) & 0xFF;
	Send.ENERGY.instant4=(tint>>0) & 0xff;
	Send.ENERGY.total1=0;  
	Send.ENERGY.total2=0;  
	Send.ENERGY.total3=(tlong>>24) & 0xff;
	Send.ENERGY.total4=(tlong>>16) & 0xff;
	Send.ENERGY.total5=(tlong>> 8) & 0xff;
	Send.ENERGY.total6=(tlong>> 0) & 0xff;
  
  Serial.Write((byte*)&Send.ENERGY,sizeof(Send.ENERGY));
              
}


void reportDomotic ( const byte* data,byte size ){

	//id1 = sensor id id2 = channel pour oregon
  if (getSensorByte1(data)==CMR180_ID1)
    reportDomoticPower ( (byte*)data, size );
  else
    //homeEasy sensor
  if (getSensorByte1(data)==HOMESWITCH_ID0)
    reportDomoticHomeEasy( (byte*)data, size );
  else
    reportDomoticTempHum (temperatureint(data) , getHumidity(data) , getId(data),channel(data), battery(data));
	
}

void DomoticStartReceive()
{
  Send.IRESPONSE.packetlength = sizeof(Send.IRESPONSE)-1;
  Send.IRESPONSE.packettype = pTypeInterfaceMessage;
  Send.IRESPONSE.subtype = cmdStartRec ;
  Send.IRESPONSE.msg1 ='C';
  Send.IRESPONSE.msg2 ='o';
  Send.IRESPONSE.msg3 ='p';
  Send.IRESPONSE.msg4 ='y';
  Send.IRESPONSE.msg5 ='r';
  Send.IRESPONSE.msg6 ='i';
  Send.IRESPONSE.msg7 ='g';
  Send.IRESPONSE.msg8 ='h';
  Send.IRESPONSE.msg9 ='t';
  Send.IRESPONSE.msg10=' ';
  Send.IRESPONSE.msg11='R';
  Send.IRESPONSE.msg12='F';
  Send.IRESPONSE.msg13='X';
  Send.IRESPONSE.msg14='C';
  Send.IRESPONSE.msg15='O';
  Send.IRESPONSE.msg16='M';
  Serial.Write((byte*)&Send.IRESPONSE,sizeof(Send.IRESPONSE));

}
void DomoticStatus()
{
  Send.IRESPONSE.packetlength = sizeof(Send.IRESPONSE)-1;
  Send.IRESPONSE.packettype = pTypeInterfaceMessage;
  Send.IRESPONSE.subtype = sTypeInterfaceResponse ;
  Send.IRESPONSE.cmnd    = cmdSTATUS ;
  Send.IRESPONSE.msg1    = trxType43392 ;
  Send.IRESPONSE.msg2    = VERSION  ;    //Firmware version
  Send.IRESPONSE.msg3    = 0  ;
  Send.IRESPONSE.msg4    = 0  ;
  Send.IRESPONSE.msg5    = 0  ;
  Send.IRESPONSE.msg6    = 1  ;    //Hardware version
  Send.IRESPONSE.msg7    = 0  ;    //Hardware version
		
	
  Serial.Write((byte*)&Send.IRESPONSE,sizeof(Send.IRESPONSE));
	
}

void reportHagerDomoticUnk ( const byte* data, byte pos ){
  Send.UNDECODED.packetlength = pos+3;
  Send.UNDECODED.packettype = pTypeUndecoded;
  Send.UNDECODED.subtype = 0xff ;

  for (byte i = 0; i < pos; ++i) {
      Send.UNDECODED.msg[i]= data[i];
  }
  Serial.Write((byte*)&Send.UNDECODED,Send.UNDECODED.packetlength+1  );
}

#ifdef Hager

extern byte GetMode(const byte* data) ;
extern byte GetZone(const byte* data);
#define HORS_GEL  0x4 
#define CONFOR    0x0
#define ECO       0x3
#define ARRET     0x5

void reportHagerDomoticSerial ( const byte* data, byte pos ){
  

		Send.LIGHTING2.packetlength=11  ; 
		Send.LIGHTING2.packettype = pTypeLighting2;   
		Send.LIGHTING2.subtype    = sTypeANSLUT ;							//AC
		Send.LIGHTING2.seqnbr			= Seqnbr++ ;
		Send.LIGHTING2.id1        = data[0];            /* id emetteur 0..3  */
		Send.LIGHTING2.id2        = data[1];            /* id emetteur 0..FF */
		Send.LIGHTING2.id3        = data[2];            /* id emetteur 0..FF */
		Send.LIGHTING2.id4        = 0      ;            /* id emetteur 0..FF */
		Send.LIGHTING2.unitcode   = GetZone(data);  		/* unit = zone 1..3  */
		Send.LIGHTING2.level    =0 ;   /* dim level 0..15   */
		Send.LIGHTING2.rssi     = 0;
//devbug
/*
    Send.LIGHTING2.packetlength = 11 + 4; //+4 debug 
		Send.LIGHTING2.data[0]  = data[3];
		Send.LIGHTING2.data[1]  = data[4];
		Send.LIGHTING2.data[2]  = data[5];
		Send.LIGHTING2.data[3]  = data[6];
*/

		if ( GetMode(data)== CONFOR)
				Send.LIGHTING2.cmnd       = 1 ;         
		else
				Send.LIGHTING2.cmnd       = 0 ;         
  Serial.Write((byte*)&Send.LIGHTING2,Send.LIGHTING2.packetlength+1  );
}
void reportHagerDomotic ( const byte* data, byte pos ){
	byte zone =  GetZone(data);   
	byte cmnd ;	
	if ( GetMode(data)== CONFOR)
			cmnd       = 1 ;         
	else
			cmnd       = 0 ;    
			
	if (LastZoneValue[zone] != 	cmnd )
	{
		LastZoneValue[zone] = 	cmnd ;
		reportHagerDomoticSerial ( data,  pos );
	}	     


}

#endif

void reportDomoticHomeEasy ( const byte* data, byte pos ){
  

		Send.LIGHTING2.packetlength=11  ; 
		Send.LIGHTING2.packettype = pTypeLighting2;   
		Send.LIGHTING2.subtype    = sTypeHEU ;					
		Send.LIGHTING2.seqnbr			= Seqnbr++ ;
		Send.LIGHTING2.id1        = data[5];            /* id emetteur 0..3  */
		Send.LIGHTING2.id2        = data[6];            /* id emetteur 0..FF */
		Send.LIGHTING2.id3        = data[7];            /* id emetteur 0..FF */
		Send.LIGHTING2.id4        = data[7];            /* id emetteur 0..FF */
		Send.LIGHTING2.unitcode   = data[2];        		/* unit = zone 1..3  */
		Send.LIGHTING2.level    = 0 ;   /* dim level 0..15   */
		Send.LIGHTING2.rssi     = 0;
//devbug
/*
		Send.LIGHTING2.packetlength = 11 + 4; //+2 debug 
		Send.LIGHTING2.data[0]  = data[3];
		Send.LIGHTING2.data[1]  = data[4];
		Send.LIGHTING2.data[2]  = data[5];
		Send.LIGHTING2.data[3]  = data[6];
*/
  	Send.LIGHTING2.cmnd     = data[3] ;         

    Serial.Write((byte*)&Send.LIGHTING2,Send.LIGHTING2.packetlength+1  );
}

void reportDomoticMD230(const byte* data, byte pos) {

//  MD230:2E DB 0B 44 10   //ok
//	MD230 : 2E DB 0B 04 50    //alarm 1
//	MD230 : 2E DB 0B 14 40   //alarm 2
//	MD230 : 2E DB 0B 14 40
//	MD230 : 2E DB 0B 54 00

	Send.LIGHTING2.packetlength = 11 ; 
	Send.LIGHTING2.packettype = pTypeLighting2;
	Send.LIGHTING2.subtype = sTypeHEU;
	Send.LIGHTING2.seqnbr = Seqnbr++;
	Send.LIGHTING2.id1 = 0 ;                 /* id emetteur 0..3  */
	Send.LIGHTING2.id2 = data[0];            /* id emetteur 0..FF */
	Send.LIGHTING2.id3 = data[1];            /* id emetteur 0..FF */
	Send.LIGHTING2.id4 = data[2];            /* id emetteur 0..FF */
	Send.LIGHTING2.unitcode = 1;         	 /* unit = zone 1..3  */
	Send.LIGHTING2.level = 0;   /* dim level 0..15   */
	Send.LIGHTING2.rssi  = 0;
	
	if ( (data[3]==0x14 ) && (data[4] == 0x40) )
		Send.LIGHTING2.cmnd = 0;
	else
		Send.LIGHTING2.cmnd = 1;

	Serial.Write((byte*)&Send.LIGHTING2, Send.LIGHTING2.packetlength + 1);
}


