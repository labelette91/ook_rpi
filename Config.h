//type de report serie 
//si = define  : report serial forma domoticz (binaire)
//#define REPORT_TYPE  REPORT_DOMOTIC 
//#define REPORT_TYPE  ( REPORT_DOMOTIC | SERIAL_DEBUG )
#define REPORT_TYPE  ( SERIAL_DEBUG )

//si =  : report serial format text 
//#define REPORT_TYPE REPORT_SERIAL 
//#define REPORT_TYPE SERIAL_DEBUG 

#define RFM69_ENABLE
//#define  BMP180_ENABLE        


//#define OTIO_ENABLE 1
#define OOK_ENABLE  1
//#define HAGER_ENABLE 1
#define HOMEEASY_ENABLE 1
//#define MD230_ENABLE 1
#define RUBICSON_ENABLE 1
#define  HIDEKI_ENABLE        
#define  RAIN_ENABLE        

#define RASPBERRY_PI

//offset in micros for pulse duration for RFM69 : 80Micros
#define OFFSET_DURATION_HIGH 80
