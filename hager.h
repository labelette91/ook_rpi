#ifndef _TYPE
#define _TYPE

#include "types.h"

#define CMD_ECO    0 
#define CMD_CONFOR 1 
#define CMD_HGEL   2 
#define CMD_ARRET  3 
#define CMD_CONFIG 4 

extern void HagerSends(byte id4 , byte cmnd) ;
extern void HagerSetPin (byte tx, byte pled);

extern void ManageHager(byte unitcode, byte id4, byte cmnd);
#endif