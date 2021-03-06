#include <avr/eeprom.h>
#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "EEPROM.h"
#include "MultiWii.h"
#include "Alarms.h"
#include "Serial.h"

void LoadDefaults(void);

uint8_t calculate_sum(uint8_t *cb , uint8_t siz) {
  uint8_t sum=0x55;  // checksum init
  while(--siz) sum += *cb++;  // calculate checksum (without checksum byte)
  return sum;
}

void readGlobalSet() {
  eeprom_read_block((void*)&global_conf, (void*)0, sizeof(global_conf));
  if(calculate_sum((uint8_t*)&global_conf, sizeof(global_conf)) != global_conf.checksum) {
    global_conf.currentSet = 0;
    global_conf.accZero[ROLL] = 5000;    // for config error signalization
  }
}
 
bool readEEPROM() {
  uint8_t i;
  global_conf.currentSet=0;
  eeprom_read_block((void*)&conf, (void*)(global_conf.currentSet * sizeof(conf) + sizeof(global_conf)), sizeof(conf));
  print("Calcul Eprom = %d, Read conf = %d \n\r", calculate_sum((uint8_t*)&conf, sizeof(conf)), conf.checksum);
  if(calculate_sum((uint8_t*)&conf, sizeof(conf)) != conf.checksum) {
	print("Loading Defaults\n\r");
    blinkLED(6,100,3);
    LoadDefaults();                 // force load defaults
    return false;                   // defaults loaded, don't reload constants (EEPROM life saving)
  }
  // 500/128 = 3.90625    3.9062 * 3.9062 = 15.259   1526*100/128 = 1192
  for(i=0;i<5;i++) {
    lookupPitchRollRC[i] = (1526+conf.rcExpo8*(i*i-15))*i*(int32_t)conf.rcRate8/1192;
  }
  for(i=0;i<11;i++) {
    int16_t tmp = 10*i-conf.thrMid8;
    uint8_t y = 1;
    if (tmp>0) y = 100-conf.thrMid8;
    if (tmp<0) y = conf.thrMid8;
    lookupThrottleRC[i] = 10*conf.thrMid8 + tmp*( 100-conf.thrExpo8+(int32_t)conf.thrExpo8*(tmp*tmp)/(y*y) )/10; // [0;1000]
    lookupThrottleRC[i] = conf.minthrottle + (int32_t)(MAXTHROTTLE-conf.minthrottle)* lookupThrottleRC[i]/1000;  // [0;1000] -> [conf.minthrottle;MAXTHROTTLE]
  }
  return true;    // setting is OK
}

void writeGlobalSet(uint8_t b) {
  global_conf.checksum = calculate_sum((uint8_t*)&global_conf, sizeof(global_conf));
  eeprom_write_block((const void*)&global_conf, (void*)0, sizeof(global_conf));
  if (b == 1) blinkLED(15,20,1);
}
 
void writeParams(uint8_t b) {
  global_conf.currentSet=0;
  conf.checksum = calculate_sum((uint8_t*)&conf, sizeof(conf));
  eeprom_write_block((const void*)&conf, (void*)(global_conf.currentSet * sizeof(conf) + sizeof(global_conf)), sizeof(conf));
  readEEPROM();
  if (b == 1) blinkLED(15,20,1);
}

void update_constants() { 
  #if defined (FAILSAFE)
    conf.failsafe_throttle = FAILSAFE_THROTTLE;
  #endif
  conf.minthrottle = MINTHROTTLE;
  #if MAG
    conf.mag_declination = (int16_t)(MAG_DECLINATION * 10);
  #endif
  writeParams(0); // this will also (p)reset checkNewConf with the current version number again.
}

void LoadDefaults() {
  uint8_t i;
  // PID CONTROLLER 1
	conf.pid[ROLL].P8     = 06;  conf.pid[ROLL].I8    = 05; conf.pid[ROLL].D8     = 04;
	conf.pid[PITCH].P8    = 06; conf.pid[PITCH].I8    = 05; conf.pid[PITCH].D8    = 04;
	conf.pid[PIDLEVEL].P8 = 02; conf.pid[PIDLEVEL].I8 = 02; conf.pid[PIDLEVEL].D8 = 02;

    conf.pid[YAW].P8      = 10;  conf.pid[YAW].I8     = 05;  conf.pid[YAW].D8     = 0;
    conf.pid[PIDALT].P8   = 64; conf.pid[PIDALT].I8   = 25; conf.pid[PIDALT].D8   = 24;

    conf.pid[PIDPOS].P8  = POSHOLD_P * 100;     conf.pid[PIDPOS].I8    = POSHOLD_I * 100;       conf.pid[PIDPOS].D8    = 0;
    conf.pid[PIDPOSR].P8 = POSHOLD_RATE_P * 10; conf.pid[PIDPOSR].I8   = POSHOLD_RATE_I * 100;  conf.pid[PIDPOSR].D8   = POSHOLD_RATE_D * 1000;
    conf.pid[PIDNAVR].P8 = NAV_P * 10;          conf.pid[PIDNAVR].I8   = NAV_I * 100;           conf.pid[PIDNAVR].D8   = NAV_D * 1000;
  
    conf.pid[PIDMAG].P8   = 40;

    conf.pid[PIDVEL].P8 = 0;      conf.pid[PIDVEL].I8 = 0;    conf.pid[PIDVEL].D8 = 0;

    conf.rcRate8 = 80; conf.rcExpo8 = 50;
	conf.rollPitchRate = 40;
	conf.yawRate = 40;
	conf.dynThrPID = 0;
	conf.thrMid8 = 50; conf.thrExpo8 = 0;
	for(i=0;i<CHECKBOXITEMS;i++) {conf.activate[i] = 0;}
	conf.angleTrim[0] = 0; conf.angleTrim[1] = 0;
	conf.powerTrigger1 = 0;

  update_constants(); // this will also write to eeprom
}
