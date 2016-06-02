#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Alarms.h"
#include "EEPROM.h"
#include "IMU.h"

void waitTransmissionI2C();
void Device_Mag_getADC();
void Baro_init();
void Mag_init();
void ACC_init();

// ************************************************************************************************************
// board orientation and setup
// ************************************************************************************************************
//default board orientation
#if !defined(ACC_ORIENTATION) 
  #define ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  = X; imu.accADC[PITCH]  = Y; imu.accADC[YAW]  = Z;}
#endif
#if !defined(GYRO_ORIENTATION) 
  #define GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[PITCH] = -X; imu.gyroADC[ROLL] = -Y; imu.gyroADC[YAW] = Z;}
#endif
#if !defined(MAG_ORIENTATION) 
  #define MAG_ORIENTATION(X, Y, Z)  {imu.magADC[ROLL]  = X; imu.magADC[PITCH]  = Y; imu.magADC[YAW]  = Z;}
#endif

uint8_t rawADC[6];
static uint32_t neutralizeTime = 0;
  
// ************************************************************************************************************
// I2C general functions
// ************************************************************************************************************

void i2c_init(void) {
  #if defined(INTERNAL_I2C_PULLUPS)
    I2C_PULLUPS_ENABLE
  #else
    I2C_PULLUPS_DISABLE
  #endif
  TWSR = 0;                                    // no prescaler => prescaler = 1
  TWBR = ((F_CPU / I2C_SPEED) - 16) / 2;       // change the I2C clock rate
  TWCR = 1<<TWEN;                              // enable twi module, no interrupt
}

void i2c_rep_start(uint8_t address) {
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) ; // send REPEAT START condition
  waitTransmissionI2C();                       // wait until transmission completed
  TWDR = address;                              // send device address
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();                       // wait until transmission completed
}

void i2c_stop(void) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  //  while(TWCR & (1<<TWSTO));                // <- can produce a blocking state with some WMP clones
}

void i2c_write(uint8_t data ) {
  TWDR = data;                                 // send data to the previously addressed device
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();
}

uint8_t i2c_read(uint8_t ack) {
  TWCR = (1<<TWINT) | (1<<TWEN) | (ack? (1<<TWEA) : 0);
  waitTransmissionI2C();
  uint8_t r = TWDR;
  if (!ack) i2c_stop();
  return r;
}

uint8_t i2c_readAck() {
  return i2c_read(1);
}

uint8_t i2c_readNak(void) {
  return i2c_read(0);
}

void waitTransmissionI2C() {
  uint16_t count = 255;
  while (!(TWCR & (1<<TWINT))) {
    count--;
    if (count==0) {              //we are in a blocking state => we don't insist
      TWCR = 0;                  //and we force a reset on TWINT register
      neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay
      i2c_errors_count++;
      break;
    }
  }
}

size_t i2c_read_to_buf(uint8_t add, void *buf, size_t size) {
  i2c_rep_start((add<<1) | 1);  // I2C read direction
  size_t bytes_read = 0;
  uint8_t *b = (uint8_t*)buf;
  while (size--) {
    /* acknowledge all but the final byte */
    *b++ = i2c_read(size > 0);
    /* TODO catch I2C errors here and abort */
    bytes_read++;
  }
  return bytes_read;
}

size_t i2c_read_reg_to_buf(uint8_t add, uint8_t reg, void *buf, size_t size) {
  i2c_rep_start(add<<1); // I2C write direction
  i2c_write(reg);        // register selection
  return i2c_read_to_buf(add, buf, size);
}

/* transform a series of bytes from big endian to little
   endian and vice versa. */
void swap_endianness(void *buf, size_t size) {
  /* we swap in-place, so we only have to
  * place _one_ element on a temporary tray
  */
  uint8_t tray;
  uint8_t *from;
  uint8_t *to;
  /* keep swapping until the pointers have assed each other */
  for (from = (uint8_t*)buf, to = &from[size-1]; from < to; from++, to--) {
    tray = *from;
    *from = *to;
    *to = tray;
  }
}

void i2c_getSixRawADC(uint8_t add, uint8_t reg) {
  i2c_read_reg_to_buf(add, reg, &rawADC, 6);
}

void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val) {
  i2c_rep_start(add<<1); // I2C write direction
  i2c_write(reg);        // register selection
  i2c_write(val);        // value to write in register
  i2c_stop();
}

uint8_t i2c_readReg(uint8_t add, uint8_t reg) {
  uint8_t val;
  i2c_read_reg_to_buf(add, reg, &val, 1);
  return val;
}

// ****************
// GYRO common part
// ****************
void GYRO_Common() {
  static int16_t previousGyroADC[3] = {0,0,0};
  static int32_t g[3];
  uint8_t axis;

  if (calibratingG>0) {
    for (axis = 0; axis < 3; axis++) {
      // Reset g[axis] at start of calibration
      if (calibratingG == 512) {
        g[axis]=0;
      }
      // Sum up 512 readings
      g[axis] +=imu.gyroADC[axis];
      // Clear global variables for next reading
      imu.gyroADC[axis]=0;
      gyroZero[axis]=0;
      if (calibratingG == 1) {
        gyroZero[axis]=(g[axis]+256)>>9;

        blinkLED(10,15,1); //the delay causes to beep the buzzer really long 

      }
    }
    calibratingG--;
    
  }

  for (axis = 0; axis < 3; axis++) {
    imu.gyroADC[axis]  -= gyroZero[axis];
    //anti gyro glitch, limit the variation between two consecutive readings
    imu.gyroADC[axis] = constrain(imu.gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);

    previousGyroADC[axis] = imu.gyroADC[axis];
  }

}

// ****************
// ACC common part
// ****************
void ACC_Common() {
  static int32_t a[3];
  if (calibratingA>0) {
    for (uint8_t axis = 0; axis < 3; axis++) {
      // Reset a[axis] at start of calibration
      if (calibratingA == 512) a[axis]=0;
      // Sum up 512 readings
      a[axis] +=imu.accADC[axis];
      // Clear global variables for next reading
      imu.accADC[axis]=0;
      global_conf.accZero[axis]=0;
    }
    // Calculate average, shift Z down by ACC_1G and store values in EEPROM at end of calibration
    if (calibratingA == 1) {
      global_conf.accZero[ROLL]  = (a[ROLL]+256)>>9;
      global_conf.accZero[PITCH] = (a[PITCH]+256)>>9;
      global_conf.accZero[YAW]   = ((a[YAW]+256)>>9)-ACC_1G; // for nunchuk 200=1G
      conf.angleTrim[ROLL]   = 0;
      conf.angleTrim[PITCH]  = 0;
      writeGlobalSet(1); // write accZero in EEPROM
    }
    calibratingA--;
  }

  imu.accADC[ROLL]  -=  global_conf.accZero[ROLL] ;
  imu.accADC[PITCH] -=  global_conf.accZero[PITCH];
  imu.accADC[YAW]   -=  global_conf.accZero[YAW] ;

}


// ************************************************************************************************************
// LSM9DS0 I2C Accelerometer
// I2C adress: 0x3B (8bit)
// ************************************************************************************************************
#if defined(LSM9DS0_ACC)
#define LSM9DS0_ACC_ADR  (0x3B >> 1) // I2C accelerometer address -- this is if you connected SA0_XM and SA0_G pins to vdd (Adafruit LSM9DS0)
//#define LSM9DS0_ACC_ADR  (0x3D >> 1) // I2C accelerometer address  -- this is if you connected SA0_XM and SA0_G pins to ground
void ACC_init () {
  i2c_writeReg(LSM9DS0_ACC_ADR,0x20,0x57);   // 50Hz data rate, XYZ enable
  i2c_writeReg(LSM9DS0_ACC_ADR,0x21,0x00);   // Set scale to 2g
}

void ACC_getADC () {  
  TWBR = ((F_CPU / 100000L) - 16) / 2; //100 kHz clock
  i2c_getSixRawADC(LSM9DS0_ACC_ADR,0x28 | 0x80);

 ACC_ORIENTATION( ((rawADC[1]<<8) | rawADC[0])>>4 ,
                  ((rawADC[3]<<8) | rawADC[2])>>4 ,
                  ((rawADC[5]<<8) | rawADC[4])>>4 );

  ACC_Common();
}
#endif


// ************************************************************************************************************
// I2C Gyroscope LSM9DS0
// ************************************************************************************************************
#if defined(LSM9DS0_GYRO)
#define LSM9DS0_GYRO_ADR (0xD6 >> 1) // I2C gyro address -- this is if you connected SA0_XM and SA0_G pins to vdd (Adafruit LSM9DS0)
//#define LSM9DS0_GYRO_ADR (0xD5 >> 1) // I2C gyro address  -- this is if you connected SA0_XM and SA0_G pins to ground
void Gyro_init(){
  i2c_writeReg(LSM9DS0_GYRO_ADR, 0x20, 0x0F);   //Ctrl reg 1: 100Hz, normal power, XYZ enable
  i2c_writeReg(LSM9DS0_GYRO_ADR, 0x23, 0x30);   //2000 dps scale
}

void Gyro_getADC(){
  TWBR = ((F_CPU / 100000L) - 16) / 2;  //100 kHz clock
  i2c_getSixRawADC(LSM9DS0_GYRO_ADR, 0x28 | 0x80);
  GYRO_ORIENTATION( ((rawADC[1]<<8) | rawADC[0]) ,
                    ((rawADC[3]<<8) | rawADC[2]) ,
                    ((rawADC[5]<<8) | rawADC[4]) );
  GYRO_Common();
}

#endif


// ************************************************************************************************************
// I2C Compass common function
// ************************************************************************************************************
#if MAG
static float   magGain[3] = {1.0,1.0,1.0};  // gain for each axis, populated at sensor init
static uint8_t magInit = 0;

uint8_t Mag_getADC() { // return 1 when news values are available, 0 otherwise
  static uint32_t t,tCal = 0;
  static int16_t magZeroTempMin[3];
  static int16_t magZeroTempMax[3];
  uint8_t axis;
  if ( currentTime < t ) return 0; //each read is spaced by 100ms
  t = currentTime + 100000;
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  Device_Mag_getADC();
  imu.magADC[ROLL]  = imu.magADC[ROLL]  * magGain[ROLL];
  imu.magADC[PITCH] = imu.magADC[PITCH] * magGain[PITCH];
  imu.magADC[YAW]   = imu.magADC[YAW]   * magGain[YAW];
  if (f.CALIBRATE_MAG) {
    tCal = t;
    for(axis=0;axis<3;axis++) {
      global_conf.magZero[axis] = 0;
      magZeroTempMin[axis] = imu.magADC[axis];
      magZeroTempMax[axis] = imu.magADC[axis];
    }
    f.CALIBRATE_MAG = 0;
  }
  if (magInit) { // we apply offset only once mag calibration is done
    imu.magADC[ROLL]  -= global_conf.magZero[ROLL];
    imu.magADC[PITCH] -= global_conf.magZero[PITCH];
    imu.magADC[YAW]   -= global_conf.magZero[YAW];
  }
 
  if (tCal != 0) {
    if ((t - tCal) < 30000000) { // 30s: you have 30s to turn the multi in all directions
      LEDPIN_TOGGLE;
      for(axis=0;axis<3;axis++) {
        if (imu.magADC[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = imu.magADC[axis];
        if (imu.magADC[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = imu.magADC[axis];
      }
    } else {
      tCal = 0;
      for(axis=0;axis<3;axis++)
        global_conf.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis])>>1;
      writeGlobalSet(1);
    }
  } else {

  }
  return 1;
}
#endif

#if defined(LSM9DS0_MAG)
#define LSM9DS0_MAG_ADR (0x3B >> 1) // I2C magnetometer address -- this is if you connected SA0_XM and SA0_G pins to vdd (Adafruit LSM9DS0)
//#define LSM9DS0_MAG_ADR (0x3D >> 1)// I2C magnetometer address  -- this is if you connected SA0_XM and SA0_G pins to ground
  void Mag_init() {

    i2c_writeReg(LSM9DS0_MAG_ADR,0x24,0x08);  // [CTRL_REG5_XM] Mag data rate - 12.5 Hz
    i2c_writeReg(LSM9DS0_MAG_ADR,0x25,0x00);  // [CTRL_REG6_XM] Mag scale to +/- 2Ga
    i2c_writeReg(LSM9DS0_MAG_ADR,0x26,0x00);  // [CTRL_REG7_XM] Continous conversion mode
    i2c_writeReg(LSM9DS0_MAG_ADR,0x23,0x04);  // [CTRL_REG4_XM]
    i2c_writeReg(LSM9DS0_MAG_ADR,0x12,0x09);  // [INT_CTRL_REG_M]
    magInit = 1;
  }

  #if not defined(MPU6050_EN_I2C_BYPASS)
    void Device_Mag_getADC() {
      i2c_getSixRawADC(LSM9DS0_MAG_ADR,0x08 | 0x80);
      MAG_ORIENTATION( ((rawADC[1]<<8) | rawADC[0]) ,
                       ((rawADC[3]<<8) | rawADC[2]) ,
                       ((rawADC[5]<<8) | rawADC[4]) );
    }
  #endif
#endif


void initSensors() {
  delay(100);
  i2c_init();
  delay(100);
  #if GYRO
  Gyro_init();
  #endif
  if (BARO) Baro_init();
  if (MAG) Mag_init();
  if (ACC) ACC_init();
  f.I2C_INIT_DONE = 1;
}


