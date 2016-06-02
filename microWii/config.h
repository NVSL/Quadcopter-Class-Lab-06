#ifndef CONFIG_H_
#define CONFIG_H_

/*************************************************************************************************/
/****           CONFIGURABLE PARAMETERS                                                       ****/
/*************************************************************************************************/

/* this file consists of several sections
 * to create a working combination you must at least make your choices in section 1.
 * 1 - BASIC SETUP - you must select an option in every block.
 *      this assumes you have 4 channels connected to your board with standard ESCs and servos.
 * 2 - COPTER TYPE SPECIFIC OPTIONS - you likely want to check for options for your copter type
 * 3 - RC SYSTEM SETUP
 * 4 - ALTERNATE CPUs & BOARDS - if you have
 * 5 - ALTERNATE SETUP - select alternate RX (SBUS, PPM, etc.), alternate ESC-range, etc. here
 * 6 - OPTIONAL FEATURES - enable nice to have features here (FlightModes, LCD, telemetry, battery monitor etc.)
 * 7 - TUNING & DEVELOPER - if you know what you are doing; you have been warned
 *     - (ESCs calibration, Dynamic Motor/Prop Balancing, Diagnostics,Memory savings.....)
 * 8 - DEPRECATED - these features will be removed in some future release
 */

/* Notes:
 * 1. parameters marked with (*) in the comment are stored in eeprom and can be changed via serial monitor or LCD.
 * 2. parameters marked with (**) in the comment are stored in eeprom and can be changed via the GUI
 */


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  1 - BASIC SETUP                                                *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /**************************    The type of multicopter    ****************************/
    //#define GIMBAL
    //#define BI
    //#define TRI
    // #define QUADP
    #define QUADX
    //#define Y4
    //#define Y6
    //#define HEX6
    //#define HEX6X
    //#define HEX6H  // New Model
    //#define OCTOX8
    //#define OCTOFLATP
    //#define OCTOFLATX
    //#define FLYING_WING
    //#define VTAIL4
    //#define AIRPLANE
    //#define SINGLECOPTER
    //#define DUALCOPTER
    //#define HELI_120_CCPM
    //#define HELI_90_DEG

  /****************************    Motor minthrottle    *******************************/
    /* Set the minimum throttle command sent to the ESC (Electronic Speed Controller)
       This is the minimum value that allow motors to run at a idle speed  */
    //#define MINTHROTTLE 1300 // for Turnigy Plush ESCs 10A
    //#define MINTHROTTLE 1120 // for Super Simple ESCs 10A
    //#define MINTHROTTLE 1064 // special ESC (simonk)
    #define MINTHROTTLE 1050 // for brushed ESCs like ladybird
    //#define MINTHROTTLE 0 // (*) (**)

  /****************************    Motor maxthrottle    *******************************/
    /* this is the maximum value for the ESCs at full power, this value can be increased up to 2000 */
    #define MAXTHROTTLE 2000

  /****************************    Mincommand          *******************************/
    /* this is the value for the ESCs when they are not armed
       in some cases, this value must be lowered down to 900 for some specific ESCs, otherwise they failed to initiate */
    #define MINCOMMAND  1000

  /**********************************    I2C speed   ************************************/
    //#define I2C_SPEED 100000L     //100kHz normal mode, this value must be used for a genuine WMP
    #define I2C_SPEED 400000L   //400kHz fast mode, it works only with some WMP clones

  /***************************    Internal i2c Pullups   ********************************/
    /* enable internal I2C pull ups (in most cases it is better to use external pullups) */
    #define INTERNAL_I2C_PULLUPS

  /**************************************************************************************/
  /*****************          boards and sensor definitions            ******************/
  /**************************************************************************************/

      
    /***************************    independent sensors    ********************************/
      /* leave it commented if you already checked a specific board above */
      /* I2C gyroscope */
       #define LSM9DS0_GYRO
      
      /* I2C accelerometer */
       #define LSM9DS0_ACC

      /* I2C barometer */
      //#define BMP085
      //#define MS561101BA

      /* I2C magnetometer */
      //#define LSM9DS0_MAG

      /* Sonar */ // for visualization purpose currently - no control code behind
      //#define SRF02 // use the Devantech SRF i2c sensors
      //#define SRF08
      //#define SRF10
      //#define SRF23

      /* enforce your individual sensor orientation - even overrides board specific defaults */
      #define FORCE_ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  = -X; imu.accADC[PITCH]  = -Y; imu.accADC[YAW]  = Z;}
      #define FORCE_GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[PITCH] = -X; imu.gyroADC[ROLL] = Y; imu.gyroADC[YAW] = -Z;}
      #define FORCE_MAG_ORIENTATION(X, Y, Z)  {imu.magADC[ROLL]  =  -X; imu.magADC[PITCH]  =  -Y; imu.magADC[YAW]  = Z;}


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  2 - COPTER TYPE SPECIFIC OPTIONS                               *******/
/*****************                                                                 ***************/
/*************************************************************************************************/
  /********************************  PID Controller *********************************/
    /* choose one of the alternate PID control algorithms
     * 1 = evolved oldschool algorithm (similar to v2.2)
     * 2 = new experimental algorithm from Alex Khoroshko - unsupported - http://www.multiwii.com/forum/viewtopic.php?f=8&t=3671&start=10#p37387
     * */
    #define PID_CONTROLLER 1

    /* NEW: not used anymore for servo coptertypes  <== NEEDS FIXING - MOVE TO WIKI */
    #define YAW_DIRECTION 1
    //#define YAW_DIRECTION -1 // if you want to reverse the yaw correction direction

    #define ONLYARMWHENFLAT //prevent the copter from arming when the copter is tilted

   /********************************    ARM/DISARM    *********************************/
   /* optionally disable stick combinations to arm/disarm the motors.
     * In most cases one of the two options to arm/disarm via TX stick is sufficient */
    #define ALLOW_ARM_DISARM_VIA_TX_YAW
    //#define ALLOW_ARM_DISARM_VIA_TX_ROLL



/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  3 - RC SYSTEM SETUP                                            *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

	/**********************    ATmega128RF receiver    *******************************/
	/* This receiver is only for the ATmega128RFxx family which have internally a 2.4Ghz receiver*/
	#define ATMEGA128RF
	#define ATMEGA128RF_CHANNEL	21	//This channel should be only between 11 and 26


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  4 - ALTERNATE CPUs & BOARDS                                    *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /**************************************************************************************/
  /********                      override default pin assignments    ********************/
  /**************************************************************************************/

  /* only enable any of this if you must change the default pin assignment, e.g. your board does not have a specific pin */
  /* you may need to change PINx and PORTx plus #shift according to the desired pin! */
  #define OVERRIDE_LEDPIN_PINMODE             pinMode (34, OUTPUT); // use A1 instead of d13
  #define OVERRIDE_LEDPIN_TOGGLE              PINB |= 1<<6;     //switch LEDPIN state (digital PIN 13)
  #define OVERRIDE_LEDPIN_OFF                 PORTB &= ~(1<<6);
  #define OVERRIDE_LEDPIN_ON                  PORTB |= (1<<6);


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  5 - ALTERNATE SETUP                                            *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /******                Serial com speed    *********************************/
    /* This is the speed of the serial interfaces */
    #define SERIAL0_COM_SPEED 115200
    #define SERIAL1_COM_SPEED 115200
    #define SERIAL2_COM_SPEED 115200
    #define SERIAL3_COM_SPEED 115200

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  6 - OPTIONAL FEATURES                                          *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /************************        Angele throttle correction         ********************/
  /* Automatically increase throttle based on the angle of the copter
     Original idea by Kraut Rob, first implementation HAdrian							*/

  #define THROTTLE_ANGLE_CORRECTION 40


  /********                          Failsafe settings                 ********************/
    /* Failsafe check pulses on four main control channels CH1-CH4. If the pulse is missing or bellow 985us (on any of these four channels) 
       the failsafe procedure is initiated. After FAILSAFE_DELAY time from failsafe detection, the level mode is on (if ACC or nunchuk is avaliable),
       PITCH, ROLL and YAW is centered and THROTTLE is set to FAILSAFE_THROTTLE value. You must set this value to descending about 1m/s or so
       for best results. This value is depended from your configuration, AUW and some other params.  Next, after FAILSAFE_OFF_DELAY the copter is disarmed, 
       and motors is stopped. If RC pulse coming back before reached FAILSAFE_OFF_DELAY time, after the small quard time the RC control is returned to normal. */
    #define FAILSAFE                                // uncomment  to activate the failsafe function
    #define FAILSAFE_DELAY     10                    // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example
    #define FAILSAFE_OFF_DELAY 30                    // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 3sec in example
    #define FAILSAFE_THROTTLE  (MINTHROTTLE + 100)    // (*) Throttle level used for landing - may be relative to MINTHROTTLE - as in this case
    #define FAILSAFE_DETECT_TRESHOLD  985

  /**************************************************************************************/
  /***********************                  TX-related         **************************/
  /**************************************************************************************/

    /* introduce a deadband around the stick center
       Must be greater than zero, comment if you dont want a deadband on roll, pitch and yaw */
    #define DEADBAND 10
    
    /* Get your magnetic declination from here : http://magnetic-declination.com/
       Convert the degree+minutes into decimal degree by ==> degree+minutes*(1/60)
       Note the sign on declination it could be negative or positive (WEST or EAST) */
    //#define MAG_DECLINATION  3.96f              //For Budapest Hungary.
    #define MAG_DECLINATION  0.0f   //(**)

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  7 - TUNING & DEVELOPER                                  **************/
/*****************                                                                 ***************/
/*************************************************************************************************/

    /* some radios have not a neutral point centered on 1500. can be changed here */
    #define MIDRC 1500

/*************************************************************************************************/
/****           END OF CONFIGURABLE PARAMETERS                                                ****/
/*************************************************************************************************/

#endif /* CONFIG_H_ */

