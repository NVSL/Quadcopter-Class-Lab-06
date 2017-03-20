  
/*
 * Welcome to MultiWii.
 *
 * If you see this message, chances are you are using the Arduino IDE. That is ok.
 * To get the MultiWii program configured for your copter, you must switch to the tab named 'config.h'.
 * Maybe that tab is not visible in the list at the top, then you must use the drop down list at the right
 * to access that tab. In that tab you must enable your baord or sensors and optionally various features.
 * For more info go to http://www.multiwii.com/wiki/index.php?title=Main_Page
 *
 * Have fun, and do not forget MultiWii is made possible and brought to you under the GPL License.
 *
 */

/* Notes for refactoring microWii 
 *  
 *  Break down key pieces into clean libraries that can be used on both the controller and the quadcopter:
 *  
 *  1.  Radio communication.
 *  2.  Serial communication
 *  3.  IMU reading
 *  4.  IMU processing: Smoothing + angle calculations.
 *  5.  Motor control
 *  6.  PID
 *  7.  MSP processing. 
 *  
 *  Test as we go.  Make sure we don't break anything.  How to test???
 */
