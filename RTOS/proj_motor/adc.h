/**
 * @file      ADC.h
 * @brief     Provide functions that initialize ADC0
 * @details   Runs on LM4F120/TM4C123.
 * Provide functions that initialize ADC0. 
 * SS0 triggered by Timer0, and calls a user function.
 * SS3 triggered by software and trigger a conversion, wait for it to finish, and return the result.
 * @version   V1.0
 * @author    Valvano
 * @copyright Copyright 2017 by Jonathan W. Valvano, valvano@mail.utexas.edu,
 * @warning   AS-IS
 * @note      For more information see  http://users.ece.utexas.edu/~valvano/
 * @date      March 9, 2017

 ******************************************************************************/


/**
 * @details  This initialization function sets up the ADC 
 * Any parameters not explicitly listed below are not modified.
 * @note Max sample rate: <=125,000 samples/second<br>
 Sequencer 0 priority: 1st (highest)<br>
 Sequencer 1 priority: 2nd<br>
 Sequencer 2 priority: 3rd<br>
 Sequencer 3 priority: 4th (lowest)<br>
 SS3 triggering event: software trigger<br>
 SS3 1st sample source: programmable using variable 'channelNum' [0:11]<br>
 SS3 interrupts: enabled but not promoted to controller
 * @param  channelNum Channel from 0 to 11
 * @return none
 * @brief  Initialize ADC sequencer 3
 */
void ADC_Init(void);


/**
 * @details  Busy-wait Analog to digital conversion on SS3
 * @param  none
 * @return 12-bit result of ADC conversion
 * @brief  Sample ADC sequencer 3
 */
void ADC_In(unsigned long *data);

void ADC_Start(void);
