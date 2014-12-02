
#ifndef configuration_H

    #define configuration_H

    #define F_ISR 1000      // FIXME the Arduino used in this lab is the 3.3 VDC 8 MHz model - no time to fix routines
                            // the actual ISR runs at half of this speed 

    #define F_CLK 16000000UL
    #define BAUD_RATE 57600L

    #define LINE_TERMINATOR 0x0A    // ASCII Line Feed

    #define BUF_LEN 256

    #define max_fields 20           // must be less than 127 ??? FIXME explain why

    #define MOT_DIR_PIN 4

    #define ISR_ACTIVE_PIN 9

    static int32_t R = 0;
    static float KP = 5.0;
    static float KI = 0.5;
    static float KD = 0.0;


#endif
