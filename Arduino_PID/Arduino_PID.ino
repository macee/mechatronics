#include <LiquidCrystal.h>

/**
 * Mechatronics Lab 6
 *
 * Copyright 2014 Aaron P. Dahlen       APDahlen@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 */



/** @Warning - there is a bug somewhere between the ISR and the Arduino analogWrite
 * function.  If you need to use the PWM from within the ISR then use:
 *
 * http://arduino.cc/en/Tutorial/SecretsOfArduinoPWM
 * 
 *    void setup(){
 *        .
 *        .
 *        .
 *        pinMode(3, OUTPUT);
 *        pinMode(11, OUTPUT);
 *        TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
 *        TCCR2B = _BV(CS22);                                               // This register controls PWM frequency
 *        OCR2A = 0;                                                        // Arduino I/O pin # 11 duty cycle
 *        OCR2B = 0;                                                        // Arduino I/O pin # 3 duty cycle
 *        .
 *        .
 *        .
 *    }
 *
 *    void loop(){
 *        .
 *        .
 *        .
 *        OCR2A = pin_11_PWM_DC;
 *        OCR2B = pin_3_PWM_DC;
 *        .
 *        .
 *        .
 *    }
 */


// AVR GCC libraries for more information see:
//      http://www.nongnu.org/avr-libc/user-manual/modules.html
//      https://www.gnu.org/software/libc/manual/

    #include <avr/io.h>
    #include <avr/interrupt.h>
    #include <stdint.h>
    #include <string.h>
    #include <ctype.h>

// Arduino libraries: see http://arduino.cc/en/Reference/Libraries

    #include <LiquidCrystal.h>

// Project specific includes

    #include "configuration.h"
    #include "USART.h"
    #include "line_parser.h"
    #include "AVR_SPI.h"

// Global variables

    char char_buf[32];
    uint8_t field_start_indices[max_fields];

    volatile uint8_t start_flag = 0;

    volatile static uint8_t request_status;
    volatile static uint32_t ISR_velocity, ISR_position;

    volatile static int32_t ISR_E, ISR_P, ISR_I, ISR_D;










    void MD10C_H_bridge_driver(int32_t drive);





void setup(){

    AVR_SPI_master_init();
    init_timer_1_CTC(F_ISR);                            // Enable the timer based ISR

    USART_init(F_CLK, BAUD_RATE);
    USART_set_terminator(LINE_TERMINATOR);

    /* Use fast PWM using direct registers */
        pinMode(3, OUTPUT);
        TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
        TCCR2B = _BV(CS20);     //_BV(CS22);            // This register controls PWM frequency
        OCR2B = 0;

    pinMode(MOT_DIR_PIN, OUTPUT);

}



/***************************************************************************************************
 *  ______  ____   _____   ______  _____  _____    ____   _    _  _   _  _____
 * |  ____|/ __ \ |  __ \ |  ____|/ ____||  __ \  / __ \ | |  | || \ | ||  __ \
 * | |__  | |  | || |__) || |__  | |  __ | |__) || |  | || |  | ||  \| || |  | |
 * |  __| | |  | ||  _  / |  __| | | |_ ||  _  / | |  | || |  | || . ` || |  | |
 * | |    | |__| || | \ \ | |____| |__| || | \ \ | |__| || |__| || |\  || |__| |
 * |_|     \____/ |_|  \_\|______|\_____||_|  \_\ \____/  \____/ |_| \_||_____/
 *
 **************************************************************************************************/

ISR(USART_RX_vect){

 /**
 * @note This Interrupt Service Routine is called when a new character is received by the USART
 * Ideally it would have been placed in the USART.cpp file but this generates a error "multiple
 * definition of vector_18".  Apparently Arduino detects when an ISR is in the main sketch.  If you
 * place it somewhere else it is missed and replaced with the Arduino handler.  This is the source
 * of the multiple definitions error...
 * see discussion @ http://forum.arduino.cc/index.php?topic=42153.0
 */
    USART_handle_ISR();
}


ISR(TIMER1_COMPA_vect){

/** @brief This Interrupt Service Routine serves as the F_ISR Hz heartbeat for the Arduino.
 * See the companion function init_timer_1_CTC for additional information.
 *
 * @Note:
 *    1) Compiler generated code pushes status register and any used registers to stack.
 *    2) Calling a subroutine from the ISR causes compiler to save all 32 registers; a
 *       slow operation (fact check).
 *    3) Status and used registers are popped by compiler generated code.
 */

    static int32_t current_position, last_position;
    static int32_t current_velocity, last_velocity;
    static uint8_t first_pass = 1;

    static uint16_t time_in_state;
    static uint8_t setpoint_state;

    static float sum_error = 0;
    float E, P, I, D, PID;

    uint8_t tx_SPI_buf[5];
    uint8_t rx_SPI_buf[5];

    digitalWrite(ISR_ACTIVE_PIN, HIGH);                                // Signal ISR start.  Use an oscilloscope to verify cooperative time scheduling

    AVR_SPI_master_xfr(5, tx_SPI_buf, rx_SPI_buf);
    current_position = rx_SPI_buf[1];
    current_position = (current_position << 8) +  rx_SPI_buf[2];
    current_position = (current_position << 8) +  rx_SPI_buf[3];
    current_position = (current_position << 8) +  rx_SPI_buf[4];

    current_velocity = current_position - last_position;
    last_position = current_position;

    if (first_pass){                        // This if statement cost one hours of lab time!  Be sure
        first_pass = 0;                     // // to capture "last_positon" before you enter the PID
        goto end_of_ISR;                    // // routine.  If not the integrator jumps to an astronomically
    }                                       // // high value on the first pass through this ISR.

/**** Start of PID code ****/

    E = R - current_velocity;

    P = KP * E;




   // your code here





    PID = P + I + D;

    MD10C_H_bridge_driver((int32_t)PID);

    /* Interface between ISR and main */
        if(request_status){
            request_status = 0;
            ISR_position = current_position;
            ISR_velocity = current_velocity;

            ISR_E = (int32_t)E;
            ISR_P = (int32_t)P;
            ISR_I = (int32_t)I;
            ISR_D = (int32_t)D;
        }


    /* Two state setpoint */
        if(++time_in_state == 400){
            time_in_state = 0;

            if(setpoint_state == 1){
                R = 100;
                setpoint_state = 0;
            }
            else{
                R = 50;
                setpoint_state = 1;
            }
        }


end_of_ISR:

    digitalWrite(ISR_ACTIVE_PIN, LOW);               // Signal end of ISR
}



/***************************************************************************************************
 *  ____            _____  _  __ _____  _____    ____   _    _  _   _  _____
 * |  _ \    /\    / ____|| |/ // ____||  __ \  / __ \ | |  | || \ | ||  __ \
 * | |_) |  /  \  | |     | ' /| |  __ | |__) || |  | || |  | ||  \| || |  | |
 * |  _ <  / /\ \ | |     |  < | | |_ ||  _  / | |  | || |  | || . ` || |  | |
 * | |_) |/ ____ \| |____ | . \| |__| || | \ \ | |__| || |__| || |\  || |__| |
 * |____//_/    \_\\_____||_|\_\\_____||_|  \_\ \____/  \____/ |_| \_||_____/
 *
 **************************************************************************************************/

void loop(){

    char temp_buf[256];
    char line[BUF_LEN];

    while(1){

        request_status = 1;
        while(request_status);

        sprintf(line, "%ld,", millis());
        USART_puts(line);

        sprintf(line, "%ld,%ld,%ld,", ISR_velocity, ISR_position, ISR_E);
        USART_puts(line);

        sprintf(line, "%ld,%ld,%ld\n", ISR_P, ISR_I, ISR_D);
        USART_puts(line);

    }
}






void init_timer_1_CTC(long desired_ISR_freq){  //FIXME shouldn't F_CLK be a parameter?
/**
 * @brief Configure timer #1 to operate in Clear Timer on Capture Match (CTC Mode)
 *
 *      desired_ISR_freq = (f_clk / prescale value) /  Output Compare Registers
 *
 *   For example:
 *        Given a Arduino Uno: F_clk = 16 MHz
 *        let prescale = 64
 *        let desired ISR heartbeat = 100 Hz
 *
 *        if follows that OCR1A = 2500
 *
 * @param desired_ISR_freq is the desired operating frequency of the ISR
 * @param f_clk must be defined globally e.g., #define f_clk 16000000L
 *
 * @return void
 *
 * @note The prescale value is set manually in this function.  Refer to ATMEL ATmega48A/PA/88A/PA/168A/PA/328/P datasheet for specific settings.
 *
 * @warning There are no checks on the desired_ISR_freq parameter.  Use this function with caution.
 *
 * @warning Use of this code will break the Arduino Servo() library.
 */
    cli();                                          // Disable global
    TCCR1A = 0;                                     // Clear timer counter control registers.  The initial value is zero but it appears Arduino code modifies them on startup...
    TCCR1B = 0;
    TCCR1B |= (1 << WGM12);                         // Timer #1 using CTC mode
    TIMSK1 |= (1 << OCIE1A);                        // Enable CTC interrupt
    TCCR1B |= (1 << CS10)|(1 << CS11);              // Prescale: divide by f_clk by 64.  Note SC12 cleared earlier
    OCR1A = (F_CLK / 64L) / desired_ISR_freq;       // Interrupt when TCNT1 equals the top value of the counter specified by OCR
    sei();                                          // Enable global
}













void MD10C_H_bridge_driver(int32_t drive){            // FIXME put this in the library along with an init section

    if(drive > 250)
        drive = 250;

    if(drive < -250)
        drive = -250;

    if(drive >= 0){
        OCR2B = 0;
        OCR2B = 0;
        OCR2B = 0;
        OCR2B = 0;
        OCR2B = 0;
        digitalWrite(MOT_DIR_PIN, HIGH);
        OCR2B = (uint8_t)drive;                      // using fast PWM on pin 3

    }
    else{
        OCR2B = 0;
        OCR2B = 0;
        OCR2B = 0;
        OCR2B = 0;
        OCR2B = 0;
        digitalWrite(MOT_DIR_PIN, LOW);
        OCR2B = (uint8_t)(0 - drive);
    }
}
