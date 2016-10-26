#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/cpufunc.h>
#include <stdlib.h>
#include "uart.h"

#define A1 0x11c1
#define Bb1 0x10c2
#define B1 0x0fd2
#define C2 0x0eee
#define Db2 0x0e18
#define D2 0x0d4d
#define Eb2 0x0c8e
#define E2 0x0bda
#define F2 0x0b2f
#define Gb2 0x0a8f
#define G2 0x09f7
#define Ab2 0x0968
#define A2 0x08e1
#define Bb2 0x0861
#define B2 0x07e9
#define C3 0x0777
#define Db3 0x070c
#define D3 0x06a7
#define Eb3 0x0647
#define E3 0x05ed
#define F3 0x0598
#define Gb3 0x0547
#define G3 0x04fc
#define Ab3 0x04b4
#define A3 0x0470
#define Bb3 0x0431
#define B3 0x03f4
#define C4 0x03bc
#define Db4 0x0386
#define D4 0x0353
#define Eb4 0x0324
#define E4 0x02f6
#define F4 0x02cc
#define Gb4 0x02a4
#define G4 0x027e
#define Ab4 0x025a
#define A4 0x0238
#define Bb4 0x0218
#define B4 0x01fa
#define C5 0x01de
#define Db5 0x01c3
#define D5 0x01aa
#define Eb5 0x0192
#define E5 0x017b
#define F5 0x0166
#define Gb5 0x0152
#define G5 0x013f
#define Ab5 0x012d
#define A5 0x011c
#define Bb5 0x010c
// higher than Bb5, start moving through the wave table in increments of two, so 32 samples per period instead of 64
#define B5 0x01fa
#define C6 0x01de
#define Db6 0x01c3
#define D6 0x01aa
#define Eb6 0x0192
#define E6 0x017b
#define F6 0x0166
#define Gb6 0x0152
#define G6 0x013f
#define Ab6 0x012d
#define A6 0x011c
#define Bb6 0x010c
// higher than Bb6, start moving through wave table in increments of 4, 16 samples per period
#define B6 0x01fa
#define C7 0x01de
#define Db7 0x01c3
#define D7 0x01aa
#define Eb7 0x0192
#define E7 0x017b
#define F7 0x0166
#define Gb7 0x0152
#define G7 0x013f
#define Ab7 0x012d
#define A7 0x011c
#define Bb7 0x010c

#define NUM_NOTES 74

uint16_t notes_array[NUM_NOTES] = {A1,Bb1,B1,C2,Db2,D2,Eb2,E2,F2,Gb2,G2,Ab2,A2,Bb2,B2,C3,Db3,D3,Eb3,E3,F3,Gb3,G3,Ab3,A3,Bb3,B3,C4,Db4,D4,Eb4,E4,F4,Gb4,G4,Ab4,A4,Bb4,B4,C5,Db5,D5,Eb5,E5,F5,Gb5,G5,Ab5,A5,Bb5,B5,C6,Db6,D6,Eb6,E6,F6,Gb6,G6,Ab6,A6,Bb6,B6,C7,Db7,D7,Eb7,E7,F7,Gb7,G7,Ab7,A7,Bb7};
uint8_t sine_wave[64] = {128,141,153,165,177,188,199,209,219,227,234,241,246,250,254,255,255,255,254,250,246,241,234,227,219,209,199,188,177,165,153,141,128,115,103,91,79,68,57,47,37,29,22,15,10,6,2,1,0,1,2,6,10,15,22,29,37,47,57,68,79,91,103,115};
uint8_t saw_wave[64] = {252,248,244,240,236,232,228,224,220,216,212,208,204,200,196,192,188,184,180,176,172,168,164,160,156,152,148,144,140,136,132,128,124,120,116,112,108,104,100,96,92,88,84,80,76,72,68,64,60,56,52,48,44,40,36,32,28,24,20,16,12,8,4,0};
uint8_t tri_wave[64] = {128,136,144,152,160,168,176,184,192,200,208,216,224,232,240,248,255,248,240,232,224,216,208,200,192,184,176,168,160,152,144,136,128,120,112,104,96,88,80,72,64,56,48,40,32,24,16,8,0,8,16,24,32,40,48,56,64,72,80,88,96,104,112,120};
uint8_t sharktooth[64] = {112,119,126,133,140,147,154,161,168,175,182,189,196,203,210,217,255,248,240,232,224,216,208,200,192,184,176,168,160,152,144,136,128,120,112,104,96,88,80,72,64,56,48,40,32,24,16,8,0,7,14,21,28,35,42,49,56,63,70,77,84,91,98,105};
volatile uint8_t wave_index = 0;
volatile uint8_t next_wave_index = 1;
uint8_t currentNote; // index of current note playing (used to determine the rate of advancement through the notes array for higher notes)
uint8_t numNotesPlaying = 0; // number of notes that have been pressed down

ISR(TIMER0_OVF_vect) {
    // PD7 high = saw wave, low = sine wave
    if (PIND & 0x80) {
        OCR0A = sharktooth[wave_index];
    }
    else {
        OCR0A = tri_wave[wave_index];
    }
}

ISR(TIMER1_COMPA_vect) {
    wave_index = next_wave_index;

    // notes 0-49 will use every sample in the wave table on every period
    if (currentNote < 50) {
        ++next_wave_index;
    }
    // from notes 50-61, play 32 samples per period, above that, 16 samples per period
    else if (currentNote >= 50) {
        if (currentNote < 62) {
            next_wave_index += 2;
        }
        else {
            next_wave_index += 4;
        }
    }

    // if next wave index is out of bounds, reset to 0
    if (next_wave_index >= 64) {
        next_wave_index = 0;
    }
}

void shiftOut(uint8_t data) {
    uint8_t i;
    for (i = 0; i < 8; i++) {
        if (data & 0x01) {
            PORTB |= 0x02; // this bit of the data is 1 (shift 1 to shift register)
        }
        else {
            PORTB &= ~(0x02); // this bit is 0 (shift 0 to shift register)
        }
        _NOP();
        PORTB |= 0x04; // rising clock
        _NOP();
        PORTB &= ~(0x04); // falling clock
        _NOP();
        data >>= 1;
    }
}

int main() {
    DDRB |= 0x01; // PB0 is output
    DDRB |= 0x06; // PB1 and PB2 are output (for shift register)    
    DDRD &= ~(0x80); // PD7 is input

    // set up PWM output pin configuration
    DDRD |= 0x40; // OC0A (PD6/pin 12) set to output
    TCCR0A |= (1 << COM0A1); // configure OC0A to clear when value compare fires, set when timer resets to bottom

    // set WGM0 2:0 to 011 to denote fast PWM
    TCCR0A |= (1 << WGM01) | (1 << WGM00);
    
    TCNT0 = 0; // set timer 0 counter to 0
    TIMSK0 |= (1 << TOIE0); // enable timer 0 overflow interrupt

    TCCR1B |= (1 << WGM12); // enable clear-timer-on-compare-match (CTC) mode for timer 1
    TIMSK1 |= (1 << OCIE1A); // enable timer 1 output compare A  match interrupt
    TCNT1 = 0; // set timer 1 counter to 0

    // set up UART for MIDI communication
    uart_init(UART_BAUD_SELECT(31250,F_CPU));
    
    sei(); // enable interrupts

    uint16_t retrieved_bytes;
    uint8_t data;

    while (1) {
/*        
        OCR1A = E4;
        _delay_ms(delay);
        OCR1A = E5;
        _delay_ms(delay);
        OCR1A = A4;
        _delay_ms(delay);
        OCR1A = B4;
        _delay_ms(delay);
        OCR1A = Eb5;
        _delay_ms(delay);
        OCR1A = Ab4;
        _delay_ms(delay);
        OCR1A = A4;
        _delay_ms(delay);
*/
        retrieved_bytes = uart_getc();
        if (retrieved_bytes & UART_NO_DATA) {
            // no data available from UART
            //PORTB |= 0x01;
        }
        else {
            if (retrieved_bytes & UART_FRAME_ERROR) {
                // PORTB |= 0x01; // turn on debug light
            }
            else {
                // process the data
                data = (unsigned char)retrieved_bytes;
                // note on command
                if ((data & 0xf0) == 0x90) {
                    shiftOut(data);
                    PORTB |= 0x01; 
                    while ((retrieved_bytes = uart_getc()) & UART_NO_DATA)
                        ; // wait until data is received
                    data = (unsigned char)retrieved_bytes;
                    data -= 0x21; // the 0 element of the notes array is A1, remove offset so our notes line up
                    currentNote = data;
                    if (currentNote >= 0 && currentNote < NUM_NOTES) {
                        OCR1A = notes_array[currentNote];
                        TCCR0B |= (1 << CS00); // start timer 0 with no prescaling
                        TCCR1B |= (1 << CS10); // start timer 1 with no prescaling
                        ++numNotesPlaying;
                    }
                    else {
                        TCCR0B &= ~(0x07); // stop note if there is one playing (stop timer 0)
                        TCCR1B &= ~(0x07); // stop timer 1
                        TCNT0 = 0;
                        TCNT1 = 0;
                    }
                    
                    // skip the third (velocity) byte, we don't need it
                    while ((retrieved_bytes = uart_getc()) & UART_NO_DATA)
                        ; // wait until there is data

                }
                // note off command (monophonic, stop note if there are no keys held down)
                else if ((data & 0xf0) == 0x80) {
                    PORTB &= ~(0x01); // turn off LED
                    --numNotesPlaying;
                    if (numNotesPlaying <= 0) {
                        numNotesPlaying = 0; // in case the number of notes became negative somehow
                    }
                    TCCR0B &= ~(0x07); // stop timer 0 (will stop the sound)
                    TCNT0 = 0; // reset timer 0
                    TCCR1B &= ~(0x07); // stop timer 1
                    TCNT1 = 0;
                    while ((retrieved_bytes = uart_getc()) & UART_NO_DATA)
                        ; // note byte (skip)
                    while ((retrieved_bytes = uart_getc()) & UART_NO_DATA)
                        ; // velocity byte (skip)
                }
            }
        }
    }
}
