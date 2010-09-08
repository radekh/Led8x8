/*
 * Project: Led8x8
 * Description: Driving Matrix LED by Arduino.
 * Copyright (c) 2010 by Radek Hnilica
 * Version: 0.2
 * Date: 2010-09-08
 * License: GPLv3 or at your opinion higher version.
 *
 * Hardware Description:
 *  This is the simplest way of connecting LED matrix to Arduino
 *  and/or ATmega328/168.  The LED matrix is connected directly to the
 *  Arduino, only with help of 8 resistors.  I want be sure to do not
 *  shor circuit anything.  The resistors can be on any side of
 *  matrix, on column or row pins, it does not matter.  The Matrix is
 *  connected to the Arduino as described in cols[] and rows[] arrays.
 *  So you can connect it any way you want.  On my breadboard I
 *  connected it in such a way that D13 and A5 remains free.  D13
 *  because there is internal LED I can use for another purposes.  And
 *  A5 left as last pin.  You can use D13 for digtial I/O, a button
 *  for instance.  And A5 can do some analogue measurment.
 *
 * Software Description:
 *  The display code is interrupt driven.  I chose Timer2 to produce
 *  interrupts and the row display routine is run as the interrupt
 *  vector TIMER2_COMPA_vect.  This frees my hand to code whatever I
 *  like in the main loop.
 */

// How is the led matrix connected to Arduino.
int cols[8] = {10, 11, 12, 14,  15, 16, 17, 18};
int rows[8] = {2, 3, 4, 5,  6, 7, 8, 9};

uint8_t vram[8];

void setup() {
	// Set pins connected to the LED Matric to output mode, and
	// turn off all the leds.
	for (int i = 0; i <= 7; i++) {
		digitalWrite(cols[i], LOW);
		digitalWrite(rows[i], HIGH);
		pinMode(cols[i], OUTPUT);
		pinMode(rows[i], OUTPUT);
	}

	// clear_vram();

	// Paint something to vram
	vram[0] = 0b00000000;
	vram[1] = 0b00110110;
	vram[2] = 0b01001001;
	vram[3] = 0b01000001;
	vram[4] = 0b00100010;
	vram[5] = 0b00010100;
	vram[6] = 0b00001000;
	vram[7] = 0b10000000;

	/*
	 * Setup Timer2 to CTC with interrupt mode with no PWM.
	 */
	noInterrupts();
	// Use internal clock from clkI/O. Must be set before TCNT2,
	// OCR2A, OCR2B, TCCR2A and TCCR2B, because of possible
	// corruption of these!  Better to write in sequence and not
	// in one command because of datasheet!
	ASSR &= ~(1<<EXCLK);    // EXCLK=0
        ASSR &= ~(1<<AS2);      // AS2=0
	// Clock Select = clkT2S/1024 (From prescaler) [CS2 2:1:0 = 111]
	TCCR2B |= 0b00000111;   // clkT2S is 16MHz/1024 = 15625Hz
	// Waveform Generation = CTC [WGM2 2:1:0 = 010 ]
	TCCR2A &= 0b11111010; TCCR2A |= 0B00000010;
	TCCR2B &= 0b11110111;
	// Compare Match Output A and B mode = Normal [COM2A1:0 COM2B1:0 = 0000]
        TCCR2A &= 0b00001111;
	// Set TOP
	OCR2A = 38;	// TOP value for mode CTC. Should generate 400Hz
	// Allow interrupt only from Timer2 Compare A Match.  Overflow
	// does not work as I expected initially.
	// OCIE2B=0, OCIE2A=1, TOIE2=0
	TIMSK2 = 0b00000010;
	// Clear the Timer/Counter2 Interrupt Flags
	TIFR2 = 0b00000000;
	interrupts();

	pinMode(13, OUTPUT);	// DEBUG
}

void loop() {
	static unsigned long last_time = 0;
	unsigned long now = millis();
	union {
		unsigned long num;
		unsigned char bs[4];
	} ms;


	if (now > 10000) {
		/* Write millis to vram */
		ms.num = millis();
		vram[0] = ms.bs[0];
		vram[1] = ms.bs[1];
		vram[2] = ms.bs[2];
		vram[3] = ms.bs[3];
	}

	if (now > last_time + 500) {
		last_time += 500;
		digitalWrite(13, !digitalRead(13));
	}
	delay(1);
}

/**/
ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) {
        static uint8_t display_row = 0;
	static uint8_t last_row = 7;
	       uint8_t row = (last_row+1) & 7;
               uint8_t vbyte = vram[row];

	digitalWrite(rows[display_row], HIGH); // Switch OFF
	display_row++; display_row &= 7;
	vbyte = vram[(int)display_row];

	/* Build new byte on output pins */
	for (uint8_t i=0; i <= 7; i++) {
		if ((vbyte & 0x80)) {
			digitalWrite(cols[i], HIGH);
		} else {
			digitalWrite(cols[i], LOW);
		}
		vbyte <<= 1;
	}
	digitalWrite(rows[display_row], LOW);
}

/*
 * This function displays one row from videoram.  It remembers what
 * row was the last shown adn display the next row.  So no arguments
 * to this function.
 */
#define STEP 100
void display(void) {
	static int last_row = 7;
	static unsigned long last_time;
	unsigned long now = millis();
	int row = (last_row+1) & 07;
	uint8_t vbyte = vram[row];

	/* Check the time */
	if (now < last_time + STEP) {
		return;
	}
	//last_time += STEP;
	last_time = now;

	/* Switch off the last row */
	digitalWrite(rows[last_row], HIGH);

	/* Display new row */
	for (int i=0; i <= 7; i++) {
		if ((vbyte & 0x80)) {
			digitalWrite(cols[i], HIGH);
		} else {
			digitalWrite(cols[i], LOW);
		}
		vbyte <<= 1;
	}
	digitalWrite(rows[row], LOW);
	last_row = row;
}

void clear_vram(void) {
	for (int i=0; i<=7; i++) {
		vram[i] = 0;
	}
}

/* Keep this comment at the end of the file
 *Local variables:
 *mode: c++
 *coding: utf-8
 *End:
 */
