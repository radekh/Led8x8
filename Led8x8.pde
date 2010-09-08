/*
 * Project: Led8x8
 * Description: Driving Matrix LED by Arduino.
 * Copyright (c) 2010 by Radek Hnilica
 * Version: 0.3
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

uint8_t vram[8];		// The videoram buffer.
uint8_t secbuf = 0;		// Seconds buffer incremented by ISR.

// Variables for counting time.  Real time in seconds, minutes, ...
uint8_t	time_sec = 0;
uint8_t	time_min = 0;
uint8_t time_hour = 0;

// Function prototypes.
void clear_vram(void);
void put_pixel(uint8_t x, uint8_t y, uint8_t color);
void display_number(uint8_t pos, uint8_t number);
void maintain_clock(void);
void display_time(void);


/*
 * This routine displays one row from the videoram to the LED matrix
 * display.  Each time it display another row so after 8 interrupts it
 * display all the videoram.
 *
 * The Timer2 is set to interrupt with exact frequency of 625Hz.  It's
 * because we want generate 1Hz frequency for the clock.
 */
ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) {
        static uint8_t display_row = 0; // The row displayed in previous run.
	static  int8_t cycle = 1;
               uint8_t vbyte;	// The byte from videoram to dispaly.
	static uint16_t pulses = 625; // Pulses counter.

	if (!--pulses) {
		pulses = 625;

		/*
		 * One second call ie. 1Hz frequency.  We do not
		 * process the second signal here.  Instead we add the
		 * second to the temporary buffer secbuf.  The signal
		 * is processed asynchronically outside of interrupt
		 * routine.
		 */
		secbuf++;	// Add the second to buffer.	
	}
	
        if (--cycle < 0) { cycle=1; } // Cycle counter.
	if (cycle) return;

	// This code is run only one time in cicle, when the cycle is 0
	digitalWrite(rows[display_row], HIGH); // Switch OFF
	display_row++; display_row &= 7;       // Next row.
	vbyte = vram[display_row];	       // Get the row.

	/* Build new byte on output pins */
	for (uint8_t i=0; i <= 7; i++) {
		if ((vbyte & 0x80)) {
			digitalWrite(cols[i], HIGH);
		} else {
			digitalWrite(cols[i], LOW);
		}
		vbyte <<= 1;
	}
	digitalWrite(rows[display_row], LOW); // Switch ON
}/* ISR TIMER2_COMPA_vect */



/*
 * Setup initial state in the variables and hardware.
 */
void setup() {
	// Set pins connected to the LED Matric to output mode, and
	// turn off all the leds.
	for (int i = 0; i <= 7; i++) {
		digitalWrite(cols[i], LOW);
		digitalWrite(rows[i], HIGH);
		pinMode(cols[i], OUTPUT);
		pinMode(rows[i], OUTPUT);
	}

	// Paint something to vram :).
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
	OCR2A = 24;	// TOP value for mode CTC. Should generate 625Hz
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

	/*
	 * After 10 seconds from restart start displaying the millis
	 * value.
	 */
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

	maintain_clock();
	display_time();

	delay(50);
} /* loop() */


/*
 * Display real time in BCD.  The time is displayed in lower half of
 * the display.
 */
void display_time() {
	uint8_t dec, one;	// For BCD

	dec = time_sec / 10;
	one = time_sec % 10;
	display_number(0, one);
	display_number(1, dec);

	display_number(2, 0);	// Blank that position

	dec = time_min / 10;
	one = time_min % 10;
	display_number(3, one);
	display_number(4, dec);

	display_number(5, 0);	// Blank that position

	dec = time_hour / 10;
	one = time_hour % 10;
	display_number(6, one);
	display_number(7, dec);
}/* display_time() */



/*
 * This function check the secbuf and processes the one second pulse.
 * It maintains the real H:M:S time in time_hour, time_min, and
 * time_sec global variables.
 */
void maintain_clock(void) {
	// If there is a one second signal in buffer, process it.
	noInterrupts();		// Cause concurrency from ISR.
	if (secbuf) {
		secbuf--; time_sec++; // Moving second pulse.
	}
	interrupts();		// End of critical section.

	// Do the sec, min, hour "calculation"
	if (time_sec == 60) {
		time_sec = 0;
		time_min++;
		if (time_min == 60) {
			time_min = 0;
			time_hour++;
			if (time_hour == 24) {
				time_hour = 0;
			}
		}
	}
	// the HH:MM:SS is in consistent state.
}


/*
 * Display one number from 0 to 9 (or possibly 15) on the lower part
 * of the display.  Position counts from right to left.
 */
void display_number(uint8_t pos, uint8_t number) {
	for (uint8_t i=0; i<=3; i++) {
		put_pixel(pos, 7-i, number &1);
		number >>=1;
	}
}


/*
 * Clear the contents of videoram.  In another words, blank the
 * screen.
 */
void clear_vram(void) {
	for (int i=0; i<=7; i++) {
		vram[i] = 0;
	}
}

/*
 * Put one pixel into videoram buffer.
 */
void put_pixel(uint8_t x, uint8_t y, uint8_t color) {
	// x selects column, y selects row, and color is 0-black, 1-shine.

	if (color) {
		vram[y] |= 1 << x;
	} else {
		vram[y] &= ~(1 << x);
	}
}

/* Keep this comment at the end of the file
 *Local variables:
 *mode: c++
 *coding: utf-8
 *End:
 */
