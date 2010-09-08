/*
 * Project: Led8x8
 * Description: Driving Matrix LED by Arduino.
 * Version: 0.1
 * Copyright (c) 2010 by Radek Hnilica
 * License: GPLv3
 */

// How is the led matrix connected to Arduino.
int cols[8] = {10, 11, 12, 14,  15, 16, 17, 18};
int rows[8] = {2, 3, 4, 5,  6, 7, 8, 9};

uint8_t vram[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void setup() {
	// Set pins to output mode
	for (int i = 0; i <= 7; i++) {
		pinMode(cols[i], OUTPUT);
		pinMode(rows[i], OUTPUT);
		digitalWrite(cols[i], LOW);
		digitalWrite(rows[i], HIGH);
	}

	clear_vram();

	// Paint something to vram
	vram[0] = 0b00000000;
	vram[1] = 0b00110110;
	vram[2] = 0b01001001;
	vram[3] = 0b01000001;
	vram[4] = 0b00100010;
	vram[5] = 0b00010100;
	vram[6] = 0b00001000;
	vram[7] = 0b10000000;
}

void loop() {
	display();
	//delay(1);
}

/*
 * This function displays one row from videoram.  It remembers what
 * row was the last shown adn display the next row.  So no arguments
 * to this function.
 */
#define STEP 3
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
