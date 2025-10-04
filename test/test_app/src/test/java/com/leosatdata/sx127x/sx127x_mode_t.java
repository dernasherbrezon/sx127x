package com.leosatdata.sx127x;

public enum sx127x_mode_t {

	SLEEP, // SLEEP
	STANDBY, // STDBY
	FSTX, // Frequency synthesis TX
	TX, // Transmit
	FSRX, // Frequency synthesis RX
	RX_CONT, // Receive continuous
	RX_SINGLE, // Receive single
	CAD; // Channel activity detection
}
