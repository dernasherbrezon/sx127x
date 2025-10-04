package com.leosatdata.sx127x;

public enum sx127x_mode_t {

	SLEEP, // SLEEP
	STANDBY, // STDBY
	FSTX, // Frequency synthesis TX
	TX, // Transmit
	FSRX, // Frequency synthesis RX
	RXCONT, // Receive continuous
	RXSINGLE, // Receive single
	CAD; // Channel activity detection
}
