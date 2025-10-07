package com.leosatdata.sx127x;

public enum sx127x_mode_t {

	SX127X_MODE_SLEEP, // SLEEP
	SX127X_MODE_STANDBY, // STDBY
	SX127X_MODE_FSTX, // Frequency synthesis TX
	SX127X_MODE_TX, // Transmit
	SX127X_MODE_FSRX, // Frequency synthesis RX
	SX127X_MODE_RX_CONT, // Receive continuous
	SX127X_MODE_RX_SINGLE, // Receive single
	SX127X_MODE_CAD; // Channel activity detection
}
