package com.leosatdata.sx127x;

public class sx127x_frame_t {

	private int frequency_error;
	private int rssi;
	private float snr;
	private long timestamp;
	private String message;

	public int getFrequency_error() {
		return frequency_error;
	}

	public void setFrequency_error(int frequency_error) {
		this.frequency_error = frequency_error;
	}

	public int getRssi() {
		return rssi;
	}

	public void setRssi(int rssi) {
		this.rssi = rssi;
	}

	public float getSnr() {
		return snr;
	}

	public void setSnr(float snr) {
		this.snr = snr;
	}

	public long getTimestamp() {
		return timestamp;
	}

	public void setTimestamp(long timestamp) {
		this.timestamp = timestamp;
	}

	public String getMessage() {
		return message;
	}

	public void setMessage(String message) {
		this.message = message;
	}

}
