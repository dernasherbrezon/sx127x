package com.leosatdata.sx127x;

public class sx127x_implicit_header_t {

	private int length;
	private boolean enable_crc;
	private sx127x_cr_t coding_rate;

	public sx127x_implicit_header_t() {
		// do nothing
	}

	public sx127x_implicit_header_t(int length, boolean enable_crc, sx127x_cr_t coding_rate) {
		this.length = length;
		this.enable_crc = enable_crc;
		this.coding_rate = coding_rate;
	}

	public int getLength() {
		return length;
	}

	public void setLength(int length) {
		this.length = length;
	}

	public boolean isEnable_crc() {
		return enable_crc;
	}

	public void setEnable_crc(boolean enable_crc) {
		this.enable_crc = enable_crc;
	}

	public sx127x_cr_t getCoding_rate() {
		return coding_rate;
	}

	public void setCoding_rate(sx127x_cr_t coding_rate) {
		this.coding_rate = coding_rate;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((coding_rate == null) ? 0 : coding_rate.hashCode());
		result = prime * result + (enable_crc ? 1231 : 1237);
		result = prime * result + length;
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		sx127x_implicit_header_t other = (sx127x_implicit_header_t) obj;
		if (coding_rate != other.coding_rate)
			return false;
		if (enable_crc != other.enable_crc)
			return false;
		if (length != other.length)
			return false;
		return true;
	}

}
