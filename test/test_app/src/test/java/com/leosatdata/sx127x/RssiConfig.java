package com.leosatdata.sx127x;

public class RssiConfig {

	private sx127x_rssi_smoothing_t smoothing;
	private int offset;

	public RssiConfig() {
		// do nothing
	}

	public RssiConfig(sx127x_rssi_smoothing_t smoothing, int offset) {
		this.smoothing = smoothing;
		this.offset = offset;
	}

	public sx127x_rssi_smoothing_t getSmoothing() {
		return smoothing;
	}

	public void setSmoothing(sx127x_rssi_smoothing_t smoothing) {
		this.smoothing = smoothing;
	}

	public int getOffset() {
		return offset;
	}

	public void setOffset(int offset) {
		this.offset = offset;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + offset;
		result = prime * result + ((smoothing == null) ? 0 : smoothing.hashCode());
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
		RssiConfig other = (RssiConfig) obj;
		if (offset != other.offset)
			return false;
		if (smoothing != other.smoothing)
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "[smoothing=" + smoothing + ", offset=" + offset + "]";
	}

}
