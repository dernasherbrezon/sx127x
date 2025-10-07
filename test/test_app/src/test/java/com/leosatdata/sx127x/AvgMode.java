package com.leosatdata.sx127x;

public class AvgMode {

	private sx127x_ook_avg_offset_t value;
	private sx127x_ook_avg_thresh_t thresh;

	public AvgMode() {
		// do nothing
	}

	public AvgMode(sx127x_ook_avg_offset_t value, sx127x_ook_avg_thresh_t thresh) {
		this.value = value;
		this.thresh = thresh;
	}

	public sx127x_ook_avg_offset_t getValue() {
		return value;
	}

	public void setValue(sx127x_ook_avg_offset_t value) {
		this.value = value;
	}

	public sx127x_ook_avg_thresh_t getThresh() {
		return thresh;
	}

	public void setThresh(sx127x_ook_avg_thresh_t thresh) {
		this.thresh = thresh;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((thresh == null) ? 0 : thresh.hashCode());
		result = prime * result + ((value == null) ? 0 : value.hashCode());
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
		AvgMode other = (AvgMode) obj;
		if (thresh != other.thresh)
			return false;
		if (value != other.value)
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "[value=" + value + ", thresh=" + thresh + "]";
	}

}
