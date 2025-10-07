package com.leosatdata.sx127x;

public class PeakMode {

	private sx127x_ook_peak_thresh_step_t step;
	private int floor_threshold;
	private sx127x_ook_peak_thresh_dec_t decrement;

	public PeakMode() {
		// do nothing
	}

	public PeakMode(sx127x_ook_peak_thresh_step_t step, int floor_threshold, sx127x_ook_peak_thresh_dec_t decrement) {
		this.step = step;
		this.floor_threshold = floor_threshold;
		this.decrement = decrement;
	}

	public sx127x_ook_peak_thresh_step_t getStep() {
		return step;
	}

	public void setStep(sx127x_ook_peak_thresh_step_t step) {
		this.step = step;
	}

	public int getFloor_threshold() {
		return floor_threshold;
	}

	public void setFloor_threshold(int floor_threshold) {
		this.floor_threshold = floor_threshold;
	}

	public sx127x_ook_peak_thresh_dec_t getDecrement() {
		return decrement;
	}

	public void setDecrement(sx127x_ook_peak_thresh_dec_t decrement) {
		this.decrement = decrement;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((decrement == null) ? 0 : decrement.hashCode());
		result = prime * result + floor_threshold;
		result = prime * result + ((step == null) ? 0 : step.hashCode());
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
		PeakMode other = (PeakMode) obj;
		if (decrement != other.decrement)
			return false;
		if (floor_threshold != other.floor_threshold)
			return false;
		if (step != other.step)
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "[step=" + step + ", floor_threshold=" + floor_threshold + ", decrement=" + decrement + "]";
	}

}
