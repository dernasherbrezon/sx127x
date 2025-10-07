package com.leosatdata.sx127x;

public class FskDataShaping {

	private sx127x_fsk_data_shaping_t shaping;
	private sx127x_pa_ramp_t pa_ramp;

	public FskDataShaping() {
		// do nothing
	}

	public FskDataShaping(sx127x_fsk_data_shaping_t shaping, sx127x_pa_ramp_t pa_ramp) {
		this.shaping = shaping;
		this.pa_ramp = pa_ramp;
	}

	public sx127x_fsk_data_shaping_t getShaping() {
		return shaping;
	}

	public void setShaping(sx127x_fsk_data_shaping_t shaping) {
		this.shaping = shaping;
	}

	public sx127x_pa_ramp_t getPa_ramp() {
		return pa_ramp;
	}

	public void setPa_ramp(sx127x_pa_ramp_t pa_ramp) {
		this.pa_ramp = pa_ramp;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((pa_ramp == null) ? 0 : pa_ramp.hashCode());
		result = prime * result + ((shaping == null) ? 0 : shaping.hashCode());
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
		FskDataShaping other = (FskDataShaping) obj;
		if (pa_ramp != other.pa_ramp)
			return false;
		if (shaping != other.shaping)
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "[shaping=" + shaping + ", pa_ramp=" + pa_ramp + "]";
	}

}
