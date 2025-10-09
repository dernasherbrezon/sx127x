package com.leosatdata.sx127x;

public class OpMode {

	sx127x_mode_t mode;
	sx127x_modulation_t modulation;

	public OpMode() {
		// do nothing
	}

	public OpMode(sx127x_mode_t mode, sx127x_modulation_t modulation) {
		this.mode = mode;
		this.modulation = modulation;
	}

	public sx127x_mode_t getMode() {
		return mode;
	}

	public void setMode(sx127x_mode_t mode) {
		this.mode = mode;
	}

	public sx127x_modulation_t getModulation() {
		return modulation;
	}

	public void setModulation(sx127x_modulation_t modulation) {
		this.modulation = modulation;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((mode == null) ? 0 : mode.hashCode());
		result = prime * result + ((modulation == null) ? 0 : modulation.hashCode());
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
		OpMode other = (OpMode) obj;
		if (mode != other.mode)
			return false;
		if (modulation != other.modulation)
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "[mode=" + mode + ", modulation=" + modulation + "]";
	}

}
