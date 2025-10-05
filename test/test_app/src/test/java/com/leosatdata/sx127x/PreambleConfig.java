package com.leosatdata.sx127x;

public class PreambleConfig {

	private boolean enable;
	private int detector_size;
	private int detector_tolerance;

	public PreambleConfig() {
		// do nothing
	}

	public PreambleConfig(boolean enable, int detector_size, int detector_tolerance) {
		this.enable = enable;
		this.detector_size = detector_size;
		this.detector_tolerance = detector_tolerance;
	}

	public boolean isEnable() {
		return enable;
	}

	public void setEnable(boolean enable) {
		this.enable = enable;
	}

	public int getDetector_size() {
		return detector_size;
	}

	public void setDetector_size(int detector_size) {
		this.detector_size = detector_size;
	}

	public int getDetector_tolerance() {
		return detector_tolerance;
	}

	public void setDetector_tolerance(int detector_tolerance) {
		this.detector_tolerance = detector_tolerance;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + detector_size;
		result = prime * result + detector_tolerance;
		result = prime * result + (enable ? 1231 : 1237);
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
		PreambleConfig other = (PreambleConfig) obj;
		if (detector_size != other.detector_size)
			return false;
		if (detector_tolerance != other.detector_tolerance)
			return false;
		if (enable != other.enable)
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "[enable=" + enable + ", detector_size=" + detector_size + ", detector_tolerance=" + detector_tolerance + "]";
	}

}
