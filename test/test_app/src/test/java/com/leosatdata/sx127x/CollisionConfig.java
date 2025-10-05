package com.leosatdata.sx127x;

public class CollisionConfig {

	private boolean enable;
	private int threshold;

	public CollisionConfig() {
		// do nothing
	}

	public CollisionConfig(boolean enable, int threshold) {
		this.enable = enable;
		this.threshold = threshold;
	}

	public boolean isEnable() {
		return enable;
	}

	public void setEnable(boolean enable) {
		this.enable = enable;
	}

	public int getThreshold() {
		return threshold;
	}

	public void setThreshold(int threshold) {
		this.threshold = threshold;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + (enable ? 1231 : 1237);
		result = prime * result + threshold;
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
		CollisionConfig other = (CollisionConfig) obj;
		if (enable != other.enable)
			return false;
		if (threshold != other.threshold)
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "[enable=" + enable + ", threshold=" + threshold + "]";
	}

}
