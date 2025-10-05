package com.leosatdata.sx127x;

import java.util.Arrays;

public class FhssConfig {

	private int period;
	private long[] frequencies;

	public FhssConfig() {
		// do nothing
	}

	public FhssConfig(int period, long[] frequencies) {
		this.period = period;
		this.frequencies = frequencies;
	}

	public int getPeriod() {
		return period;
	}

	public void setPeriod(int period) {
		this.period = period;
	}

	public long[] getFrequencies() {
		return frequencies;
	}

	public void setFrequencies(long[] frequencies) {
		this.frequencies = frequencies;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + Arrays.hashCode(frequencies);
		result = prime * result + period;
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
		FhssConfig other = (FhssConfig) obj;
		if (!Arrays.equals(frequencies, other.frequencies))
			return false;
		if (period != other.period)
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "[period=" + period + ", frequencies=" + Arrays.toString(frequencies) + "]";
	}

}
