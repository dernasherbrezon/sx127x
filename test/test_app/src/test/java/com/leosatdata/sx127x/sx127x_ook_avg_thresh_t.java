package com.leosatdata.sx127x;

public enum sx127x_ook_avg_thresh_t {

	SX127X_2_PI("2pi"), SX127X_4_PI("4pi"), SX127X_8_PI("8pi"), SX127X_32_PI("32pi");

	private final String name;

	private sx127x_ook_avg_thresh_t(String name) {
		this.name = name;
	}

	public String getName() {
		return name;
	}

	public static sx127x_ook_avg_thresh_t valueOfName(String str) {
		for (sx127x_ook_avg_thresh_t cur : values()) {
			if (cur.name.equals(str)) {
				return cur;
			}
		}
		return null;
	}

}
