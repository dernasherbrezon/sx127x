package com.leosatdata.sx127x;

public enum sx127x_ook_peak_thresh_step_t {

	SX127X_0_5_DB("0.5db"), SX127X_1_0_DB("1.0db"), SX127X_1_5_DB("1.5db"), SX127X_2_0_DB("2.0db"), SX127X_3_0_DB("3.0db"), SX127X_4_0_DB("4.0db"), SX127X_5_0_DB("5.0db"), SX127X_6_0_DB("6.0db");

	private final String name;

	private sx127x_ook_peak_thresh_step_t(String name) {
		this.name = name;
	}

	public String getName() {
		return name;
	}

	public static sx127x_ook_peak_thresh_step_t valueOfName(String str) {
		for (sx127x_ook_peak_thresh_step_t cur : values()) {
			if (cur.name.equals(str)) {
				return cur;
			}
		}
		return null;
	}

}
