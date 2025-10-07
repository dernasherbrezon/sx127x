package com.leosatdata.sx127x;

public enum sx127x_ook_avg_offset_t {

	SX127X_0_DB("0db"), SX127X_2_DB("2db"), SX127X_4_DB("4db"), SX127X_6_DB("6db");

	private final String name;

	private sx127x_ook_avg_offset_t(String name) {
		this.name = name;
	}

	public String getName() {
		return name;
	}

	public static sx127x_ook_avg_offset_t valueOfName(String str) {
		for (sx127x_ook_avg_offset_t cur : values()) {
			if (cur.name.equals(str)) {
				return cur;
			}
		}
		return null;
	}

}
