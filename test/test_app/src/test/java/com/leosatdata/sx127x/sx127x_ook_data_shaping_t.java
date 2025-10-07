package com.leosatdata.sx127x;

public enum sx127x_ook_data_shaping_t {

	SX127X_1_BIT_RATE("1BITRATE"),
	SX127X_2_BIT_RATE("2BITRATE"),
	SX127X_OOK_SHAPING_NONE("NONE");

	private final String name;

	private sx127x_ook_data_shaping_t(String name) {
		this.name = name;
	}

	public String getName() {
		return name;
	}

	public static sx127x_ook_data_shaping_t valueOfName(String str) {
		for (sx127x_ook_data_shaping_t cur : values()) {
			if (cur.name.equals(str)) {
				return cur;
			}
		}
		return null;
	}

}
