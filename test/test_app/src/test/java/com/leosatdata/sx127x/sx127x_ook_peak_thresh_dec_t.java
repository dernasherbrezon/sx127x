package com.leosatdata.sx127x;

public enum sx127x_ook_peak_thresh_dec_t {

	SX127X_1_1_CHIP("1.1"), SX127X_1_2_CHIP("1.2"), SX127X_1_4_CHIP("1.4"), SX127X_1_8_CHIP("1.8"), SX127X_2_1_CHIP("2.1"), SX127X_4_1_CHIP("4.1"), SX127X_8_1_CHIP("8.1"), SX127X_16_1_CHIP("16.1");

	private final String name;

	private sx127x_ook_peak_thresh_dec_t(String name) {
		this.name = name;
	}

	public String getName() {
		return name;
	}

	public static sx127x_ook_peak_thresh_dec_t valueOfName(String str) {
		for (sx127x_ook_peak_thresh_dec_t cur : values()) {
			if (cur.name.equals(str)) {
				return cur;
			}
		}
		return null;
	}

}
