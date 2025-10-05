package com.leosatdata.sx127x;

public enum sx127x_fsk_data_shaping_t {

	BT03("BT_0.3"), BT05("BT_0.5"), BT10("BT_1.0"), NONE("NONE");

	private final String name;

	private sx127x_fsk_data_shaping_t(String name) {
		this.name = name;
	}

	public String getName() {
		return name;
	}

	public static sx127x_fsk_data_shaping_t valueOfName(String str) {
		for (sx127x_fsk_data_shaping_t cur : values()) {
			if (cur.name.equals(str)) {
				return cur;
			}
		}
		return null;
	}

}
