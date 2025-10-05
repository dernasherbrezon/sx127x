package com.leosatdata.sx127x;

public enum sx127x_rssi_smoothing_t {

	SX127X_2("2"), SX127X_4("4"), SX127X_8("8"), SX127X_16("16"), SX127X_32("32"), SX127X_64("64"), SX127X_128("128"), SX127X_256("256");

	private final String name;

	private sx127x_rssi_smoothing_t(String name) {
		this.name = name;
	}

	public String getName() {
		return name;
	}

	public static sx127x_rssi_smoothing_t valueOfName(String str) {
		for (sx127x_rssi_smoothing_t cur : values()) {
			if (cur.name.equals(str)) {
				return cur;
			}
		}
		return null;
	}

}
