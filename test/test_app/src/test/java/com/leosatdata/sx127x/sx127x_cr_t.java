package com.leosatdata.sx127x;

public enum sx127x_cr_t {

	SX127x_CR_4_5("4_5"), SX127x_CR_4_6("4_6"), SX127x_CR_4_7("4_7"), SX127x_CR_4_8("4_8");

	private final String name;

	private sx127x_cr_t(String name) {
		this.name = name;
	}

	public String getName() {
		return name;
	}

	public static sx127x_cr_t valueFromName(String name) {
		for (sx127x_cr_t cur : values()) {
			if (cur.name.equals(name)) {
				return cur;
			}
		}
		return null;
	}

}
