package com.leosatdata.sx127x;

public enum sx127x_preamble_type_t {

	P55("55"), PAA("AA");

	private final String name;

	private sx127x_preamble_type_t(String name) {
		this.name = name;
	}

	public String getName() {
		return name;
	}

	public static sx127x_preamble_type_t valueOfName(String str) {
		for (sx127x_preamble_type_t cur : values()) {
			if (cur.name.equals(str)) {
				return cur;
			}
		}
		return null;
	}

}
