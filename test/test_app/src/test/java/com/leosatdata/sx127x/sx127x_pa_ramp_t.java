package com.leosatdata.sx127x;

public enum sx127x_pa_ramp_t {

	SX127X_PA_RAMP_1("3.4ms"), SX127X_PA_RAMP_2("2ms"), SX127X_PA_RAMP_3("1ms"), SX127X_PA_RAMP_4("500us"), SX127X_PA_RAMP_5("250us"), SX127X_PA_RAMP_6("125us"), SX127X_PA_RAMP_7("100us"), SX127X_PA_RAMP_8("62us"), SX127X_PA_RAMP_9("50us"), SX127X_PA_RAMP_10("40us"), SX127X_PA_RAMP_11("31us"),
	SX127X_PA_RAMP_12("25us"), SX127X_PA_RAMP_13("20us"), SX127X_PA_RAMP_14("15us"), SX127X_PA_RAMP_15("12us"), SX127X_PA_RAMP_16("10us");

	private final String name;

	private sx127x_pa_ramp_t(String name) {
		this.name = name;
	}

	public String getName() {
		return name;
	}

	public static sx127x_pa_ramp_t valueOfName(String str) {
		for (sx127x_pa_ramp_t cur : values()) {
			if (cur.name.equals(str)) {
				return cur;
			}
		}
		return null;
	}
}
