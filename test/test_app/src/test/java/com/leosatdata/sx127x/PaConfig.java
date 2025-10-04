package com.leosatdata.sx127x;

public class PaConfig {

	private sx127x_pa_pin_t pin;
	private int power;

	public PaConfig() {
		// do nothing
	}

	public PaConfig(sx127x_pa_pin_t pin, int power) {
		this.pin = pin;
		this.power = power;
	}

	public sx127x_pa_pin_t getPin() {
		return pin;
	}

	public void setPin(sx127x_pa_pin_t pin) {
		this.pin = pin;
	}

	public int getPower() {
		return power;
	}

	public void setPower(int power) {
		this.power = power;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((pin == null) ? 0 : pin.hashCode());
		result = prime * result + power;
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		PaConfig other = (PaConfig) obj;
		if (pin != other.pin)
			return false;
		if (power != other.power)
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "[pin=" + pin + ", power=" + power + "]";
	}

}
