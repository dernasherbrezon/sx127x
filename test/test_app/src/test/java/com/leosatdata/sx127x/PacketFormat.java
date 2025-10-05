package com.leosatdata.sx127x;

public class PacketFormat {

	private sx127x_packet_format_t format;
	private int max_payload_length;

	public PacketFormat() {
		// do nothing
	}

	public PacketFormat(sx127x_packet_format_t format, int max_payload_length) {
		this.format = format;
		this.max_payload_length = max_payload_length;
	}

	public sx127x_packet_format_t getFormat() {
		return format;
	}

	public void setFormat(sx127x_packet_format_t format) {
		this.format = format;
	}

	public int getMax_payload_length() {
		return max_payload_length;
	}

	public void setMax_payload_length(int max_payload_length) {
		this.max_payload_length = max_payload_length;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((format == null) ? 0 : format.hashCode());
		result = prime * result + max_payload_length;
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
		PacketFormat other = (PacketFormat) obj;
		if (format != other.format)
			return false;
		if (max_payload_length != other.max_payload_length)
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "[format=" + format + ", max_payload_length=" + max_payload_length + "]";
	}

}
