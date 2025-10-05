package com.leosatdata.sx127x;

public class AddressConfig {

	private sx127x_address_filtering_t type;
	private int node_address;
	private int broadcast_address;

	public AddressConfig() {
		// do nothing
	}

	public AddressConfig(sx127x_address_filtering_t type, int node_address, int broadcast_address) {
		this.type = type;
		this.node_address = node_address;
		this.broadcast_address = broadcast_address;
	}

	public sx127x_address_filtering_t getType() {
		return type;
	}

	public void setType(sx127x_address_filtering_t type) {
		this.type = type;
	}

	public int getNode_address() {
		return node_address;
	}

	public void setNode_address(int node_address) {
		this.node_address = node_address;
	}

	public int getBroadcast_address() {
		return broadcast_address;
	}

	public void setBroadcast_address(int broadcast_address) {
		this.broadcast_address = broadcast_address;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + broadcast_address;
		result = prime * result + node_address;
		result = prime * result + ((type == null) ? 0 : type.hashCode());
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
		AddressConfig other = (AddressConfig) obj;
		if (broadcast_address != other.broadcast_address)
			return false;
		if (node_address != other.node_address)
			return false;
		if (type != other.type)
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "[type=" + type + ", node_address=" + node_address + ", broadcast_address=" + broadcast_address + "]";
	}

}
