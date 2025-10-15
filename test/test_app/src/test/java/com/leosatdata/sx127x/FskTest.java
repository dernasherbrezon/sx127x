package com.leosatdata.sx127x;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

public class FskTest {

	private static Sx127x rx;
	private static Sx127x tx;
	private static long frequency;

	@BeforeClass
	public static void init() {
		rx = new Sx127x(System.getProperty("rx"), 10000, "rx");
		rx.start();
		tx = new Sx127x(System.getProperty("tx"), 10000, "tx");
		tx.start();

		String freqStr = System.getProperty("freq");
		if (freqStr == null) {
			freqStr = "434200000";
		}
		frequency = Long.valueOf(freqStr);
	}

	@Test
	public void testBeacons() {
		int beaconInterval = 200;
		int beaconsExpected = 5;
		String message = LoraTest.createRandom(6);

		rx.sx127x_fsk_ook_set_packet_format(new PacketFormat(sx127x_packet_format_t.SX127X_FIXED, message.length() / 2));
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SX127X_MODE_RX_CONT, sx127x_modulation_t.SX127X_MODULATION_FSK));

		tx.sx127x_fsk_ook_set_packet_format(new PacketFormat(sx127x_packet_format_t.SX127X_FIXED, message.length() / 2));
		tx.sx127x_tx_set_pa_config(new PaConfig(sx127x_pa_pin_t.SX127X_PA_PIN_BOOST, 4));
		tx.sx127x_fsk_ook_tx_start_beacon(message, beaconInterval);

		try {
			Thread.sleep((long) ((beaconsExpected + 0.3f) * beaconInterval));
		} catch (InterruptedException e) {
			Thread.currentThread().interrupt();
			return;
		}
		tx.sx127x_fsk_ook_tx_stop_beacon();

		LoraTest.assertFrames(rx, message, message, message, message, message);
	}

	@Test
	public void testFixed() {
		int[] messageSizes = new int[] { 6, 255, 2047 };
		for (int i = 0; i < messageSizes.length; i++) {
			String message = LoraTest.createRandom(messageSizes[i]);

			rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SX127X_MODE_SLEEP, sx127x_modulation_t.SX127X_MODULATION_FSK));
			rx.sx127x_fsk_ook_set_packet_format(new PacketFormat(sx127x_packet_format_t.SX127X_FIXED, message.length() / 2));
			rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SX127X_MODE_RX_CONT, sx127x_modulation_t.SX127X_MODULATION_FSK));

			tx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SX127X_MODE_SLEEP, sx127x_modulation_t.SX127X_MODULATION_FSK));
			tx.sx127x_fsk_ook_set_packet_format(new PacketFormat(sx127x_packet_format_t.SX127X_FIXED, message.length() / 2));
			tx.sx127x_tx_set_pa_config(new PaConfig(sx127x_pa_pin_t.SX127X_PA_PIN_BOOST, 4));
			tx.sx127x_fsk_ook_tx_set_for_transmission(message);
			tx.tx(sx127x_modulation_t.SX127X_MODULATION_FSK);

			LoraTest.assertFrames(rx, message);
		}
	}

	@Test
	public void testMaxBaud() {
		rx.sx127x_fsk_ook_set_bitrate(300000);
		rx.sx127x_fsk_set_fdev(100000);
		rx.sx127x_fsk_ook_rx_set_bandwidth(170000);
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SX127X_MODE_RX_CONT, sx127x_modulation_t.SX127X_MODULATION_FSK));

		String message = LoraTest.createRandom(6);
		tx.sx127x_tx_set_pa_config(new PaConfig(sx127x_pa_pin_t.SX127X_PA_PIN_BOOST, 4));
		tx.sx127x_fsk_ook_set_bitrate(300000);
		tx.sx127x_fsk_set_fdev(100000);
		tx.sx127x_fsk_ook_rx_set_bandwidth(170000);
		tx.sx127x_fsk_ook_tx_set_for_transmission(message);
		tx.tx(sx127x_modulation_t.SX127X_MODULATION_FSK);

		LoraTest.assertFrames(rx, message);
	}

	@Test
	public void testFiltered() {
		int nodeAddress = 0xbe;
		int broadcastAddress = 0xfe;

		rx.sx127x_fsk_ook_set_address_filtering(new AddressConfig(sx127x_address_filtering_t.SX127X_FILTER_NODE_AND_BROADCAST, nodeAddress, broadcastAddress));
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SX127X_MODE_RX_CONT, sx127x_modulation_t.SX127X_MODULATION_FSK));

		tx.sx127x_tx_set_pa_config(new PaConfig(sx127x_pa_pin_t.SX127X_PA_PIN_BOOST, 4));

		String m1 = "01CAFE";
		tx.sx127x_fsk_ook_tx_set_for_transmission_with_address(m1, nodeAddress);
		tx.tx(sx127x_modulation_t.SX127X_MODULATION_FSK);

		String m2 = "02CAFE";
		tx.sx127x_fsk_ook_tx_set_for_transmission_with_address(m2, 0x12);
		tx.tx(sx127x_modulation_t.SX127X_MODULATION_FSK);

		String m3 = "03CAFE";
		tx.sx127x_fsk_ook_tx_set_for_transmission(m3);
		tx.tx(sx127x_modulation_t.SX127X_MODULATION_FSK);

		String m4 = "04CAFE";
		tx.sx127x_fsk_ook_tx_set_for_transmission_with_address(m4, broadcastAddress);
		tx.tx(sx127x_modulation_t.SX127X_MODULATION_FSK);

		LoraTest.assertFrames(rx, m1, m4);
	}

	@Test
	public void testVariableLength() {
		rx.sx127x_fsk_ook_set_packet_format(new PacketFormat(sx127x_packet_format_t.SX127X_VARIABLE, 255));
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SX127X_MODE_RX_CONT, sx127x_modulation_t.SX127X_MODULATION_FSK));

		// it looks like some boards don't have RFO pin connected to the antenna
		tx.sx127x_tx_set_pa_config(new PaConfig(sx127x_pa_pin_t.SX127X_PA_PIN_BOOST, 4));
		tx.sx127x_fsk_ook_set_packet_format(new PacketFormat(sx127x_packet_format_t.SX127X_VARIABLE, 255));

		String small = LoraTest.createRandom(2);
		tx.sx127x_fsk_ook_tx_set_for_transmission(small);
		tx.tx(sx127x_modulation_t.SX127X_MODULATION_FSK);
		LoraTest.assertFrames(rx, small);

		String maxSingleBatch = LoraTest.createRandom(63);
		tx.sx127x_fsk_ook_tx_set_for_transmission(maxSingleBatch);
		tx.tx(sx127x_modulation_t.SX127X_MODULATION_FSK);
		LoraTest.assertFrames(rx, maxSingleBatch);

		String maxVariable = LoraTest.createRandom(255);
		tx.sx127x_fsk_ook_tx_set_for_transmission(maxVariable);
		tx.tx(sx127x_modulation_t.SX127X_MODULATION_FSK);
		LoraTest.assertFrames(rx, maxVariable);

	}

	@Test
	public void testReset() {
		rx.sx127x_set_frequency(frequency);
		assertEquals(frequency, rx.sx127x_get_frequency());
		rx.reset();
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SX127X_MODE_SLEEP, sx127x_modulation_t.SX127X_MODULATION_FSK));
		assertEquals(434000000, rx.sx127x_get_frequency());
		assertEquals(0, rx.sx127x_rx_get_lna_gain());
		assertEquals(4799, (int) rx.sx127x_fsk_ook_get_bitrate());
		assertEquals(5004, (int) rx.sx127x_fsk_get_fdev());
		assertEquals("55555555", rx.sx127x_fsk_ook_get_syncword());
		assertEquals(sx127x_packet_encoding_t.SX127X_NRZ, rx.sx127x_fsk_ook_get_packet_encoding());
		assertEquals(sx127x_crc_type_t.SX127X_CRC_CCITT, rx.sx127x_fsk_ook_get_crc());
		assertEquals(new AddressConfig(sx127x_address_filtering_t.SX127X_FILTER_NONE, 0, 0), rx.sx127x_fsk_ook_get_address_filtering());
		assertEquals(new PacketFormat(sx127x_packet_format_t.SX127X_VARIABLE, 64), rx.sx127x_fsk_ook_get_packet_format());
		assertEquals(new FskDataShaping(sx127x_fsk_data_shaping_t.SX127X_FSK_SHAPING_NONE, sx127x_pa_ramp_t.SX127X_PA_RAMP_10), rx.sx127x_fsk_get_data_shaping());
		assertEquals(sx127x_preamble_type_t.SX127X_PREAMBLE_AA, rx.sx127x_fsk_ook_get_preamble_type());
		assertFalse(rx.sx127x_fsk_ook_rx_get_afc_auto());
		assertEquals(50000, (int) rx.sx127x_fsk_ook_rx_get_afc_bandwidth());
		assertEquals(10416, (int) rx.sx127x_fsk_ook_rx_get_bandwidth());
		assertEquals(new RssiConfig(sx127x_rssi_smoothing_t.SX127X_8, 0), rx.sx127x_fsk_ook_rx_get_rssi_config());
		assertEquals(new CollisionConfig(false, 10), rx.sx127x_fsk_ook_rx_get_collision_restart());
		assertEquals(sx127x_rx_trigger_t.SX127X_RX_TRIGGER_NONE, rx.sx127x_fsk_ook_rx_get_trigger());
		assertEquals(new PreambleConfig(false, 3, 0), rx.sx127x_fsk_ook_rx_get_preamble_detector());
		assertEquals(3, rx.sx127x_get_preamble_length());
	}

	@Test
	public void testRegisters() {
		OpMode req = new OpMode(sx127x_mode_t.SX127X_MODE_SLEEP, sx127x_modulation_t.SX127X_MODULATION_FSK);
		rx.sx127x_set_opmod(req);
		assertEquals(req, rx.sx127x_get_opmod());
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SX127X_MODE_STANDBY, sx127x_modulation_t.SX127X_MODULATION_FSK));
		rx.sx127x_set_frequency(frequency);
		assertEquals(frequency, rx.sx127x_get_frequency());
		int gain = 4;
		rx.sx127x_rx_set_lna_gain(gain);
		assertEquals(gain, rx.sx127x_rx_get_lna_gain());
		float bitrate = 4800;
		rx.sx127x_fsk_ook_set_bitrate(bitrate);
		assertEquals(4800, (int) rx.sx127x_fsk_ook_get_bitrate());
		rx.sx127x_fsk_set_fdev(5000.0f);
		assertEquals(4943, (int) rx.sx127x_fsk_get_fdev());
		String syncword = "12AD";
		rx.sx127x_fsk_ook_set_syncword(syncword);
		assertEquals(syncword, rx.sx127x_fsk_ook_get_syncword());
		sx127x_packet_encoding_t encoding = sx127x_packet_encoding_t.SX127X_NRZ;
		rx.sx127x_fsk_ook_set_packet_encoding(encoding);
		assertEquals(encoding, rx.sx127x_fsk_ook_get_packet_encoding());
		sx127x_crc_type_t crc = sx127x_crc_type_t.SX127X_CRC_CCITT;
		rx.sx127x_fsk_ook_set_crc(crc);
		assertEquals(crc, rx.sx127x_fsk_ook_get_crc());
		AddressConfig address = new AddressConfig(sx127x_address_filtering_t.SX127X_FILTER_NODE_AND_BROADCAST, 0x11, 0x12);
		rx.sx127x_fsk_ook_set_address_filtering(address);
		assertEquals(address, rx.sx127x_fsk_ook_get_address_filtering());
		PacketFormat format = new PacketFormat(sx127x_packet_format_t.SX127X_VARIABLE, 255);
		rx.sx127x_fsk_ook_set_packet_format(format);
		assertEquals(format, rx.sx127x_fsk_ook_get_packet_format());
		format = new PacketFormat(sx127x_packet_format_t.SX127X_FIXED, 2047);
		rx.sx127x_fsk_ook_set_packet_format(format);
		assertEquals(format, rx.sx127x_fsk_ook_get_packet_format());
		FskDataShaping shaping = new FskDataShaping(sx127x_fsk_data_shaping_t.SX127X_BT_0_5, sx127x_pa_ramp_t.SX127X_PA_RAMP_10);
		rx.sx127x_fsk_set_data_shaping(shaping);
		assertEquals(shaping, rx.sx127x_fsk_get_data_shaping());
		rx.sx127x_fsk_ook_set_preamble_type(sx127x_preamble_type_t.SX127X_PREAMBLE_55);
		assertEquals(sx127x_preamble_type_t.SX127X_PREAMBLE_55, rx.sx127x_fsk_ook_get_preamble_type());
		rx.sx127x_fsk_ook_rx_set_afc_auto(true);
		assertTrue(rx.sx127x_fsk_ook_rx_get_afc_auto());
		float afcBandwidth = 20000.0f;
		rx.sx127x_fsk_ook_rx_set_afc_bandwidth(afcBandwidth);
		assertEquals(20833, (int) rx.sx127x_fsk_ook_rx_get_afc_bandwidth());
		float bandwidth = 5000.0f;
		rx.sx127x_fsk_ook_rx_set_bandwidth(bandwidth);
		assertEquals(5208, (int) rx.sx127x_fsk_ook_rx_get_bandwidth());
		RssiConfig rssi = new RssiConfig(sx127x_rssi_smoothing_t.SX127X_8, 0);
		rx.sx127x_fsk_ook_rx_set_rssi_config(rssi);
		assertEquals(rssi, rx.sx127x_fsk_ook_rx_get_rssi_config());
		CollisionConfig collision = new CollisionConfig(true, 10);
		rx.sx127x_fsk_ook_rx_set_collision_restart(collision);
		assertEquals(collision, rx.sx127x_fsk_ook_rx_get_collision_restart());
		rx.sx127x_fsk_ook_rx_set_trigger(sx127x_rx_trigger_t.SX127X_RX_TRIGGER_RSSI_PREAMBLE);
		assertEquals(sx127x_rx_trigger_t.SX127X_RX_TRIGGER_RSSI_PREAMBLE, rx.sx127x_fsk_ook_rx_get_trigger());
		PreambleConfig preamble = new PreambleConfig(true, 2, 0x0A);
		rx.sx127x_fsk_ook_rx_set_preamble_detector(preamble);
		assertEquals(preamble, rx.sx127x_fsk_ook_rx_get_preamble_detector());
	}

	@Before
	public void start() {
		rx.reset();
		rx.resetUart();
		tx.reset();
		tx.resetUart();
		OpMode req = new OpMode(sx127x_mode_t.SX127X_MODE_SLEEP, sx127x_modulation_t.SX127X_MODULATION_FSK);
		rx.sx127x_set_opmod(req);
		rx.sx127x_set_frequency(frequency);
		tx.sx127x_set_opmod(req);
		tx.sx127x_set_frequency(frequency);
	}

	@AfterClass
	public static void destroy() {
		if (rx != null) {
			rx.stop();
		}
		if (tx != null) {
			tx.stop();
		}
	}

}
