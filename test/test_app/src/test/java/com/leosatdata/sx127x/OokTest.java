package com.leosatdata.sx127x;

import static org.junit.Assert.assertEquals;

import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

public class OokTest {

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
			freqStr = "868200000";
		}
		frequency = Long.valueOf(freqStr);
		OpMode req = new OpMode(sx127x_mode_t.SX127X_MODE_SLEEP, sx127x_modulation_t.SX127X_MODULATION_OOK);
		rx.sx127x_set_opmod(req);
		rx.sx127x_set_frequency(frequency);
		tx.sx127x_set_opmod(req);
		tx.sx127x_set_frequency(frequency);
	}

	@Test
	public void testRegisters() {
		OpMode req = new OpMode(sx127x_mode_t.SX127X_MODE_SLEEP, sx127x_modulation_t.SX127X_MODULATION_OOK);
		rx.sx127x_set_opmod(req);
		assertEquals(req, rx.sx127x_get_opmod());
		float bitrate = 4800;
		rx.sx127x_fsk_ook_set_bitrate(bitrate);
		assertEquals(4800, (int) rx.sx127x_fsk_ook_get_bitrate());
		OokDataShaping shaping = new OokDataShaping(sx127x_ook_data_shaping_t.SX127X_1_BIT_RATE, sx127x_pa_ramp_t.SX127X_PA_RAMP_10);
		rx.sx127x_ook_set_data_shaping(shaping);
		assertEquals(shaping, rx.sx127x_ook_get_data_shaping());
		PeakMode peak = new PeakMode(sx127x_ook_peak_thresh_step_t.SX127X_0_5_DB, 0x0c, sx127x_ook_peak_thresh_dec_t.SX127X_1_1_CHIP);
		rx.sx127x_ook_rx_set_peak_mode(peak);
		assertEquals(peak, rx.sx127x_ook_rx_get_peak_mode());
		assertEquals(sx127x_ook_thresh_type_t.SX127X_OOK_PEAK, rx.sx127x_ook_get_ook_thresh_type());
		int fixedThreshold = 0x11;
		rx.sx127x_ook_rx_set_fixed_mode(fixedThreshold);
		assertEquals(fixedThreshold, rx.sx127x_ook_rx_get_fixed_mode());
		assertEquals(sx127x_ook_thresh_type_t.SX127X_OOK_FIXED, rx.sx127x_ook_get_ook_thresh_type());
		AvgMode avg = new AvgMode(sx127x_ook_avg_offset_t.SX127X_2_DB, sx127x_ook_avg_thresh_t.SX127X_4_PI);
		rx.sx127x_ook_rx_set_avg_mode(avg);
		assertEquals(avg, rx.sx127x_ook_rx_get_avg_mode());
		assertEquals(sx127x_ook_thresh_type_t.SX127X_OOK_AVG, rx.sx127x_ook_get_ook_thresh_type());
	}

	@Test
	public void testReset() {
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SX127X_MODE_SLEEP, sx127x_modulation_t.SX127X_MODULATION_OOK));
		rx.sx127x_set_frequency(frequency);
		assertEquals(frequency, rx.sx127x_get_frequency());
		rx.reset();
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SX127X_MODE_SLEEP, sx127x_modulation_t.SX127X_MODULATION_OOK));
		assertEquals(434000000, rx.sx127x_get_frequency());
		assertEquals(new OokDataShaping(sx127x_ook_data_shaping_t.SX127X_OOK_SHAPING_NONE, sx127x_pa_ramp_t.SX127X_PA_RAMP_10), rx.sx127x_ook_get_data_shaping());
		assertEquals(sx127x_ook_thresh_type_t.SX127X_OOK_PEAK, rx.sx127x_ook_get_ook_thresh_type());
		assertEquals(new PeakMode(sx127x_ook_peak_thresh_step_t.SX127X_0_5_DB, 12, sx127x_ook_peak_thresh_dec_t.SX127X_1_1_CHIP), rx.sx127x_ook_rx_get_peak_mode());
	}

	@Test
	public void testVariableLength() {
		rx.sx127x_fsk_ook_set_packet_format(new PacketFormat(sx127x_packet_format_t.SX127X_VARIABLE, 255));
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SX127X_MODE_RX_CONT, sx127x_modulation_t.SX127X_MODULATION_OOK));

		tx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SX127X_MODE_SLEEP, sx127x_modulation_t.SX127X_MODULATION_OOK));
		// it looks like some boards don't have RFO pin connected to the antenna
		tx.sx127x_tx_set_pa_config(new PaConfig(sx127x_pa_pin_t.SX127X_PA_PIN_BOOST, 4));
		tx.sx127x_fsk_ook_set_packet_format(new PacketFormat(sx127x_packet_format_t.SX127X_VARIABLE, 255));

		String small = LoraTest.createRandom(2);
		tx.sx127x_fsk_ook_tx_set_for_transmission(small);
		tx.tx(sx127x_modulation_t.SX127X_MODULATION_OOK);
		LoraTest.assertFrames(rx, small);

		String maxSingleBatch = LoraTest.createRandom(63);
		tx.sx127x_fsk_ook_tx_set_for_transmission(maxSingleBatch);
		tx.tx(sx127x_modulation_t.SX127X_MODULATION_OOK);
		LoraTest.assertFrames(rx, maxSingleBatch);
	}

	@Before
	public void start() {
		rx.reset();
		rx.resetUart();
		tx.reset();
		tx.resetUart();
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
