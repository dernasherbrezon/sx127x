package com.leosatdata.sx127x;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.AfterClass;
import org.junit.BeforeClass;
import org.junit.Test;

public class LoraTest {

	private static Sx127x rx;

	@BeforeClass
	public static void init() {
		rx = new Sx127x(System.getProperty("rx"), 10000);
		rx.start();
	}

	@Test
	public void testRegisters() {
		OpMode req = new OpMode(sx127x_mode_t.SLEEP, sx127x_modulation_t.LORA);
		rx.sx127x_set_opmod(req);
		assertEquals(req, rx.sx127x_get_opmod());
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.STANDBY, sx127x_modulation_t.LORA));
		long freq = 868200012L;
		rx.sx127x_set_frequency(freq);
		assertEquals(868200000L, rx.sx127x_get_frequency());
		long bw = 125000;
		rx.sx127x_lora_set_bandwidth(bw);
		assertEquals(bw, rx.sx127x_lora_get_bandwidth());
		rx.sx127x_lora_set_implicit_header(null);
		assertNull(rx.sx127x_lora_get_implicit_header());
		int sf = 9;
		rx.sx127x_lora_set_spreading_factor(sf);
		assertEquals(sf, rx.sx127x_lora_get_spreading_factor());
		int syncword = 18;
		rx.sx127x_lora_set_syncword(syncword);
		assertEquals(syncword, rx.sx127x_lora_get_syncword());
		int preambleLength = 8;
		rx.sx127x_set_preamble_length(preambleLength);
		assertEquals(preambleLength, rx.sx127x_get_preamble_length());
		rx.sx127x_lora_set_low_datarate_optimization(true);
		assertTrue(rx.sx127x_lora_get_low_datarate_optimization());
		rx.sx127x_rx_set_lna_boost_hf(true);
		assertTrue(rx.sx127x_rx_get_lna_boost_hf());
		int gain = 4;
		rx.sx127x_rx_set_lna_gain(gain);
		assertEquals(gain, rx.sx127x_rx_get_lna_gain());
		PaConfig pa = new PaConfig(sx127x_pa_pin_t.BOOST, 4);
		rx.sx127x_tx_set_pa_config(pa);
		assertEquals(pa, rx.sx127x_tx_get_pa_config());
	}

	@AfterClass
	public static void destroy() {
		if (rx == null) {
			return;
		}
		rx.stop();
	}

}
