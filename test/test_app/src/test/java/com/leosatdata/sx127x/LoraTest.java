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
		pa = new PaConfig(sx127x_pa_pin_t.RFO, -4);
		rx.sx127x_tx_set_pa_config(pa);
		assertEquals(pa, rx.sx127x_tx_get_pa_config());
		pa = new PaConfig(sx127x_pa_pin_t.RFO, 15);
		rx.sx127x_tx_set_pa_config(pa);
		assertEquals(pa, rx.sx127x_tx_get_pa_config());
		pa = new PaConfig(sx127x_pa_pin_t.BOOST, 2);
		rx.sx127x_tx_set_pa_config(pa);
		assertEquals(pa, rx.sx127x_tx_get_pa_config());
		pa = new PaConfig(sx127x_pa_pin_t.BOOST, 20);
		rx.sx127x_tx_set_pa_config(pa);
		assertEquals(pa, rx.sx127x_tx_get_pa_config());
		pa = new PaConfig(sx127x_pa_pin_t.BOOST, 17);
		rx.sx127x_tx_set_pa_config(pa);
		assertEquals(pa, rx.sx127x_tx_get_pa_config());
		int ocp = 100;
		rx.sx127x_tx_set_ocp(ocp);
		assertEquals(ocp, rx.sx127x_tx_get_ocp().intValue());
		ocp = 150;
		rx.sx127x_tx_set_ocp(ocp);
		assertEquals(ocp, rx.sx127x_tx_get_ocp().intValue());
		rx.sx127x_tx_set_ocp(250);
		assertEquals(240, rx.sx127x_tx_get_ocp().intValue());
		rx.sx127x_tx_set_ocp(null);
		assertNull(rx.sx127x_tx_get_ocp());
		sx127x_tx_header_t txHeader = new sx127x_tx_header_t(true, sx127x_cr_t.SX127x_CR_4_5);
		rx.sx127x_lora_tx_set_explicit_header(txHeader);
		assertEquals(txHeader, rx.sx127x_lora_tx_get_explicit_header());

		rx.sx127x_lora_set_ppm_offset(4000);
		assertEquals(3655, rx.sx127x_lora_get_ppm_offset());

		gain = 0;
		rx.sx127x_rx_set_lna_gain(gain);
		assertEquals(gain, rx.sx127x_rx_get_lna_gain());
	}

	@AfterClass
	public static void destroy() {
		if (rx == null) {
			return;
		}
		rx.stop();
	}

}
