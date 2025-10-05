package com.leosatdata.sx127x;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

public class LoraTest {

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
			freqStr = "868200012";
		}
		frequency = Long.valueOf(freqStr);
		rx.sx127x_set_frequency(frequency);
		tx.sx127x_set_frequency(frequency);
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

		FhssConfig fhss = new FhssConfig(5, new long[] { 868200000L, 868250000L, 868700000L, 868200000L });
		rx.sx127x_lora_set_frequency_hopping(fhss);
		assertEquals(fhss, rx.sx127x_lora_get_frequency_hopping());
	}

	@Test
	public void testExplicitHeader() {
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SLEEP, sx127x_modulation_t.LORA));
		rx.sx127x_lora_set_implicit_header(null);
		rx.sx127x_lora_reset_fifo();
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.RXCONT, sx127x_modulation_t.LORA));

		sendExplicitMessage();

		List<sx127x_frame_t> frames = pullFrames(rx, 1);
		assertEquals(1, frames.size());
		assertEquals("CAFE", frames.get(0).getMessage());
	}

	@Test
	public void testImplicitHeader() {
		OpMode req = new OpMode(sx127x_mode_t.SLEEP, sx127x_modulation_t.LORA);
		String message = "CAFE";
		sx127x_implicit_header_t header = new sx127x_implicit_header_t(message.length() / 2, true, sx127x_cr_t.SX127x_CR_4_5);

		rx.sx127x_set_opmod(req);
		rx.sx127x_lora_set_implicit_header(header);
		rx.sx127x_lora_reset_fifo();
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.RXCONT, sx127x_modulation_t.LORA));

		tx.sx127x_set_opmod(req);
		tx.sx127x_lora_set_implicit_header(header);
		// it looks like some boards don't have RFO pin connected to the antenna
		tx.sx127x_tx_set_pa_config(new PaConfig(sx127x_pa_pin_t.BOOST, 4));
		tx.sx127x_lora_reset_fifo();
		tx.sx127x_lora_tx_set_for_transmission(message);
		tx.tx(sx127x_modulation_t.LORA);

		List<sx127x_frame_t> frames = pullFrames(rx, 1);
		assertEquals(1, frames.size());
		assertEquals(message, frames.get(0).getMessage());
	}

	@Test
	public void testFhss() {
		long[] frequencies = new long[3];
		frequencies[0] = frequency + 500000;
		frequencies[1] = frequency + 1000000;
		frequencies[2] = frequency;
		OpMode req = new OpMode(sx127x_mode_t.SLEEP, sx127x_modulation_t.LORA);

		rx.sx127x_set_opmod(req);
		rx.sx127x_lora_set_implicit_header(null);
		rx.sx127x_lora_reset_fifo();
		rx.sx127x_lora_set_frequency_hopping(new FhssConfig(5, frequencies));
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.RXCONT, sx127x_modulation_t.LORA));

		tx.sx127x_set_opmod(req);
		tx.sx127x_lora_set_implicit_header(null);
		// it looks like some boards don't have RFO pin connected to the antenna
		tx.sx127x_tx_set_pa_config(new PaConfig(sx127x_pa_pin_t.BOOST, 4));
		tx.sx127x_lora_reset_fifo();
		tx.sx127x_lora_set_frequency_hopping(new FhssConfig(5, frequencies));
		tx.sx127x_lora_tx_set_for_transmission("CAFE");
		tx.tx(sx127x_modulation_t.LORA);
	}

	@Test
	public void testCad() {
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SLEEP, sx127x_modulation_t.LORA));
		rx.sx127x_lora_set_implicit_header(null);
		rx.sx127x_lora_reset_fifo();
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.CAD, sx127x_modulation_t.LORA));

		sendExplicitMessage();

		List<sx127x_frame_t> frames = pullFrames(rx, 1);
		assertEquals(1, frames.size());
		assertEquals("CAFE", frames.get(0).getMessage());
	}

	@Test
	public void testDeepSleep() {
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SLEEP, sx127x_modulation_t.LORA));
		rx.sx127x_lora_set_implicit_header(null);
		rx.sx127x_lora_reset_fifo();
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.RXCONT, sx127x_modulation_t.LORA));
		rx.deepSleep();

		sendExplicitMessage();

		// rx need some time to wake up
		int attempts = 3;
		while (attempts > 0) {
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e1) {
				Thread.currentThread().interrupt();
				return;
			}
			try {
				rx.sx127x_handle_interrupt();
				break;
			} catch (Exception e) {
				attempts--;
			}
		}

		List<sx127x_frame_t> frames = pullFrames(rx, 1);
		assertEquals(1, frames.size());
		assertEquals("CAFE", frames.get(0).getMessage());
	}

	@Before
	public void start() {
		rx.reset();
		tx.reset();
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

	private static void sendExplicitMessage() {
		tx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SLEEP, sx127x_modulation_t.LORA));
		sx127x_tx_header_t txHeader = new sx127x_tx_header_t(true, sx127x_cr_t.SX127x_CR_4_5);
		tx.sx127x_lora_tx_set_explicit_header(txHeader);
		// it looks like some boards don't have RFO pin connected to the antenna
		tx.sx127x_tx_set_pa_config(new PaConfig(sx127x_pa_pin_t.BOOST, 4));
		tx.sx127x_lora_reset_fifo();
		tx.sx127x_lora_tx_set_for_transmission("CAFE");
		tx.tx(sx127x_modulation_t.LORA);
	}

	private static List<sx127x_frame_t> pullFrames(Sx127x device, int expectedCount) {
		List<sx127x_frame_t> result = new ArrayList<>();
		long totalWait = 5000;
		long currentWait = 0;
		long period = 1000;
		while (result.size() < expectedCount && currentWait < totalWait) {
			try {
				Thread.sleep(period);
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt();
				break;
			}
			currentWait += period;
			result.addAll(device.pull());
		}
		return result;
	}

}
