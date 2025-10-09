package com.leosatdata.sx127x;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

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
			freqStr = "434200000";
		}
		frequency = Long.valueOf(freqStr);
	}

	@Test
	public void testRegisters() {
		OpMode req = new OpMode(sx127x_mode_t.SX127X_MODE_SLEEP, sx127x_modulation_t.SX127X_MODULATION_LORA);
		rx.sx127x_set_opmod(req);
		assertEquals(req, rx.sx127x_get_opmod());
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SX127X_MODE_STANDBY, sx127x_modulation_t.SX127X_MODULATION_LORA));
		rx.sx127x_set_frequency(frequency);
		assertEquals(frequency, rx.sx127x_get_frequency());
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

		PaConfig pa = new PaConfig(sx127x_pa_pin_t.SX127X_PA_PIN_BOOST, 4);
		rx.sx127x_tx_set_pa_config(pa);
		assertEquals(pa, rx.sx127x_tx_get_pa_config());
		pa = new PaConfig(sx127x_pa_pin_t.SX127X_PA_PIN_RFO, -4);
		rx.sx127x_tx_set_pa_config(pa);
		assertEquals(pa, rx.sx127x_tx_get_pa_config());
		pa = new PaConfig(sx127x_pa_pin_t.SX127X_PA_PIN_RFO, 15);
		rx.sx127x_tx_set_pa_config(pa);
		assertEquals(pa, rx.sx127x_tx_get_pa_config());
		pa = new PaConfig(sx127x_pa_pin_t.SX127X_PA_PIN_BOOST, 2);
		rx.sx127x_tx_set_pa_config(pa);
		assertEquals(pa, rx.sx127x_tx_get_pa_config());
		pa = new PaConfig(sx127x_pa_pin_t.SX127X_PA_PIN_BOOST, 20);
		rx.sx127x_tx_set_pa_config(pa);
		assertEquals(pa, rx.sx127x_tx_get_pa_config());
		pa = new PaConfig(sx127x_pa_pin_t.SX127X_PA_PIN_BOOST, 17);
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
		sx127x_tx_header_t txHeader = new sx127x_tx_header_t(true, sx127x_cr_t.SX127X_CR_4_5);
		rx.sx127x_lora_tx_set_explicit_header(txHeader);
		assertEquals(txHeader, rx.sx127x_lora_tx_get_explicit_header());

		rx.sx127x_lora_set_ppm_offset(4000);
		assertEquals(3656, rx.sx127x_lora_get_ppm_offset());

		gain = 0;
		rx.sx127x_rx_set_lna_gain(gain);
		assertEquals(gain, rx.sx127x_rx_get_lna_gain());

		FhssConfig fhss = new FhssConfig(5, new long[] { 868200000L, 868250000L, 868700000L, 868200000L });
		rx.sx127x_lora_set_frequency_hopping(fhss);
		assertEquals(fhss, rx.sx127x_lora_get_frequency_hopping());

		int value = 0b01100111;
		int reg = 0x33;
		rx.sx127x_write_register(reg, value);
		assertEquals(value, rx.sx127x_read_register(reg));
	}

	@Test
	public void testReset() {
		assertEquals(frequency, rx.sx127x_get_frequency());
		rx.reset();
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SX127X_MODE_SLEEP, sx127x_modulation_t.SX127X_MODULATION_LORA));
		// make sure reset actually works
		assertEquals(434000000, rx.sx127x_get_frequency());
		assertEquals(125000, rx.sx127x_lora_get_bandwidth());
		assertEquals(7, rx.sx127x_lora_get_spreading_factor());
		assertEquals(0, rx.sx127x_rx_get_lna_gain());
		assertEquals(new PaConfig(sx127x_pa_pin_t.SX127X_PA_PIN_RFO, 13), rx.sx127x_tx_get_pa_config());
		assertEquals(18, rx.sx127x_lora_get_syncword());
		assertEquals(8, rx.sx127x_get_preamble_length());
		assertNull(rx.sx127x_lora_get_implicit_header());
	}

	@Test
	public void testExplicitHeader() {
		String message = createRandom(2);
		rx.sx127x_lora_set_implicit_header(null);
		rx.sx127x_lora_reset_fifo();
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SX127X_MODE_RX_CONT, sx127x_modulation_t.SX127X_MODULATION_LORA));

		sx127x_tx_header_t txHeader = new sx127x_tx_header_t(true, sx127x_cr_t.SX127X_CR_4_5);
		tx.sx127x_lora_tx_set_explicit_header(txHeader);
		// it looks like some boards don't have RFO pin connected to the antenna
		tx.sx127x_tx_set_pa_config(new PaConfig(sx127x_pa_pin_t.SX127X_PA_PIN_BOOST, 4));
		tx.sx127x_lora_reset_fifo();
		tx.sx127x_lora_tx_set_for_transmission(message);
		tx.tx(sx127x_modulation_t.SX127X_MODULATION_LORA);

		LoraTest.assertFrames(rx, message);

		// set is actually set, not append
		tx.sx127x_lora_tx_set_for_transmission(message);
		tx.sx127x_lora_tx_set_for_transmission(message);
		tx.tx(sx127x_modulation_t.SX127X_MODULATION_LORA);

		LoraTest.assertFrames(rx, message);
	}

	@Test
	public void testImplicitHeader() {
		String message = createRandom(2);
		sx127x_implicit_header_t header = new sx127x_implicit_header_t(message.length() / 2, true, sx127x_cr_t.SX127X_CR_4_5);

		rx.sx127x_lora_set_implicit_header(header);
		rx.sx127x_lora_reset_fifo();
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SX127X_MODE_RX_CONT, sx127x_modulation_t.SX127X_MODULATION_LORA));

		tx.sx127x_lora_set_implicit_header(header);
		// it looks like some boards don't have RFO pin connected to the antenna
		tx.sx127x_tx_set_pa_config(new PaConfig(sx127x_pa_pin_t.SX127X_PA_PIN_BOOST, 4));
		tx.sx127x_lora_reset_fifo();
		tx.sx127x_lora_tx_set_for_transmission(message);
		tx.tx(sx127x_modulation_t.SX127X_MODULATION_LORA);

		LoraTest.assertFrames(rx, message);
	}

	@Test
	public void testFhss() {
		long[] frequencies = new long[3];
		frequencies[0] = frequency + 500000;
		frequencies[1] = frequency + 1000000;
		frequencies[2] = frequency;
		String message = createRandom(2);
		rx.sx127x_lora_reset_fifo();
		rx.sx127x_lora_set_frequency_hopping(new FhssConfig(5, frequencies));
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SX127X_MODE_RX_CONT, sx127x_modulation_t.SX127X_MODULATION_LORA));

		// it looks like some boards don't have RFO pin connected to the antenna
		tx.sx127x_tx_set_pa_config(new PaConfig(sx127x_pa_pin_t.SX127X_PA_PIN_BOOST, 4));
		tx.sx127x_lora_reset_fifo();
		sx127x_tx_header_t txHeader = new sx127x_tx_header_t(true, sx127x_cr_t.SX127X_CR_4_5);
		tx.sx127x_lora_tx_set_explicit_header(txHeader);
		tx.sx127x_lora_set_frequency_hopping(new FhssConfig(5, frequencies));
		tx.sx127x_lora_tx_set_for_transmission(message);
		tx.tx(sx127x_modulation_t.SX127X_MODULATION_LORA);

		LoraTest.assertFrames(rx, message);
	}

	@Test
	public void testCad() {
		String message = createRandom(2);
		rx.sx127x_lora_set_implicit_header(null);
		rx.sx127x_lora_reset_fifo();
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SX127X_MODE_CAD, sx127x_modulation_t.SX127X_MODULATION_LORA));

		sx127x_tx_header_t txHeader = new sx127x_tx_header_t(true, sx127x_cr_t.SX127X_CR_4_5);
		tx.sx127x_lora_tx_set_explicit_header(txHeader);
		// it looks like some boards don't have RFO pin connected to the antenna
		tx.sx127x_tx_set_pa_config(new PaConfig(sx127x_pa_pin_t.SX127X_PA_PIN_BOOST, 4));
		tx.sx127x_lora_reset_fifo();
		tx.sx127x_lora_tx_set_for_transmission(message);
		tx.tx(sx127x_modulation_t.SX127X_MODULATION_LORA);

		LoraTest.assertFrames(rx, message);
	}

	@Test
	public void testDeepSleep() {
		String message = createRandom(2);
		rx.sx127x_lora_set_implicit_header(null);
		rx.sx127x_lora_reset_fifo();
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SX127X_MODE_RX_CONT, sx127x_modulation_t.SX127X_MODULATION_LORA));
		rx.deepSleep();

		sx127x_tx_header_t txHeader = new sx127x_tx_header_t(true, sx127x_cr_t.SX127X_CR_4_5);
		tx.sx127x_lora_tx_set_explicit_header(txHeader);
		// it looks like some boards don't have RFO pin connected to the antenna
		tx.sx127x_tx_set_pa_config(new PaConfig(sx127x_pa_pin_t.SX127X_PA_PIN_BOOST, 4));
		tx.sx127x_lora_reset_fifo();
		tx.sx127x_lora_tx_set_for_transmission(message);
		tx.tx(sx127x_modulation_t.SX127X_MODULATION_LORA);

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

		LoraTest.assertFrames(rx, message);
	}

	@Before
	public void start() {
		rx.reset();
		rx.resetUart();
		tx.reset();
		tx.resetUart();
		// make sure communication is happening on the configured frequency
		OpMode req = new OpMode(sx127x_mode_t.SX127X_MODE_SLEEP, sx127x_modulation_t.SX127X_MODULATION_LORA);
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

	public static void assertFrames(Sx127x device, String... framesStr) {
		List<sx127x_frame_t> frames = LoraTest.pullFrames(device, framesStr.length);
		assertEquals(framesStr.length, frames.size());
		for (int i = 0; i < framesStr.length; i++) {
			assertEquals(framesStr[i], frames.get(i).getMessage());
		}
	}

	public static List<sx127x_frame_t> pullFrames(Sx127x device, int expectedCount) {
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

	public static String createRandom(int length) {
		StringBuilder result = new StringBuilder();
		Random r = new Random(System.currentTimeMillis());
		for (int i = 0; i < length; i++) {
			result.append(String.format("%02x", (byte) (r.nextInt() % 255)));
		}
		return result.toString().toUpperCase();
	}

}
