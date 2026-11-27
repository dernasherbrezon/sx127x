package com.leosatdata.sx127x;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

public class Sx127xAsyncTest {

	private static Sx127xAsync rx;
	private static Sx127xAsync tx;
	private static long frequency;
	private static SyncSx127xCallback rxCallback = new SyncSx127xCallback();
	private static SyncSx127xCallback txCallback = new SyncSx127xCallback();

	@BeforeClass
	public static void init() {
		rx = new Sx127xAsync(System.getProperty("rx"), 10000, "rx", rxCallback);
		rx.start();
		tx = new Sx127xAsync(System.getProperty("tx"), 10000, "tx", txCallback);
		tx.start();

		String freqStr = System.getProperty("freq");
		if (freqStr == null) {
			freqStr = "434200000";
		}
		frequency = Long.valueOf(freqStr);
	}

	@Test
	public void testAsyncLora() {
		// sync operation
		OpMode req = new OpMode(sx127x_mode_t.SX127X_MODE_SLEEP, sx127x_modulation_t.SX127X_MODULATION_LORA);
		rx.sx127x_set_opmod(req);
		assertEquals(req, rx.sx127x_get_opmod());

		rx.sx127x_lora_set_implicit_header(null);
		rx.sx127x_lora_reset_fifo();
		rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SX127X_MODE_RX_CONT, sx127x_modulation_t.SX127X_MODULATION_LORA));
		String message = LoraTest.createRandom(2);
		sx127x_tx_header_t txHeader = new sx127x_tx_header_t(true, sx127x_cr_t.SX127X_CR_4_5);
		tx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SX127X_MODE_STANDBY, sx127x_modulation_t.SX127X_MODULATION_LORA));
		tx.sx127x_lora_tx_set_explicit_header(txHeader);
		tx.sx127x_lora_reset_fifo();
		tx.sx127x_tx_set_pa_config(new PaConfig(sx127x_pa_pin_t.SX127X_PA_PIN_BOOST, 4));
		tx.sx127x_lora_tx_set_for_transmission(message);
		tx.tx(sx127x_modulation_t.SX127X_MODULATION_LORA);

		sx127x_frame_t frame = rxCallback.waitForFrame();
		assertNotNull(frame);
		assertEquals(message, frame.getMessage());
	}

	@Test
	public void testAsyncFsk() {
		int[] messageSizes = new int[] { 6, 255, 2047 };
		for (int i = 0; i < messageSizes.length; i++) {
			String message = LoraTest.createRandom(messageSizes[i]);

			rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SX127X_MODE_SLEEP, sx127x_modulation_t.SX127X_MODULATION_FSK));
			rx.sx127x_fsk_ook_set_packet_format(new PacketFormat(sx127x_packet_format_t.SX127X_FIXED, messageSizes[i]));
			rx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SX127X_MODE_RX_CONT, sx127x_modulation_t.SX127X_MODULATION_FSK));

			tx.sx127x_set_opmod(new OpMode(sx127x_mode_t.SX127X_MODE_SLEEP, sx127x_modulation_t.SX127X_MODULATION_FSK));
			tx.sx127x_fsk_ook_set_packet_format(new PacketFormat(sx127x_packet_format_t.SX127X_FIXED, messageSizes[i]));
			tx.sx127x_tx_set_pa_config(new PaConfig(sx127x_pa_pin_t.SX127X_PA_PIN_BOOST, 4));
			tx.sx127x_fsk_ook_tx_set_for_transmission(message);
			tx.tx(sx127x_modulation_t.SX127X_MODULATION_FSK);

			sx127x_frame_t frame = rxCallback.waitForFrame();
			assertNotNull(frame);
			assertEquals(message, frame.getMessage());
			rxCallback.reset();
		}
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

	@Before
	public void start() {
		rx.reset();
		rx.resetUart();
		rxCallback.reset();
		tx.reset();
		tx.resetUart();
		txCallback.reset();
		// make sure communication is happening on the configured frequency
		OpMode req = new OpMode(sx127x_mode_t.SX127X_MODE_SLEEP, sx127x_modulation_t.SX127X_MODULATION_LORA);
		rx.sx127x_set_opmod(req);
		rx.sx127x_set_frequency(frequency);
		tx.sx127x_set_opmod(req);
		tx.sx127x_set_frequency(frequency);
	}
}
