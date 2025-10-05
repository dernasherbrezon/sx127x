package com.leosatdata.sx127x;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.regex.Pattern;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.fazecast.jSerialComm.SerialPort;
import com.fazecast.jSerialComm.SerialPortInvalidPortException;

public class Sx127x {

	private static final Logger LOG = LoggerFactory.getLogger(Sx127x.class);
	private static final Pattern COMMA = Pattern.compile(",");

	private final String portDescriptor;
	private final int timeout;
	private final String id;

	private SerialPort port;

	public Sx127x(String portDescriptor, int timeout, String id) {
		this.portDescriptor = portDescriptor;
		this.timeout = timeout;
		this.id = id;
	}

	public void start() {
		try {
			port = SerialPort.getCommPort(portDescriptor);
		} catch (SerialPortInvalidPortException e) {
			throw new RuntimeException("unable to send request: " + e.getMessage());
		}

		// this is important
		port.setComPortTimeouts(SerialPort.TIMEOUT_READ_SEMI_BLOCKING, timeout, timeout);
		// some defaults
		port.setBaudRate(115200);
		port.setParity(SerialPort.NO_PARITY);
		port.setNumDataBits(8);
		port.setNumStopBits(SerialPort.ONE_STOP_BIT);
		if (!port.openPort()) {
			throw new RuntimeException("can't open port: " + portDescriptor);
		}
		try {
			skipToStart();
		} catch (IOException e) {
			// ignore timeout
		}
	}

	public void stop() {
		if (port == null) {
			return;
		}
		if (!port.closePort()) {
			LOG.info("can't close the port");
		}
	}

	public void sx127x_set_opmod(OpMode mode) {
		sendRequest("AT+OPMOD=" + mode.getMode() + "," + mode.getModulation());
	}

	public OpMode sx127x_get_opmod() {
		String[] parts = COMMA.split(query("AT+OPMOD?"));
		return new OpMode(sx127x_mode_t.valueOf(parts[0]), sx127x_modulation_t.valueOf(parts[1]));
	}

	public void sx127x_set_frequency(long freq) {
		sendRequest("AT+FREQ=" + freq);
	}

	public long sx127x_get_frequency() {
		return Long.valueOf(query("AT+FREQ?"));
	}

	public void sx127x_lora_set_bandwidth(long bw) {
		sendRequest("AT+LBW=" + bw);
	}

	public long sx127x_lora_get_bandwidth() {
		return Long.valueOf(query("AT+LBW?"));
	}

	public void sx127x_lora_set_implicit_header(sx127x_implicit_header_t header) {
		if (header == null) {
			sendRequest("AT+IMPLHDR=");
			return;
		}
		sendRequest("AT+IMPLHDR=" + header.getLength() + "," + String.valueOf(header.isEnable_crc()).toUpperCase() + "," + header.getCoding_rate().getName());
	}

	public sx127x_implicit_header_t sx127x_lora_get_implicit_header() {
		String param = query("AT+IMPLHDR?");
		if (param.equalsIgnoreCase("DISABLED")) {
			return null;
		}
		String[] parts = COMMA.split(param);
		sx127x_implicit_header_t result = new sx127x_implicit_header_t();
		result.setLength(Integer.valueOf(parts[0]));
		result.setEnable_crc(Boolean.valueOf(parts[1].toLowerCase()));
		result.setCoding_rate(sx127x_cr_t.valueFromName(parts[2]));
		return result;
	}

	public void sx127x_lora_set_spreading_factor(int sf) {
		sendRequest("AT+SF=" + sf);
	}

	public int sx127x_lora_get_spreading_factor() {
		return Integer.valueOf(query("AT+SF?"));
	}

	public void sx127x_lora_set_syncword(int syncword) {
		sendRequest("AT+LORASW=" + syncword);
	}

	public int sx127x_lora_get_syncword() {
		return Integer.valueOf(query("AT+LORASW?"));
	}

	private String query(String request) {
		List<String> params = sendRequest(request);
		if (params.size() != 1) {
			throw new RuntimeException("cant get response for: " + request);
		}
		return params.get(0);
	}

	public void sx127x_set_preamble_length(int length) {
		sendRequest("AT+PLEN=" + length);
	}

	public int sx127x_get_preamble_length() {
		return Integer.valueOf(query("AT+PLEN?"));
	}

	public void sx127x_lora_set_low_datarate_optimization(boolean enable) {
		sendRequest("AT+LDO=" + String.valueOf(enable).toUpperCase());
	}

	public boolean sx127x_lora_get_low_datarate_optimization() {
		return Boolean.parseBoolean(query("AT+LDO?").toLowerCase());
	}

	public void sx127x_rx_set_lna_boost_hf(boolean enable) {
		sendRequest("AT+LNABOOST=" + String.valueOf(enable).toUpperCase());
	}

	public boolean sx127x_rx_get_lna_boost_hf() {
		return Boolean.parseBoolean(query("AT+LNABOOST?").toLowerCase());
	}

	public void sx127x_rx_set_lna_gain(int gain) {
		sendRequest("AT+GAIN=" + gain);
	}

	public int sx127x_rx_get_lna_gain() {
		return Integer.valueOf(query("AT+GAIN?"));
	}

	public void sx127x_tx_set_pa_config(PaConfig config) {
		sendRequest("AT+PA=" + config.getPin() + "," + config.getPower());
	}

	public void sx127x_tx_set_ocp(Integer milliamps) {
		if (milliamps == null) {
			sendRequest("AT+OCP=");
			return;
		}
		sendRequest("AT+OCP=" + milliamps);
	}

	public void reset() {
		sendRequest("AT+RESET=");
	}

	public Integer sx127x_tx_get_ocp() {
		String param = query("AT+OCP?");
		if (param.equalsIgnoreCase("DISABLED")) {
			return null;
		}
		return Integer.parseInt(param);
	}

	public PaConfig sx127x_tx_get_pa_config() {
		String[] parts = COMMA.split(query("AT+PA?"));
		PaConfig result = new PaConfig();
		result.setPin(sx127x_pa_pin_t.valueOf(parts[0]));
		result.setPower(Integer.parseInt(parts[1]));
		return result;
	}

	public void sx127x_lora_tx_set_explicit_header(sx127x_tx_header_t header) {
		sendRequest("AT+TXHDR=" + String.valueOf(header.isEnable_crc()).toUpperCase() + "," + header.getCoding_rate().getName());
	}

	public sx127x_tx_header_t sx127x_lora_tx_get_explicit_header() {
		String param = query("AT+TXHDR?");
		if (param.equalsIgnoreCase("DISABLED")) {
			return null;
		}
		String[] parts = COMMA.split(param);
		sx127x_tx_header_t result = new sx127x_tx_header_t();
		result.setEnable_crc(Boolean.parseBoolean(parts[0].toLowerCase()));
		result.setCoding_rate(sx127x_cr_t.valueFromName(parts[1]));
		return result;
	}

	public void sx127x_lora_set_ppm_offset(int offset) {
		sendRequest("AT+PPM=" + offset);
	}

	public int sx127x_lora_get_ppm_offset() {
		return Integer.parseInt(query("AT+PPM?"));
	}

	public void sx127x_lora_tx_set_for_transmission(String hexBytes) {
		sendRequest("AT+LORATX=" + hexBytes);
	}

	public void tx(sx127x_modulation_t modulation) {
		sendRequest("AT+TX=" + modulation);
	}

	public void sx127x_lora_reset_fifo() {
		sendRequest("AT+RSTFIFO=");
	}

	public List<sx127x_frame_t> pull() {
		List<String> data = sendRequest("AT+PULL?");
		List<sx127x_frame_t> result = new ArrayList<>();
		for (String cur : data) {
			String[] parts = COMMA.split(cur);

			sx127x_frame_t frame = new sx127x_frame_t();
			frame.setMessage(parts[0]);
			frame.setRssi(Integer.parseInt(parts[1]));
			frame.setSnr(Float.parseFloat(parts[2]));
			frame.setFrequency_error(Integer.parseInt(parts[3]));
			frame.setTimestamp(Long.valueOf(parts[4]));
			result.add(frame);
		}
		return result;
	}

	public void sx127x_lora_set_frequency_hopping(FhssConfig config) {
		StringBuilder req = new StringBuilder();
		for (int i = 0; i < config.getFrequencies().length; i++) {
			if (i != 0) {
				req.append(",");
			}
			req.append(config.getFrequencies()[i]);
		}
		sendRequest("AT+FREQHOP=" + config.getPeriod() + "," + req.toString());
	}

	public FhssConfig sx127x_lora_get_frequency_hopping() {
		String param = query("AT+FREQHOP?");
		String[] parts = COMMA.split(param);
		FhssConfig result = new FhssConfig();
		result.setPeriod(Integer.parseInt(parts[0]));
		if (parts.length > 1) {
			long[] frequencies = new long[parts.length - 1];
			for (int i = 0; i < frequencies.length; i++) {
				frequencies[i] = Long.valueOf(parts[i + 1]);
			}
			result.setFrequencies(frequencies);
		}
		return result;
	}

	public void sx127x_handle_interrupt() {
		sendRequest("AT+IRQ=");
	}

	public void deepSleep() {
		sendRequest("AT+DEEPSLEEP=");
	}

	private List<String> sendRequest(String request) {
		request = request + "\r\n";
		LOG.info("[{}] {}", id, request.trim());
		try {
			port.getOutputStream().write(request.getBytes(StandardCharsets.ISO_8859_1));
		} catch (IOException e) {
			if (!port.closePort()) {
				LOG.info("can't close the port");
			}
			throw new RuntimeException("unable to get status: " + e.getMessage());
		}
		try {
			return readResponse();
		} catch (IOException e) {
			throw new RuntimeException("unable to read status: " + e.getMessage());
		}
	}

	private List<String> readResponse() throws IOException {
		try (BufferedReader reader = new BufferedReader(new InputStreamReader(port.getInputStream(), StandardCharsets.ISO_8859_1))) {
			String curLine = null;
			List<String> result = new ArrayList<>();
			StringBuilder errorMessage = new StringBuilder();
			while ((curLine = reader.readLine()) != null) {
				curLine = curLine.trim();
				// discard corrupted serial communication
				if (curLine.length() == 0) {
					continue;
				}
				if (curLine.contains("[E]")) {
					LOG.error("[{}] {}", id, curLine);
				} else {
					LOG.info("[{}] {}", id, curLine);
				}
				// skip logging
				if (curLine.charAt(0) == '[') {
					continue;
				}
				if (curLine.equalsIgnoreCase("ERROR")) {
					throw new RuntimeException(errorMessage.toString());
				}
				if (curLine.equalsIgnoreCase("OK")) {
					return result;
				}
				// not clear yet if the response is valid or error message
				// update both
				if (errorMessage.length() > 0) {
					errorMessage.append(": ");
				}
				errorMessage.append(curLine);
				result.add(curLine);
			}
		}
		return Collections.emptyList();
	}

	private void skipToStart() throws IOException {
		InputStream inputStream = port.getInputStream();
		if (inputStream.available() == 0) {
			return;
		}
		try (BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream, StandardCharsets.ISO_8859_1))) {
			String curLine = null;
			while ((curLine = reader.readLine()) != null) {
				curLine = curLine.trim();
				LOG.info(curLine);
				// discard corrupted serial communication
				if (curLine.contains("Returned from app_main") || inputStream.available() == 0) {
					break;
				}
			}
		}
	}
}
