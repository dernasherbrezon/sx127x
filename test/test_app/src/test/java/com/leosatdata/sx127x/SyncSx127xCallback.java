package com.leosatdata.sx127x;

import java.util.ArrayList;
import java.util.List;

public class SyncSx127xCallback implements Sx127xCallback {

	private final List<sx127x_frame_t> frames = new ArrayList<>();

	@Override
	public synchronized void onFrame(sx127x_frame_t frame) {
		frames.add(frame);
		this.notifyAll();
	}

	public synchronized sx127x_frame_t waitForFrame() {
		try {
			this.wait(10000);
			if (frames.isEmpty()) {
				return null;
			}
			return frames.get(0);
		} catch (InterruptedException e) {
			Thread.currentThread().interrupt();
			return null;
		}
	}

	public void reset() {
		frames.clear();
	}

}
