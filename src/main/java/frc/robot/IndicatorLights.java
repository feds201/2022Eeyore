package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class IndicatorLights implements Subsystem {

	public static final int PERIOD = 5;

	private final int count;
	private final AddressableLED strip;
	private final AddressableLEDBuffer buffer;
	private final HashMap<LEDZone, ZoneController> zones;

	private int position = 0;
	private int time = 0;

	public IndicatorLights(int port, int count) {
		this.count = count;
		strip = new AddressableLED(port);
		strip.setLength(count);
		buffer = new AddressableLEDBuffer(count);

		zones = new HashMap<>();
		zones.put(LEDZone.BASE, new ZoneController(60));
		zones.put(LEDZone.TIPS, new ZoneController(10));
		zones.put(LEDZone.TOP, new ZoneController(5));
		zones.put(LEDZone.BOTTOM, new ZoneController(5));
	}

	public void set(LEDZone zone, LEDPattern pattern, Color color) {
		if (zone == null)
			throw new IllegalArgumentException("zone is null");
		if (pattern == null)
			throw new IllegalArgumentException("pattern is null");
		zones.get(zone).set(pattern, color != null ? color : Color.kBlack);
	}

	@Override
	public void tick() {
		time--;
		if (time <= 0) {
			position++;
			time = PERIOD;
		}

		for (LEDZone zone : LEDZone.values())
			zones.get(zone).tick();

		ZoneController zone = zones.get(LEDZone.BASE);
		if (!zone.isPassthrough()) {
			Color[] zoneBuffer = zone.getBuffer();
			for (int i = 0; i < count; i++)
				buffer.setLED(i, zoneBuffer[i]);
		}
		zone = zones.get(LEDZone.TIPS);
		if (!zone.isPassthrough()) {
			Color[] zoneBuffer = zone.getBuffer();
			for (int i = 0; i < 5; i++)
				buffer.setLED(i, zoneBuffer[i]);
			for (int i = 5; i < 10; i++)
				buffer.setLED(i + (count - 10), zoneBuffer[i]);
		}
		zone = zones.get(LEDZone.TOP);
		if (!zone.isPassthrough()) {
			Color[] zoneBuffer = zone.getBuffer();
			for (int i = 0; i < 5; i++)
				buffer.setLED(i, zoneBuffer[i]);
		}
		zone = zones.get(LEDZone.BOTTOM);
		if (!zone.isPassthrough()) {
			Color[] zoneBuffer = zone.getBuffer();
			for (int i = 0; i < 5; i++)
				buffer.setLED(i + (count - 5), zoneBuffer[i]);
		}

		strip.setData(buffer);
		strip.start();
	}

	public static enum LEDPattern {
		PASS, SOLID, BLINK, FORWARD, REVERSE
	}

	public static enum LEDZone {
		BASE, TIPS, TOP, BOTTOM
	}

	private class ZoneController {

		private final Color[] buffer;

		private LEDPattern pattern;
		private Color color;

		public ZoneController(int size) {
			buffer = new Color[size];
		}

		public void set(LEDPattern pattern, Color color) {
			this.pattern = pattern;
			this.color = color;
		}

		public boolean isPassthrough() {
			return pattern == LEDPattern.PASS;
		}

		public Color[] getBuffer() {
			return buffer;
		}

		public void tick() {
			switch (pattern) {
			case PASS:
				break;
			case SOLID:
				for (int i = 0; i < buffer.length; i++)
					buffer[i] = color;
				break;
			case BLINK:
				if (position % 2 == 0) {
					for (int i = 0; i < buffer.length; i++)
						buffer[i] = color;
				} else {
					for (int i = 0; i < buffer.length; i++)
						buffer[i] = Color.kBlack;
				}
				break;
			case FORWARD:
				for (int i = 0; i < buffer.length; i++) {
					if ((i + position) % 4 == 0)
						buffer[i] = color;
					else
						buffer[i] = Color.kBlack;
				}
				break;
			case REVERSE:
				for (int i = 0; i < buffer.length; i++) {
					if ((i + position) % 4 == 0)
						buffer[i] = color;
					else
						buffer[i] = Color.kBlack;
				}
				break;
			}
		}
	}
}
