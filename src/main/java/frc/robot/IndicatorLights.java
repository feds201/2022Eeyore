package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class IndicatorLights implements Subsystem {

	private final int count;
	private final AddressableLED strip;
	private final AddressableLEDBuffer buffer;
	private final HashMap<LEDZone, ZoneController> zones;

	private int position = 0;

	public IndicatorLights(int port, int count) {
		this.count = count;
		strip = new AddressableLED(port);
		strip.setLength(count);
		buffer = new AddressableLEDBuffer(count);

		zones = new HashMap<>();
		zones.put(LEDZone.BASE, new ZoneController(count));
		zones.put(LEDZone.LEFT, new ZoneController(60));
		zones.put(LEDZone.RIGHT, new ZoneController(60));
		zones.put(LEDZone.ACCENT, new ZoneController(40));
		zones.put(LEDZone.TIPS, new ZoneController(30));
		zones.put(LEDZone.TOP, new ZoneController(15));
		zones.put(LEDZone.BOTTOM, new ZoneController(15));
		zones.put(LEDZone.CENTER, new ZoneController(10));
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
		position++;

		for (LEDZone zone : LEDZone.values())
			zones.get(zone).tick();

		ZoneController zone = zones.get(LEDZone.BASE);
		if (!zone.isPassthrough()) {
			Color[] zoneBuffer = zone.getBuffer();
			for (int i = 0; i < count; i++)
				buffer.setLED(i, zoneBuffer[i]);
		}
		zone = zones.get(LEDZone.LEFT);
		if (!zone.isPassthrough()) {
			Color[] zoneBuffer = zone.getBuffer();
			for (int i = 0; i < 60; i++)
				buffer.setLED(i, zoneBuffer[i]);
		}
		zone = zones.get(LEDZone.RIGHT);
		if (!zone.isPassthrough()) {
			Color[] zoneBuffer = zone.getBuffer();
			for (int i = 0; i < 60; i++)
				buffer.setLED(i + (count - 60), zoneBuffer[i]);
		}
		zone = zones.get(LEDZone.ACCENT);
		if (!zone.isPassthrough()) {
			Color[] zoneBuffer = zone.getBuffer();
			for (int i = 0; i < 15; i++)
				buffer.setLED(i, zoneBuffer[i]);
			for (int i = 15; i < 25; i++)
				buffer.setLED(i + (count - 80), zoneBuffer[i]);
			for (int i = 25; i < 40; i++)
				buffer.setLED(i + (count - 40), zoneBuffer[i]);
		}
		zone = zones.get(LEDZone.TIPS);
		if (!zone.isPassthrough()) {
			Color[] zoneBuffer = zone.getBuffer();
			for (int i = 0; i < 15; i++)
				buffer.setLED(i, zoneBuffer[i]);
			for (int i = 15; i < 30; i++)
				buffer.setLED(i + (count - 30), zoneBuffer[i]);
		}
		zone = zones.get(LEDZone.TOP);
		if (!zone.isPassthrough()) {
			Color[] zoneBuffer = zone.getBuffer();
			for (int i = 0; i < 15; i++)
				buffer.setLED(i, zoneBuffer[i]);
		}
		zone = zones.get(LEDZone.BOTTOM);
		if (!zone.isPassthrough()) {
			Color[] zoneBuffer = zone.getBuffer();
			for (int i = 0; i < 15; i++)
				buffer.setLED(i + (count - 15), zoneBuffer[i]);
		}
		zone = zones.get(LEDZone.CENTER);
		if (!zone.isPassthrough()) {
			Color[] zoneBuffer = zone.getBuffer();
			for (int i = 0; i < 10; i++)
				buffer.setLED(i + (count - 65), zoneBuffer[i]);
		}

		strip.setData(buffer);
		strip.start();
	}

	public static enum LEDPattern {
		PASS, SOLID, BLINK, FORWARD, REVERSE, RAINBOW
	}

	public static enum LEDZone {
		BASE, LEFT, RIGHT, ACCENT, TIPS, TOP, BOTTOM, CENTER
	}

	private class ZoneController {

		private final Color[] buffer;

		private LEDPattern pattern = LEDPattern.PASS;
		private Color color = Color.kBlack;

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
				if ((position / 10) % 2 == 0) {
					for (int i = 0; i < buffer.length; i++)
						buffer[i] = color;
				} else {
					for (int i = 0; i < buffer.length; i++)
						buffer[i] = Color.kBlack;
				}
				break;
			case FORWARD:
				for (int i = 0; i < buffer.length; i++) {
					if ((i + position / 10) % 4 == 0)
						buffer[i] = color;
					else
						buffer[i] = Color.kBlack;
				}
				break;
			case REVERSE:
				for (int i = 0; i < buffer.length; i++) {
					if ((i - position / 10) % 4 == 0)
						buffer[i] = color;
					else
						buffer[i] = Color.kBlack;
				}
				break;
			case RAINBOW:
				for (int i = 0; i < buffer.length; i++) {
					if ((i + position) % 14 <= 1)
						buffer[i] = Color.kRed;
					else if ((i + position) % 14 <= 3)
						buffer[i] = Color.kOrange;
					else if ((i + position) % 14 <= 5)
						buffer[i] = Color.kYellow;
					else if ((i + position) % 14 <= 7)
						buffer[i] = Color.kGreen;
					else if ((i + position) % 14 <= 9)
						buffer[i] = Color.kBlue;
					else if ((i + position) % 14 <= 11)
						buffer[i] = Color.kBlueViolet;
					else
						buffer[i] = Color.kViolet;
				}
				break;
			}
		}
	}
}
