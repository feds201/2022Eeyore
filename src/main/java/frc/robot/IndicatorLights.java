package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class IndicatorLights implements Subsystem {

	public static final int SLOW_PERIOD = 10;
	public static final int FAST_PERIOD = 5;

	private final AddressableLED strip;
	private final AddressableLEDBuffer buffer;

	private LEDPattern pattern = LEDPattern.SOLID;
	private int red = 0;
	private int green = 0;
	private int blue = 0;

	private int position = 0;
	private int time = 0;

	public IndicatorLights(int port, int count) {
		strip = new AddressableLED(port);
		strip.setLength(count);
		buffer = new AddressableLEDBuffer(count);
	}

	public void set(LEDPattern pattern, int red, int green, int blue) {
		if (red < 0 || red > 255 ||
			green < 0 || green > 255 ||
			blue < 0 || blue > 255)
			throw new IllegalArgumentException("color out of bounds");
		this.pattern = pattern;
		this.red = red;
		this.green = green;
		this.blue = blue;
		position = 0;
		time = 0;
	}

	@Override
	public void tick() {
		switch (pattern) {
		case SOLID:
			for (int i = 0; i < buffer.getLength(); i++)
				buffer.setRGB(i, red, green, blue);
			break;
		case BLINK:
			if (time <= 0) {
				if (position == 0) {
					for (int i = 0; i < buffer.getLength(); i++)
						buffer.setRGB(i, red, green, blue);
					position = 1;
				} else {
					for (int i = 0; i < buffer.getLength(); i++)
						buffer.setRGB(i, 0, 0, 0);
					position = 0;
				}
				time = SLOW_PERIOD;
			} else {
				time--;
			}
			break;
		case FORWARD:
			if (time <= 0) {
				for (int i = 0; i < buffer.getLength(); i++) {
					if ((i + position) % 4 == 0)
						buffer.setRGB(i, red, green, blue);
					else
						buffer.setRGB(i, 0, 0, 0);
				}
				position++;
				if (position >= 4)
					position = 0;
				time = SLOW_PERIOD;
			} else {
				time--;
			}
			break;
		case REVERSE:
			if (time <= 0) {
				for (int i = 0; i < buffer.getLength(); i++) {
					if ((i + position) % 4 == 0)
						buffer.setRGB(i, red, green, blue);
					else
						buffer.setRGB(i, 0, 0, 0);
				}
				position--;
				if (position <= 0)
					position = 4;
				time = SLOW_PERIOD;
			} else {
				time--;
			}
			break;
		case BLINK_FAST:
			if (time <= 0) {
				if (position == 0) {
					for (int i = 0; i < buffer.getLength(); i++)
						buffer.setRGB(i, red, green, blue);
					position = 1;
				} else {
					for (int i = 0; i < buffer.getLength(); i++)
						buffer.setRGB(i, 0, 0, 0);
					position = 0;
				}
				time = FAST_PERIOD;
			} else {
				time--;
			}
			break;
		case FORWARD_FAST:
			if (time <= 0) {
				for (int i = 0; i < buffer.getLength(); i++) {
					if ((i + position) % 6 == 0)
						buffer.setRGB(i, red, green, blue);
					else
						buffer.setRGB(i, 0, 0, 0);
				}
				position++;
				if (position >= 6)
					position = 0;
				time = FAST_PERIOD;
			} else {
				time--;
			}
			break;
		case REVERSE_FAST:
			if (time <= 0) {
				for (int i = 0; i < buffer.getLength(); i++) {
					if ((i + position) % 6 == 0)
						buffer.setRGB(i, red, green, blue);
					else
						buffer.setRGB(i, 0, 0, 0);
				}
				position--;
				if (position <= 0)
					position = 6;
				time = FAST_PERIOD;
			} else {
				time--;
			}
			break;
		}
		strip.setData(buffer);
		strip.start();
	}

	public static enum LEDPattern {
		SOLID, BLINK, FORWARD, REVERSE, BLINK_FAST, FORWARD_FAST, REVERSE_FAST
	}
}
