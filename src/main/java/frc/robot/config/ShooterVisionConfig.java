package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

public class ShooterVisionConfig {

	public SlotConfiguration pid;

	public ShooterVisionPoint[] points;
	public double a;
	public double b;
	public double c;
	public double d;
	public double distanceOffset;

	public static class ShooterVisionPoint {

		public double distance;
		public double topSpeed;
		public double bottomSpeed;
	}
}
