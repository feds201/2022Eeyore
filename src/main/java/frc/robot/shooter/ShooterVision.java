package frc.robot.shooter;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Subsystem;
import frc.robot.config.ShooterVisionConfig;
import frc.robot.config.ShooterVisionConfig.ShooterVisionPoint;

public class ShooterVision implements Subsystem {

	private final NetworkTable table;

	private SlotConfiguration pid;

	private ShooterVisionPoint[] points;
	private double a;
	private double b;
	private double c;
	private double d;
	private double distanceOffset;

	private long lastTime;
	private double iacc = 0;
	private double lastErr = 0;

	public ShooterVision(ShooterVisionConfig config) {
		table = NetworkTableInstance.getDefault().getTable("limelight");

		configure(config);
		setActive(false);
	}

	public void setActive(boolean active) {
		table.getEntry("ledMode").setDouble(active ? 3 : 0);
	}

	public void adjustDistance(int offsetDelta) {
		distanceOffset += offsetDelta;
	}

	public boolean hasTarget() {
		return table.getEntry("tv").getDouble(0) == 1;
	}

	public double getYawCorrection() {
		long currentTime = System.currentTimeMillis();
		double timeDeltaSeconds = (currentTime - lastTime) / 1000d;
		lastTime = currentTime;

		double error = table.getEntry("tx").getDouble(0);
		if (Math.abs(error) <= pid.integralZone) {
			iacc += error * timeDeltaSeconds;
			if (Math.abs(iacc) > pid.maxIntegralAccumulator)
				iacc = Math.signum(iacc) * pid.maxIntegralAccumulator;
		}
		else
			iacc = 0;
		double output = error * pid.kP + iacc * pid.kI + (lastErr - error) / timeDeltaSeconds * pid.kD;
		lastErr = error;
		return output;
	}

	public double[] getShooterSpeeds() {
		if (!hasTarget())
			return new double[] { 0, 0 };

		double angle = table.getEntry("ty").getDouble(0);
		double distance = a * angle * angle * angle + b * angle * angle + c * angle + d + distanceOffset;

		ShooterVisionPoint lowPoint = points[0];
		ShooterVisionPoint highPoint = points[1];
		for (int i = 2; i < points.length; i++) {
			if (distance < highPoint.distance)
				break;
			lowPoint = highPoint;
			highPoint = points[i];
		}

		double topSlope = (highPoint.topSpeed - lowPoint.topSpeed) / (highPoint.distance - lowPoint.distance);
		double bottomSlope = (highPoint.bottomSpeed - lowPoint.bottomSpeed) / (highPoint.distance - lowPoint.distance);
		double topIntercept = highPoint.topSpeed - topSlope * highPoint.distance;
		double bottomIntercept = highPoint.bottomSpeed - bottomSlope * highPoint.distance;
		double topSpeed = topSlope * distance + topIntercept;
		double bottomSpeed = bottomSlope * distance + bottomIntercept;

		return new double[] { topSpeed, bottomSpeed };
	}

	public void configure(ShooterVisionConfig config) {
		pid = config.pid;

		points = config.points;
		a = config.a;
		b = config.b;
		c = config.c;
		d = config.d;
		distanceOffset = config.distanceOffset;

		lastTime = System.currentTimeMillis();
		iacc = 0;
		lastErr = 0;
	}
}
