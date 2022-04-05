package frc.robot.shooter;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Subsystem;
import frc.robot.config.ShooterVisionConfig;
import frc.robot.config.ShooterVisionConfig.ShooterVisionPoint;

public class ShooterVision implements Subsystem {

	public static final double MAX_VERTEX_X = 1.5;
	public static final double MAX_VERTEX_Y = 1.5;
	public static final double LIMELIGHT_FOV = 59.6 / 2 / 360;
	public static final double TARGET_SIZE = 4.5 / 2;

	private final NetworkTable table;

	private SlotConfiguration pid;

	private ShooterVisionPoint[] points;
	private double a;
	private double b;
	private double c;
	private double d;
	private double distanceOffset;

	private boolean hasTarget = false;
	private double[] target = new double[2];
	private double distance;
	private double yawCorrection;
	private boolean aligned;
	private double[] speeds = new double[2];

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
		return hasTarget;
	}

	public double[] getTarget() {
		return target;
	}

	public double getYawCorrection() {
		return yawCorrection;
	}

	public boolean isAligned() {
		return aligned;
	}

	public double[] getShooterSpeeds() {
		return speeds;
	}

	@Override
	public void tick() {
		long currentTime = System.currentTimeMillis();
		double timeDeltaSeconds = (currentTime - lastTime) / 1000d;
		lastTime = currentTime;

		if (table.getEntry("tv").getDouble(0) == 1) {
			double x;
			double y;
			if (table.getEntry("ta2").getDouble(0) != 0) {
				double x0 = table.getEntry("tx0").getDouble(0);
				double y0 = table.getEntry("ty0").getDouble(0);
				double x1 = table.getEntry("tx1").getDouble(0);
				double y1 = table.getEntry("ty1").getDouble(0);
				double x2 = table.getEntry("tx2").getDouble(0);
				double y2 = table.getEntry("ty2").getDouble(0);

				double d = (x0 - x1) * (x0 - x2) * (x1 - x2);
				double a = (x2 * (y1 - y0) + x1 * (y0 - y2) + x0 * (y2 - y1)) / d;
				double b = (x2 * x2 * (y0 - y1) + x1 * x1 * (y2 - y0) + x0 * x0 * (y1 - y2)) / d;
				double c = (x1 * x2 * (x1 - x2) * y0 + x2 * (x2 - x0) * y1 + x0 * x1 * (x0 - x1) * y2) / d;

				x = -b / (2 * a);
				y = c - b * b / (4 * a);

				if (!Double.isFinite(x) || !Double.isFinite(y) ||
					Math.abs(x) > MAX_VERTEX_X || Math.abs(y) > MAX_VERTEX_Y) {
					x = (x0 + x1) / 2;
					y = (y0 + y1) / 2;
				}
			} else if (table.getEntry("ta1").getDouble(0) != 0) {
				double x0 = table.getEntry("tx0").getDouble(0);
				double y0 = table.getEntry("ty0").getDouble(0);
				double x1 = table.getEntry("tx1").getDouble(0);
				double y1 = table.getEntry("ty1").getDouble(0);

				x = (x0 + x1) / 2;
				y = (y0 + y1) / 2;
			} else {
				x = table.getEntry("tx0").getDouble(0);
				y = table.getEntry("ty0").getDouble(0);
			}

			hasTarget = true;
			target[0] = x;
			target[1] = y;
			distance = a * y * y * y + b * y * y + c * y + d + distanceOffset;
			aligned = distance * Math.tan(Math.abs(x) * LIMELIGHT_FOV * Math.PI * 2) < TARGET_SIZE;

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
			speeds[0] = topSlope * distance + topIntercept;
			speeds[1] = bottomSlope * distance + bottomIntercept;
		} else {
			hasTarget = false;
			target[0] = 0;
			target[1] = 0;
			distance = 0;
			aligned = false;
			speeds[0] = 0;
			speeds[1] = 0;
		}

		double error = target[0];
		if (Math.abs(error) <= pid.integralZone) {
			iacc += error * timeDeltaSeconds;
			if (Math.abs(iacc) > pid.maxIntegralAccumulator)
				iacc = Math.signum(iacc) * pid.maxIntegralAccumulator;
		}
		else
			iacc = 0;
		yawCorrection = error * pid.kP + iacc * pid.kI + (error - lastErr) / timeDeltaSeconds * pid.kD;
		lastErr = error;
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
