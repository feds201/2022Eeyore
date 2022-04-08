package frc.robot.shooter;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.ArrayPool;
import frc.robot.Subsystem;
import frc.robot.config.ShooterVisionConfig;
import frc.robot.config.ShooterVisionConfig.ShooterVisionPoint;

public class ShooterVision implements Subsystem {

	public static final double MAX_VERTEX_X = 1.5;
	public static final double MAX_VERTEX_Y = 1.5;
	public static final double MIN_X = -1.0;
	public static final double MAX_X = 0.9;
	public static final double MIN_Y = -0.5;
	public static final double MAX_Y = 1.0;

	public static final double MAX_DISTANCE = 0.5;
	public static final double MAX_DISTANCE_RATIO = 3.0;

	public static final double LIMELIGHT_FOV = 59.6 / 2 / 360;
	public static final double TARGET_SIZE = 4.5 / 1.5;

	private final NetworkTable table;

	private SlotConfiguration pid;

	private ShooterVisionPoint[] points;
	private double a;
	private double b;
	private double c;
	private double d;
	private double speedFactor;

	private List<double[]> contours = new ArrayList<>(7);
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

	public void adjustSpeedFactor(double factorDelta) {
		speedFactor += factorDelta;
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

		int possibleContours;
		if (table.getEntry("tv").getDouble(0) != 1)
			possibleContours = 0;
		else if (table.getEntry("ta6").getDouble(0) != 0)
			possibleContours = 7;
		else if (table.getEntry("ta5").getDouble(0) != 0)
			possibleContours = 6;
		else if (table.getEntry("ta4").getDouble(0) != 0)
			possibleContours = 5;
		else if (table.getEntry("ta3").getDouble(0) != 0)
			possibleContours = 4;
		else if (table.getEntry("ta2").getDouble(0) != 0)
			possibleContours = 3;
		else if (table.getEntry("ta1").getDouble(0) != 0)
			possibleContours = 2;
		else
			possibleContours = 1;

		for (int i = 0; i < possibleContours; i++) {
			double x = table.getEntry("tx" + i).getDouble(0);
			double y = table.getEntry("ty" + i).getDouble(0);
			if (x >= MIN_X && x <= MAX_X && y >= MIN_Y && y <= MAX_Y) {
				double[] array = ArrayPool.reserve(2);
				array[0] = x;
				array[1] = y;
				contours.add(array);
			}
		}

		if (contours.size() >= 2)
		{
			double[] minFirst = null;
			double[] minSecond = null;
			double minDistance = Double.POSITIVE_INFINITY;
			for (int i = 0; i < contours.size() - 1; i++) {
				for (int j = i + 1; j < contours.size(); j++) {
					double[] first = contours.get(i);
					double[] second = contours.get(j);
					double xDiff = first[0] - second[0];
					double yDiff = first[1] - second[1];
					double distance = Math.sqrt(xDiff * xDiff + yDiff * yDiff);
					if (distance < minDistance) {
						minFirst = first;
						minSecond = second;
						minDistance = distance;
					}
				}
			}

			if (minDistance > MAX_DISTANCE) {
				for (int i = 0; i < contours.size(); i++) {
					double[] array = contours.get(i);
					if (array != minFirst)
						ArrayPool.release(array);
				}
				contours.clear();
				if (minFirst != null)
					contours.add(minFirst);
			} else {
				for (int i = 0; i < contours.size(); i++) {
					double[] contour = contours.get(i);
					if (contour == minFirst || contour == minSecond)
						continue;
					double xDiff0 = minFirst[0] - contour[0];
					double yDiff0 = minFirst[1] - contour[1];
					double distance0 = Math.sqrt(xDiff0 * xDiff0 + yDiff0 * yDiff0);
					double xDiff1 = minSecond[0] - contour[0];
					double yDiff1 = minSecond[1] - contour[1];
					double distance1 = Math.sqrt(xDiff1 * xDiff1 + yDiff1 * yDiff1);
					if ((distance0 > minDistance * MAX_DISTANCE_RATIO &&
							distance1 > minDistance * MAX_DISTANCE_RATIO) ||
						(distance0 > MAX_DISTANCE && distance1 > MAX_DISTANCE)) {
						contours.remove(i);
						ArrayPool.release(contour);
						i--;
					}
				}
			}
		}

		if (contours.size() > 0) {
			double x;
			double y;
			if (contours.size() >= 3) {
				double x0 = contours.get(0)[0];
				double y0 = contours.get(0)[1];
				double x1 = contours.get(1)[0];
				double y1 = contours.get(1)[1];
				double x2 = contours.get(2)[0];
				double y2 = contours.get(2)[1];

				double d = (x0 - x1) * (x0 - x2) * (x1 - x2);
				double a = (x2 * (y1 - y0) + x1 * (y0 - y2) + x0 * (y2 - y1)) / d;
				double b = (x2 * x2 * (y0 - y1) + x1 * x1 * (y2 - y0) + x0 * x0 * (y1 - y2)) / d;
				double c = (x1 * x2 * (x1 - x2) * y0 + x2 * x0 * (x2 - x0) * y1 + x0 * x1 * (x0 - x1) * y2) / d;

				x = -b / (2 * a);
				y = c - b * b / (4 * a);

				if (!Double.isFinite(x) || !Double.isFinite(y) ||
					Math.abs(x) > MAX_VERTEX_X || Math.abs(y) > MAX_VERTEX_Y) {
					x = (x0 + x1) / 2;
					y = (y0 + y1) / 2;
				}
			} else if (contours.size() == 2) {
				double x0 = contours.get(0)[0];
				double y0 = contours.get(0)[1];
				double x1 = contours.get(1)[0];
				double y1 = contours.get(1)[1];

				x = (x0 + x1) / 2;
				y = (y0 + y1) / 2;
			} else {
				x = contours.get(0)[0];
				y = contours.get(0)[1];
			}

			for (int i = 0; i < contours.size(); i++)
				ArrayPool.release(contours.get(i));
			contours.clear();

			hasTarget = true;
			target[0] = x;
			target[1] = y;
			distance = a * y * y * y + b * y * y + c * y + d;
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
			speeds[0] = (topSlope * distance + topIntercept) * speedFactor;
			speeds[1] = (bottomSlope * distance + bottomIntercept) * speedFactor;
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
		speedFactor = config.speedFactor;

		lastTime = System.currentTimeMillis();
		iacc = 0;
		lastErr = 0;
	}
}
