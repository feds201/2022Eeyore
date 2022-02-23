package frc.robot;

import java.util.function.DoubleUnaryOperator;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.config.ShooterVisionConfig;
import frc.robot.config.ShooterVisionConfig.ShooterVisionPoint;

public class ShooterVision implements Subsystem {

	public static final DoubleUnaryOperator ANGLE_TO_DISTANCE = (x) -> 5.59 * Math.exp(-0.044 * x);

	private final NetworkTable table;

	private SlotConfiguration pid;

	private double lowGoalSpeedTop;
	private double lowGoalSpeedBottom;

	private ShooterVisionPoint[] points;
	private double distanceOffset;

	private double iacc = 0;
	private double lastErr = 0;
	private double lastTime;

	public ShooterVision(ShooterVisionConfig config) {
		table = NetworkTableInstance.getDefault().getTable("limelight");
		lastTime = System.currentTimeMillis();

		configure(config);
		setActive(false);
	}

	public void setActive(boolean active) {
		table.getEntry("ledMode").setDouble(active ? 3 : 1);
		table.getEntry("camMode").setDouble(active ? 0 : 1);
	}

	public boolean hasTarget() {
		return table.getEntry("tv").getDouble(0) == 1;
	}

	public double getYawCorrection() {
		double currentTime = System.currentTimeMillis();
		double delta = currentTime - lastTime;
		lastTime = currentTime;

		double error = table.getEntry("tx").getDouble(0);
		if (Math.abs(error) <= pid.integralZone) {
			iacc += error * delta;
			if (Math.abs(iacc) > pid.maxIntegralAccumulator)
				iacc = Math.signum(iacc) * pid.maxIntegralAccumulator;
		}
		else
			iacc = 0;
		double output = error * pid.kP + iacc * pid.kI + (lastErr - error) / delta * pid.kD;
		lastErr = error;
		return output;
	}

	public double[] getShooterSpeeds() {
		if (!hasTarget())
			return new double[] { lowGoalSpeedTop, lowGoalSpeedBottom };
		double distance = ANGLE_TO_DISTANCE.applyAsDouble(table.getEntry("ty").getDouble(0)) + distanceOffset;

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

		lowGoalSpeedTop = config.lowGoalSpeedTop;
		lowGoalSpeedBottom = config.lowGoalSpeedBottom;

		points = config.points;
		distanceOffset = config.distanceOffset;

		iacc = 0;
		lastErr = 0;
	}
}
