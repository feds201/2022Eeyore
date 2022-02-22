package frc.robot;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.config.ShooterVisionConfig;

public class ShooterVision implements Subsystem {

	private final NetworkTable table;

	private SlotConfiguration pid;
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

	public double getCorrection() {
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

	public void configure(ShooterVisionConfig config) {
		pid = config.pid;
		iacc = 0;
		lastErr = 0;
	}
}
