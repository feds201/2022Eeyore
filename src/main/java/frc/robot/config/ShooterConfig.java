package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PersistentException;

public class ShooterConfig {

	public SlotConfiguration pid;
	public double fireThresholdLower;
	public double fireThresholdUpper;
	public double feederSpeed;
	public boolean shooterBrake;

	public boolean shooterCurrentLimitEnabled;
	public double shooterCurrentLimit;
	public double shooterCurrentLimitTime;

	public boolean feederCurrentLimitEnabled;
	public double feederCurrentLimit;
	public double feederCurrentLimitTime;

	public static ShooterConfig load(String file) throws PersistentException {
		NetworkTable table = NetworkTableInstance.getDefault().getTable("config/shooter");
		ShooterConfig config = new ShooterConfig();
		table.loadEntries(file);

		config.pid = new SlotConfiguration();
		config.pid.closedLoopPeriod = (int) table.getEntry("pid.period").getDouble(1);
		config.pid.kP = table.getEntry("pid.kp").getDouble(0);
		config.pid.kI = table.getEntry("pid.ki").getDouble(0);
		config.pid.integralZone = table.getEntry("pid.izone").getDouble(0);
		config.pid.maxIntegralAccumulator = table.getEntry("pid.maxiacc").getDouble(0);
		config.pid.kD = table.getEntry("pid.kd").getDouble(0);
		config.pid.kF = table.getEntry("pid.kf").getDouble(0);

		config.shooterBrake = table.getEntry("shooterbrake").getBoolean(false);
		config.fireThresholdLower = table.getEntry("firethresholdlower").getDouble(0);
		config.fireThresholdUpper = table.getEntry("firethresholdupper").getDouble(2);
		config.feederSpeed = table.getEntry("feederspeed").getDouble(0);

		config.shooterCurrentLimitEnabled = table.getEntry("shootercurrentlimitenabled").getBoolean(false);
		config.shooterCurrentLimit = table.getEntry("shootercurrentlimit").getDouble(0);
		config.shooterCurrentLimitTime = table.getEntry("shootercurrentlimittime").getDouble(0);

		config.feederCurrentLimitEnabled = table.getEntry("feedercurrentlimitenabled").getBoolean(false);
		config.feederCurrentLimit = table.getEntry("feedercurrentlimit").getDouble(0);
		config.feederCurrentLimitTime = table.getEntry("feedercurrentlimittime").getDouble(0);

		return config;
	}
}
