package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PersistentException;

public class ShooterConfig {

	public SlotConfiguration topPid;
	public SlotConfiguration bottomPid;
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
		NetworkTable table = NetworkTableInstance.getDefault().getTable("/config/shooter");
		ShooterConfig config = new ShooterConfig();
		table.loadEntries(file);

		config.topPid = new SlotConfiguration();
		config.topPid.closedLoopPeriod = (int) table.getEntry("toppid.period").getDouble(1);
		config.topPid.kP = table.getEntry("toppid.kp").getDouble(0);
		config.topPid.kI = table.getEntry("toppid.ki").getDouble(0);
		config.topPid.integralZone = table.getEntry("toppid.izone").getDouble(0);
		config.topPid.maxIntegralAccumulator = table.getEntry("toppid.maxiacc").getDouble(0);
		config.topPid.kD = table.getEntry("toppid.kd").getDouble(0);
		config.topPid.kF = table.getEntry("toppid.kf").getDouble(0);

		config.bottomPid = new SlotConfiguration();
		config.bottomPid.closedLoopPeriod = (int) table.getEntry("bottompid.period").getDouble(1);
		config.bottomPid.kP = table.getEntry("bottompid.kp").getDouble(0);
		config.bottomPid.kI = table.getEntry("bottompid.ki").getDouble(0);
		config.bottomPid.integralZone = table.getEntry("bottompid.izone").getDouble(0);
		config.bottomPid.maxIntegralAccumulator = table.getEntry("bottompid.maxiacc").getDouble(0);
		config.bottomPid.kD = table.getEntry("bottompid.kd").getDouble(0);
		config.bottomPid.kF = table.getEntry("bottompid.kf").getDouble(0);

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
