package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PersistentException;

public class SwerveDriveConfig {

	public SwerveModuleConfig moduleConfig;
	public double gyroFactor;

	public double width;
	public double length;
	public double wheelDistance;

	public double maxLinearAccel;
	public double maxRotateAccel;

	public static SwerveDriveConfig load(String file) throws PersistentException {
		NetworkTable table = NetworkTableInstance.getDefault().getTable("/config/swerve");
		SwerveDriveConfig config = new SwerveDriveConfig();
		table.loadEntries(file);

		config.moduleConfig = new SwerveModuleConfig();
		config.moduleConfig.pid = new SlotConfiguration();
		config.moduleConfig.pid.closedLoopPeriod = (int) table.getEntry("module.pid.period").getDouble(1);
		config.moduleConfig.pid.kP = table.getEntry("module.pid.kp").getDouble(0);
		config.moduleConfig.pid.kI = table.getEntry("module.pid.ki").getDouble(0);
		config.moduleConfig.pid.integralZone = table.getEntry("module.pid.izone").getDouble(0);
		config.moduleConfig.pid.maxIntegralAccumulator = table.getEntry("module.pid.maxiacc").getDouble(0);
		config.moduleConfig.pid.kD = table.getEntry("module.pid.kd").getDouble(0);
		config.moduleConfig.pid.kF = table.getEntry("module.pid.kf").getDouble(0);

		config.moduleConfig.maxRamp = table.getEntry("module.maxramp").getDouble(0);
		config.moduleConfig.reverseThreshold = table.getEntry("module.reversethreshold").getDouble(0.25);
		config.moduleConfig.steerBrake = table.getEntry("module.steerbrake").getBoolean(false);

		config.moduleConfig.steerCurrentLimitEnabled = table.getEntry("module.steercurrentlimitenabled").getBoolean(false);
		config.moduleConfig.steerCurrentLimit = table.getEntry("module.steercurrentlimit").getDouble(0);
		config.moduleConfig.steerCurrentLimitTime = table.getEntry("module.steercurrentlimittime").getDouble(0);

		config.moduleConfig.driveCurrentLimitEnabled = table.getEntry("module.drivecurrentlimitenabled").getBoolean(false);
		config.moduleConfig.driveCurrentLimit = table.getEntry("module.drivecurrentlimit").getDouble(0);
		config.moduleConfig.driveCurrentLimitTime = table.getEntry("module.drivecurrentlimittime").getDouble(0);

		config.gyroFactor = table.getEntry("gyrofactor").getDouble(0);

		config.width = table.getEntry("width").getDouble(1);
		config.length = table.getEntry("length").getDouble(1);
		config.wheelDistance = table.getEntry("wheeldistance").getDouble(0);

		config.maxLinearAccel = table.getEntry("maxlinearaccel").getDouble(0);
		config.maxRotateAccel = table.getEntry("maxrotateaccel").getDouble(0);

		return config;
	}
}
