package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PersistentException;

public class ShooterVisionConfig {

	public SlotConfiguration pid;

	public static ShooterVisionConfig load(String file) throws PersistentException {
		NetworkTable table = NetworkTableInstance.getDefault().getTable("/config/shootervision");
		ShooterVisionConfig config = new ShooterVisionConfig();
		table.loadEntries(file);

		config.pid = new SlotConfiguration();
		config.pid.kP = table.getEntry("pid.kp").getDouble(0);
		config.pid.kI = table.getEntry("pid.ki").getDouble(0);
		config.pid.integralZone = table.getEntry("pid.izone").getDouble(0);
		config.pid.maxIntegralAccumulator = table.getEntry("pid.maxiacc").getDouble(0);
		config.pid.kD = table.getEntry("pid.kd").getDouble(0);
		config.pid.kF = table.getEntry("pid.kf").getDouble(0);

		return config;
	}
}
