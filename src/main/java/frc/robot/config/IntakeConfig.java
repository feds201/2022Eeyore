package frc.robot.config;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PersistentException;

public class IntakeConfig {

	public boolean brake;
	public double speed;

	public boolean currentLimitEnabled;
	public double currentLimit;
	public double currentLimitTime;

	public static IntakeConfig load(String file) throws PersistentException {
		NetworkTable table = NetworkTableInstance.getDefault().getTable("/config/intake");
		IntakeConfig config = new IntakeConfig();
		table.loadEntries(file);

		config.brake = table.getEntry("brake").getBoolean(false);
		config.speed = table.getEntry("speed").getDouble(0);

		config.currentLimitEnabled = table.getEntry("currentlimitenabled").getBoolean(false);
		config.currentLimit = table.getEntry("currentlimit").getDouble(0);
		config.currentLimitTime = table.getEntry("currentlimittime").getDouble(0);

		return config;
	}
}
