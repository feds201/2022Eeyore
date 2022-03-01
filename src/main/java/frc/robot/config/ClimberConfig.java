package frc.robot.config;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PersistentException;

public class ClimberConfig {

	public double speed;
	public double ramp;
	public double encoderCounts;

	public boolean currentLimitEnabled;
	public double currentLimit;
	public double currentLimitTime;

	public static ClimberConfig load(String file) throws PersistentException {
		NetworkTable table = NetworkTableInstance.getDefault().getTable("/config/climber");
		ClimberConfig config = new ClimberConfig();
		table.loadEntries(file);

		config.speed = table.getEntry("speed").getDouble(0);
		config.ramp = table.getEntry("ramp").getDouble(0);
		config.encoderCounts = table.getEntry("encodercounts").getDouble(0);

		config.currentLimitEnabled = table.getEntry("currentlimitenabled").getBoolean(false);
		config.currentLimit = table.getEntry("currentlimit").getDouble(0);
		config.currentLimitTime = table.getEntry("currentlimittime").getDouble(0);

		return config;
	}
}
