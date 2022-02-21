package frc.robot.config;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PersistentException;

public class GeneralConfig {

	public double shooterTopSpeed;
	public double shooterBottomSpeed;

	public static GeneralConfig load(String file) throws PersistentException {
		NetworkTable table = NetworkTableInstance.getDefault().getTable("config/general");
		GeneralConfig config = new GeneralConfig();
		table.loadEntries(file);

		config.shooterTopSpeed = table.getEntry("shootertopspeed").getDouble(0);
		config.shooterBottomSpeed = table.getEntry("shooterbottomspeed").getDouble(0);

		return config;
	}
}
