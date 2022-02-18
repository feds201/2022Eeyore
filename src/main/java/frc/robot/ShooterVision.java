package frc.robot;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ShooterVision implements Subsystem {

	private final SlotConfiguration pid;
	private final NetworkTable table;

	public ShooterVision(SlotConfiguration pid) {
		this.pid = pid;
		table = NetworkTableInstance.getDefault().getTable("limelight");
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
		return table.getEntry("tx").getDouble(0) * -pid.kP;
	}
}
