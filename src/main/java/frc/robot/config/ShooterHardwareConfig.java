package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

public class ShooterHardwareConfig {

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
}
