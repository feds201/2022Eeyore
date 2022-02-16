package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

public class Shooter implements Subsystem {

	public static final double FALCON_MAX_SPEED = 6000 * 2048 / 10;
	public static final double SHOOTER_CURRENT_LIMIT = 20;
	public static final double SHOOTER_CURRENT_LIMIT_TIME = 1.5;
	public static final int FEEDER_CURRENT_LIMIT = 20;
	public static final double FEEDER_CURRENT_LIMIT_TIME = 0.75;

	private final TalonFX topMotor;
	private final TalonFX bottomMotor;
	private final TalonSRX feederMotor;

	private final double fireThresholdLower;
	private final double fireThresholdUpper;

	private double topSpeed;
	private double bottomSpeed;
	private boolean fire = false;

	public Shooter(int topChannel, int bottomChannel, int feederChannel,
					double fireThresholdLower, double fireThresholdUpper, SlotConfiguration pid) {
		topMotor = new TalonFX(topChannel);
		bottomMotor = new TalonFX(bottomChannel);
		TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();
		shooterMotorConfig.neutralDeadband = 0.001;
		shooterMotorConfig.openloopRamp = 0;
		shooterMotorConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
		shooterMotorConfig.slot0 = pid;
		shooterMotorConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration();
		shooterMotorConfig.supplyCurrLimit.enable = true;
		shooterMotorConfig.supplyCurrLimit.currentLimit = SHOOTER_CURRENT_LIMIT;
		shooterMotorConfig.supplyCurrLimit.triggerThresholdTime = SHOOTER_CURRENT_LIMIT_TIME;
		topMotor.configAllSettings(shooterMotorConfig);
		topMotor.selectProfileSlot(0, 0);
		topMotor.setNeutralMode(NeutralMode.Coast);
		topMotor.setInverted(false);
		bottomMotor.configAllSettings(shooterMotorConfig);
		bottomMotor.selectProfileSlot(0, 0);
		bottomMotor.setNeutralMode(NeutralMode.Coast);
		bottomMotor.setInverted(true);

		feederMotor = new TalonSRX(feederChannel);
		TalonSRXConfiguration feederMotorConfig = new TalonSRXConfiguration();
		feederMotorConfig.neutralDeadband = 0.001;
		feederMotorConfig.continuousCurrentLimit = FEEDER_CURRENT_LIMIT;
		feederMotorConfig.peakCurrentDuration = (int) (FEEDER_CURRENT_LIMIT_TIME * 1000);
		feederMotor.configAllSettings(feederMotorConfig);
		feederMotor.setNeutralMode(NeutralMode.Brake);
		feederMotor.setInverted(true);

		this.fireThresholdLower = fireThresholdLower;
		this.fireThresholdUpper = fireThresholdUpper;
	}

	public void setSpeed(double topSpeedPercentage, double bottomSpeedPercentage) {
		if (topSpeedPercentage < 0 || topSpeedPercentage > 1 ||
			bottomSpeedPercentage < 0 || bottomSpeedPercentage > 1)
			throw new IllegalArgumentException("shooter speed out of bounds");
		topSpeed = topSpeedPercentage * FALCON_MAX_SPEED;
		bottomSpeed = bottomSpeedPercentage * FALCON_MAX_SPEED;
	}

	public void setFire(boolean fire) {
		this.fire = fire;
	}

	@Override
	public void tick() {
		topMotor.set(ControlMode.Velocity, topSpeed);
		bottomMotor.set(ControlMode.Velocity, bottomSpeed);

		if (fire && topSpeed > 0 && bottomSpeed > 0 &&
			getCurrentSpeedTopPercentage() > fireThresholdLower &&
			getCurrentSpeedTopPercentage() < fireThresholdUpper &&
			getCurrentSpeedBottomPercentage() > fireThresholdLower &&
			getCurrentSpeedBottomPercentage() < fireThresholdUpper)
			feederMotor.set(ControlMode.PercentOutput, 1);
		else
			feederMotor.set(ControlMode.PercentOutput, 0);
	}

	public double getCurrentSpeedTop() {
		return topMotor.getSelectedSensorVelocity() / 2048 * 10;
	}

	public double getCurrentSpeedBottom() {
		return bottomMotor.getSelectedSensorVelocity() / 2048 * 10;
	}

	public double getCurrentSpeedTopPercentage() {
		return getCurrentSpeedTop() / topSpeed;
	}

	public double getCurrentSpeedBottomPercentage() {
		return getCurrentSpeedBottom() / topSpeed;
	}
}
