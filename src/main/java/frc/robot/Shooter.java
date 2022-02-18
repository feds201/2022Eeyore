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

	public static final double FALCON_MAX_SPEED = 21900;
	public static final double SHOOTER_CURRENT_LIMIT = 20;
	public static final double SHOOTER_CURRENT_LIMIT_TIME = 1.5;
	public static final int FEEDER_CURRENT_LIMIT = 20;
	public static final double FEEDER_CURRENT_LIMIT_TIME = 0.75;

	private final TalonFX topMotor;
	private final TalonFX bottomMotor;
	private final TalonSRX feederMotor;

	private final double fireThresholdLower;
	private final double fireThresholdUpper;
	private final double feederSpeed;

	private double topSpeedPercentage;
	private double bottomSpeedPercentage;
	private boolean fire = false;

	private boolean updateSpeed = true;
	private boolean currentlyFiring = false;

	public Shooter(int topChannel, int bottomChannel, int feederChannel,
					double fireThresholdLower, double fireThresholdUpper, double feederSpeed,
					SlotConfiguration pid) {
		if (feederSpeed < 0 || feederSpeed > 1)
			throw new IllegalArgumentException("feeder speed out of bounds");

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
		this.feederSpeed = feederSpeed;
	}

	public void setSpeed(double topSpeedPercentage, double bottomSpeedPercentage) {
		if (topSpeedPercentage < 0 || topSpeedPercentage > 1 ||
			bottomSpeedPercentage < 0 || bottomSpeedPercentage > 1)
			throw new IllegalArgumentException("shooter speed out of bounds");
		if (this.topSpeedPercentage != topSpeedPercentage ||
			this.bottomSpeedPercentage != bottomSpeedPercentage) {
			this.topSpeedPercentage = topSpeedPercentage;
			this.bottomSpeedPercentage = bottomSpeedPercentage;
			updateSpeed = true;
		}
	}

	public void setFire(boolean fire) {
		this.fire = fire;
	}

	@Override
	public void tick() {
		if (updateSpeed) {
			if (topSpeedPercentage > 0)
				topMotor.set(ControlMode.Velocity, topSpeedPercentage * FALCON_MAX_SPEED);
			else
				topMotor.set(ControlMode.PercentOutput, 0);
			if (bottomSpeedPercentage > 0)
				bottomMotor.set(ControlMode.Velocity, bottomSpeedPercentage * FALCON_MAX_SPEED);
			else
				bottomMotor.set(ControlMode.PercentOutput, 0);
		}

		boolean shouldFire = fire && topSpeedPercentage > 0 && bottomSpeedPercentage > 0 &&
								getCurrentSpeedTopPercentage() > fireThresholdLower &&
								getCurrentSpeedTopPercentage() < fireThresholdUpper &&
								getCurrentSpeedBottomPercentage() > fireThresholdLower &&
								getCurrentSpeedBottomPercentage() < fireThresholdUpper;
		if (shouldFire != currentlyFiring) {
			feederMotor.set(ControlMode.PercentOutput, shouldFire ? feederSpeed : 0);
			currentlyFiring = shouldFire;
		}
	}

	public double getCurrentSpeedTopPercentage() {
		return topMotor.getSelectedSensorVelocity() / (topSpeedPercentage * FALCON_MAX_SPEED);
	}

	public double getCurrentSpeedBottomPercentage() {
		return bottomMotor.getSelectedSensorVelocity() / (bottomSpeedPercentage * FALCON_MAX_SPEED);
	}
}
