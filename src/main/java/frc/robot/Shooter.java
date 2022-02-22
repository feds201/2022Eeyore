package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class Shooter implements Subsystem {

	public static final double FALCON_MAX_SPEED = 21900;

	public static final double SHOOTER_CURRENT_LIMIT = 20;
	public static final double SHOOTER_CURRENT_LIMIT_TIME = 1.5;
	public static final double FEEDER_CURRENT_LIMIT = 20;
	public static final double FEEDER_CURRENT_LIMIT_TIME = 0.75;

	private final TalonFX topMotor;
	private final TalonFX bottomMotor;
	private final TalonFX feederMotor;

	private final double fireThresholdLower;
	private final double fireThresholdUpper;
	private final double feederSpeed;

	private double topSpeed;
	private double bottomSpeed;
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
		topMotor.setNeutralMode(NeutralMode.Brake);
		topMotor.setInverted(TalonFXInvertType.CounterClockwise);
		bottomMotor.configAllSettings(shooterMotorConfig);
		bottomMotor.selectProfileSlot(0, 0);
		bottomMotor.setNeutralMode(NeutralMode.Brake);
		bottomMotor.setInverted(TalonFXInvertType.Clockwise);

		feederMotor = new TalonFX(feederChannel);
		TalonFXConfiguration feederMotorConfig = new TalonFXConfiguration();
		feederMotorConfig.neutralDeadband = 0.001;
		feederMotorConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration();
		feederMotorConfig.supplyCurrLimit.enable = true;
		feederMotorConfig.supplyCurrLimit.currentLimit = FEEDER_CURRENT_LIMIT;
		feederMotorConfig.supplyCurrLimit.triggerThresholdTime = FEEDER_CURRENT_LIMIT_TIME;
		feederMotor.configAllSettings(feederMotorConfig);
		feederMotor.setNeutralMode(NeutralMode.Brake);
		feederMotor.setInverted(TalonFXInvertType.Clockwise);

		this.fireThresholdLower = fireThresholdLower;
		this.fireThresholdUpper = fireThresholdUpper;
		this.feederSpeed = feederSpeed;
	}

	public void setSpeed(double topSpeed, double bottomSpeed) {
		if (this.topSpeed != topSpeed ||
			this.bottomSpeed != bottomSpeed) {
			this.topSpeed = topSpeed;
			this.bottomSpeed = bottomSpeed;
			updateSpeed = true;
		}
	}

	public void setFire(boolean fire) {
		this.fire = fire;
	}

	@Override
	public void tick() {
		if (updateSpeed) {
			if (topSpeed != 0)
				topMotor.set(ControlMode.Velocity, topSpeed);
			else
				topMotor.set(ControlMode.PercentOutput, 0);
			if (bottomSpeed != 0)
				bottomMotor.set(ControlMode.Velocity, bottomSpeed);
			else
				bottomMotor.set(ControlMode.PercentOutput, 0);
		}

		boolean shouldFire = fire && topSpeed != 0 && bottomSpeed != 0 &&
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
		return topMotor.getSelectedSensorVelocity() / topSpeed;
	}

	public double getCurrentSpeedBottomPercentage() {
		return bottomMotor.getSelectedSensorVelocity() / bottomSpeed;
	}
}
