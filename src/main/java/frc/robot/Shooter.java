package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import frc.robot.config.ShooterConfig;

public class Shooter implements Subsystem {

	private final TalonFX topMotor;
	private final TalonFX bottomMotor;
	private final TalonFX feederMotor;

	private double fireThresholdLower;
	private double fireThresholdUpper;
	private double feederSpeed;

	private double topSpeed;
	private double bottomSpeed;
	private boolean fire = false;

	private boolean updateSpeed = true;
	private boolean currentlyFiring = false;

	public Shooter(int topChannel, int bottomChannel, int feederChannel,
					ShooterConfig config) {
		if (feederSpeed < 0 || feederSpeed > 1)
			throw new IllegalArgumentException("feeder speed out of bounds");

		topMotor = new TalonFX(topChannel);
		bottomMotor = new TalonFX(bottomChannel);
		feederMotor = new TalonFX(feederChannel);

		configure(config);
	}

	public void setSpeed(double topSpeed, double bottomSpeed) {
		if (topSpeed < 0 || bottomSpeed < 0)
			throw new IllegalArgumentException("shooter speed out of bounds");
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
			if (topSpeed > 0)
				topMotor.set(ControlMode.Velocity, topSpeed);
			else
				topMotor.set(ControlMode.PercentOutput, 0);
			if (bottomSpeed > 0)
				bottomMotor.set(ControlMode.Velocity, bottomSpeed);
			else
				bottomMotor.set(ControlMode.PercentOutput, 0);
		}

		boolean shouldFire = fire && topSpeed > 0 && bottomSpeed > 0 &&
								getCurrentSpeedTopPercentage() > fireThresholdLower &&
								getCurrentSpeedTopPercentage() < fireThresholdUpper &&
								getCurrentSpeedBottomPercentage() > fireThresholdLower &&
								getCurrentSpeedBottomPercentage() < fireThresholdUpper;
		if (shouldFire != currentlyFiring) {
			feederMotor.set(ControlMode.PercentOutput, shouldFire ? feederSpeed : 0);
			currentlyFiring = shouldFire;
		}
	}

	public void configure(ShooterConfig config) {
		TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();
		shooterMotorConfig.neutralDeadband = 0.001;
		shooterMotorConfig.openloopRamp = 0;
		shooterMotorConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
		shooterMotorConfig.slot0 = config.pid;
		shooterMotorConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration();
		shooterMotorConfig.supplyCurrLimit.enable = config.shooterCurrentLimitEnabled;
		shooterMotorConfig.supplyCurrLimit.currentLimit = config.shooterCurrentLimit;
		shooterMotorConfig.supplyCurrLimit.triggerThresholdTime = config.shooterCurrentLimitTime;
		topMotor.configAllSettings(shooterMotorConfig);
		topMotor.selectProfileSlot(0, 0);
		topMotor.setNeutralMode(config.shooterBrake ? NeutralMode.Brake : NeutralMode.Coast);
		topMotor.setInverted(TalonFXInvertType.CounterClockwise);
		bottomMotor.configAllSettings(shooterMotorConfig);
		bottomMotor.selectProfileSlot(0, 0);
		bottomMotor.setNeutralMode(config.shooterBrake ? NeutralMode.Brake : NeutralMode.Coast);
		bottomMotor.setInverted(TalonFXInvertType.Clockwise);

		TalonFXConfiguration feederMotorConfig = new TalonFXConfiguration();
		feederMotorConfig.neutralDeadband = 0.001;
		feederMotorConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration();
		feederMotorConfig.supplyCurrLimit.enable = config.feederCurrentLimitEnabled;
		feederMotorConfig.supplyCurrLimit.currentLimit = config.feederCurrentLimit;
		feederMotorConfig.supplyCurrLimit.triggerThresholdTime = config.feederCurrentLimitTime;
		feederMotor.configAllSettings(feederMotorConfig);
		feederMotor.setNeutralMode(NeutralMode.Brake);
		feederMotor.setInverted(TalonFXInvertType.Clockwise);

		fireThresholdLower = config.fireThresholdLower;
		fireThresholdUpper = config.fireThresholdUpper;
		feederSpeed = config.feederSpeed;
	}

	public double getCurrentSpeedTopPercentage() {
		return topMotor.getSelectedSensorVelocity() / topSpeed;
	}

	public double getCurrentSpeedBottomPercentage() {
		return bottomMotor.getSelectedSensorVelocity() / bottomSpeed;
	}
}
