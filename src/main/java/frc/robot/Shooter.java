package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
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
				topMotor.neutralOutput();
			if (bottomSpeed != 0)
				bottomMotor.set(ControlMode.Velocity, bottomSpeed);
			else
				bottomMotor.neutralOutput();
		}

		boolean shouldFire = fire && (topSpeed != 0 || bottomSpeed != 0) &&
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

	public void configure(ShooterConfig config) {
		TalonFXConfiguration topMotorConfig = new TalonFXConfiguration();
		topMotorConfig.neutralDeadband = 0.001;
		topMotorConfig.openloopRamp = 0;
		topMotorConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
		topMotorConfig.slot0 = config.topPid;
		topMotorConfig.voltageCompSaturation = 12;
		topMotorConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration();
		topMotorConfig.supplyCurrLimit.enable = config.shooterCurrentLimitEnabled;
		topMotorConfig.supplyCurrLimit.currentLimit = config.shooterCurrentLimit;
		topMotorConfig.supplyCurrLimit.triggerThresholdTime = config.shooterCurrentLimitTime;
		topMotor.configAllSettings(topMotorConfig);
		topMotor.selectProfileSlot(0, 0);
		topMotor.setNeutralMode(config.shooterBrake ? NeutralMode.Brake : NeutralMode.Coast);
		topMotor.setInverted(TalonFXInvertType.CounterClockwise);
		topMotor.enableVoltageCompensation(true);
		topMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
		topMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
		topMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
		topMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
		topMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
		topMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255);
		topMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
		topMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
		topMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
		topMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255);

		TalonFXConfiguration bottomMotorConfig = new TalonFXConfiguration();
		bottomMotorConfig.neutralDeadband = 0.001;
		bottomMotorConfig.openloopRamp = 0;
		bottomMotorConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
		bottomMotorConfig.slot0 = config.bottomPid;
		bottomMotorConfig.voltageCompSaturation = 12;
		bottomMotorConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration();
		bottomMotorConfig.supplyCurrLimit.enable = config.shooterCurrentLimitEnabled;
		bottomMotorConfig.supplyCurrLimit.currentLimit = config.shooterCurrentLimit;
		bottomMotorConfig.supplyCurrLimit.triggerThresholdTime = config.shooterCurrentLimitTime;
		bottomMotor.configAllSettings(bottomMotorConfig);
		bottomMotor.selectProfileSlot(0, 0);
		bottomMotor.setNeutralMode(config.shooterBrake ? NeutralMode.Brake : NeutralMode.Coast);
		bottomMotor.setInverted(TalonFXInvertType.Clockwise);
		bottomMotor.enableVoltageCompensation(true);
		bottomMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
		bottomMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
		bottomMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
		bottomMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
		bottomMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
		bottomMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255);
		bottomMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
		bottomMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
		bottomMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
		bottomMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255);

		TalonFXConfiguration feederMotorConfig = new TalonFXConfiguration();
		feederMotorConfig.neutralDeadband = 0.001;
		feederMotorConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration();
		feederMotorConfig.supplyCurrLimit.enable = config.feederCurrentLimitEnabled;
		feederMotorConfig.supplyCurrLimit.currentLimit = config.feederCurrentLimit;
		feederMotorConfig.supplyCurrLimit.triggerThresholdTime = config.feederCurrentLimitTime;
		feederMotor.configAllSettings(feederMotorConfig);
		feederMotor.setNeutralMode(NeutralMode.Brake);
		feederMotor.setInverted(TalonFXInvertType.Clockwise);
		feederMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
		feederMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
		feederMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
		feederMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
		feederMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
		feederMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255);
		feederMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
		feederMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
		feederMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
		feederMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255);

		fireThresholdLower = config.fireThresholdLower;
		fireThresholdUpper = config.fireThresholdUpper;
		feederSpeed = config.feederSpeed;
	}
}
