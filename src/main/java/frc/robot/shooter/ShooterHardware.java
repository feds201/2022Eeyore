package frc.robot.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import frc.robot.Subsystem;
import frc.robot.config.ShooterHardwareConfig;

public class ShooterHardware implements Subsystem {

	private final TalonFX topMotor;
	private final TalonFX bottomMotor;
	private final TalonFX feederMotor;

	private double fireThreshold;
	private double feederSpeed;
	private double feederUnjamSpeed;

	private double topSpeed;
	private double bottomSpeed;
	private boolean fire = false;
	private boolean unjam = false;

	private boolean updateSpeed = true;
	private boolean currentlyFiring = false;
	private boolean currentlyUnjamming = false;

	private long lastTime;
	private double minFireTime;
	private double fireTime;

	public ShooterHardware(int topChannel, int bottomChannel, int feederChannel,
							ShooterHardwareConfig config) {
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

	public boolean isSpinning() {
		return topSpeed != 0 || bottomSpeed != 0;
	}

	public boolean isReady() {
		return isSpinning() &&
				Math.abs(topSpeed - topMotor.getSelectedSensorVelocity()) < fireThreshold &&
				Math.abs(bottomSpeed - bottomMotor.getSelectedSensorVelocity()) < fireThreshold;
	}

	public boolean isFiring() {
		return currentlyFiring;
	}

	public void setUnjam(boolean unjam) {
		this.unjam = unjam;
	}

	@Override
	public void tick() {
		long currentTime = System.currentTimeMillis();
		double timeDeltaSeconds = (currentTime - lastTime) / 1000d;
		lastTime = currentTime;

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

		if (fireTime > 0)
			fireTime -= timeDeltaSeconds;
		boolean shouldFire = fireTime > 0 || (fire && isReady());

		if (unjam || currentlyUnjamming) {
			if (unjam != currentlyUnjamming) {
				feederMotor.set(ControlMode.PercentOutput, unjam ? feederUnjamSpeed : 0);
				currentlyUnjamming = unjam;
				fireTime = 0;
			}
		} else {
			if (shouldFire != currentlyFiring) {
				feederMotor.set(ControlMode.PercentOutput, shouldFire ? feederSpeed : 0);
				if (shouldFire)
					fireTime = minFireTime;
				currentlyFiring = shouldFire;
			}
		}
	}

	public void configure(ShooterHardwareConfig config) {
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

		fireThreshold = config.fireThreshold;
		feederSpeed = config.feederSpeed;
		feederUnjamSpeed = config.feederUnjamSpeed;
		minFireTime = config.minFireTime;

		lastTime = System.currentTimeMillis();
		fireTime = 0;
	}
}
