package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.config.ClimberConfig;

public class Climber implements Subsystem {

	private final TalonFX leftMotor;
	private final TalonFX rightMotor;
	private final DigitalInput limitSwitch;

	private double forwardSpeed;
	private double reverseSpeed;

	private double highEncoderCountsLow;
	private double highEncoderCountsHigh;

	private double velocity = 0;
	private boolean high = false;

	public Climber(int leftChannel, int rightChannel, int limitChannel, ClimberConfig config) {
		leftMotor = new TalonFX(leftChannel);
		rightMotor = new TalonFX(rightChannel);
		limitSwitch = new DigitalInput(limitChannel);

		configure(config);
	}

	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}

	public void setHigh(boolean high) {
		this.high = high;
	}

	@Override
	public void tick() {
		if (high) {
			if (leftMotor.getSelectedSensorPosition() < highEncoderCountsLow)
				leftMotor.set(ControlMode.PercentOutput, forwardSpeed);
			else if (leftMotor.getSelectedSensorPosition() > highEncoderCountsHigh &&
						!limitSwitch.get())
				leftMotor.set(ControlMode.PercentOutput, reverseSpeed);
			else
				leftMotor.set(ControlMode.PercentOutput, 0);
		} else {
			if (velocity > 0)
				leftMotor.set(ControlMode.PercentOutput, forwardSpeed * velocity);
			else if (velocity < 0 && !limitSwitch.get())
				leftMotor.set(ControlMode.PercentOutput, reverseSpeed * -velocity);
			else
				leftMotor.set(ControlMode.PercentOutput, 0);
		}
	}

	public void configure(ClimberConfig config) {
		TalonFXConfiguration leftConfig = new TalonFXConfiguration();
		leftConfig.neutralDeadband = 0.001;
		leftConfig.openloopRamp = config.ramp;
		leftConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
		leftConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
		leftConfig.feedbackNotContinuous = false;
		leftConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration();
		leftConfig.supplyCurrLimit.enable = config.currentLimitEnabled;
		leftConfig.supplyCurrLimit.currentLimit = config.currentLimit;
		leftConfig.supplyCurrLimit.triggerThresholdTime = config.currentLimitTime;
		leftConfig.forwardSoftLimitEnable = true;
		leftConfig.forwardSoftLimitThreshold = config.upEncoderCounts;
		leftConfig.reverseSoftLimitEnable = true;
		leftConfig.reverseSoftLimitThreshold = config.downEncoderCounts;
		leftMotor.configAllSettings(leftConfig);
		leftMotor.setInverted(true);
		leftMotor.setNeutralMode(NeutralMode.Brake);
		leftMotor.setSelectedSensorPosition(0);
		leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
		leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
		leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
		leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
		leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
		leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255);
		leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
		leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
		leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
		leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255);

		TalonFXConfiguration rightConfig = new TalonFXConfiguration();
		rightConfig.neutralDeadband = 0.001;
		rightConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration();
		rightConfig.supplyCurrLimit.enable = config.currentLimitEnabled;
		rightConfig.supplyCurrLimit.currentLimit = config.currentLimit;
		rightConfig.supplyCurrLimit.triggerThresholdTime = config.currentLimitTime;
		rightMotor.configAllSettings(rightConfig);
		rightMotor.setInverted(TalonFXInvertType.OpposeMaster);
		rightMotor.setNeutralMode(NeutralMode.Brake);
		rightMotor.follow(leftMotor);
		rightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
		rightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
		rightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
		rightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
		rightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
		rightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255);
		rightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
		rightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
		rightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
		rightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255);

		forwardSpeed = config.forwardSpeed;
		reverseSpeed = config.reverseSpeed;

		highEncoderCountsLow = config.highEncoderCountsLow;
		highEncoderCountsHigh = config.highEncoderCountsHigh;
	}
}
