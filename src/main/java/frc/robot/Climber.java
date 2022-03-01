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

	private double speed;

	private boolean update = true;
	private int position = 0;

	public Climber(int leftChannel, int rightChannel, int limitSwitchPort, ClimberConfig config) {
		leftMotor = new TalonFX(leftChannel);
		rightMotor = new TalonFX(rightChannel);
		limitSwitch = new DigitalInput(limitSwitchPort);

		configure(config);
	}

	public void setTargetPosition(int position) {
		if (this.position != position) {
			this.position = position;
			update = true;
		}
	}

	@Override
	public void tick() {
		if (limitSwitch.get())
			leftMotor.setSelectedSensorPosition(0);

		if (update) {
			if (position == 1)
				leftMotor.set(ControlMode.PercentOutput, speed);
			else if (position == -1)
				leftMotor.set(ControlMode.PercentOutput, -speed);
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
		leftConfig.forwardSoftLimitThreshold = config.encoderCounts;
		leftConfig.reverseSoftLimitEnable = true;
		leftConfig.reverseSoftLimitThreshold = 0;
		leftMotor.configAllSettings(leftConfig);
		leftMotor.setInverted(true);
		leftMotor.setNeutralMode(NeutralMode.Brake);
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

		speed = config.speed;
	}
}
