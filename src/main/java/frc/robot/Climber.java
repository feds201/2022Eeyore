package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class Climber implements Subsystem {

	public static final double CURRENT_LIMIT = 35;
	public static final double CURRENT_LIMIT_TIME = 0.75;

	private final TalonFX leftMotor;
	private final TalonFX rightMotor;
	private final double speed;

	private boolean update = true;
	private int position = 0;

	public Climber(int leftChannel, int rightChannel, int encoderCounts, double speed, double ramp) {
		if (speed < 0 || speed > 1)
			throw new IllegalArgumentException("speed out of bounds");

		leftMotor = new TalonFX(leftChannel);
		TalonFXConfiguration leftConfig = new TalonFXConfiguration();
		leftConfig.neutralDeadband = 0.001;
		leftConfig.openloopRamp = ramp;
		leftConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
		leftConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
		leftConfig.feedbackNotContinuous = false;
		leftConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration();
		leftConfig.supplyCurrLimit.enable = true;
		leftConfig.supplyCurrLimit.currentLimit = CURRENT_LIMIT;
		leftConfig.supplyCurrLimit.triggerThresholdTime = CURRENT_LIMIT_TIME;
		leftConfig.forwardSoftLimitEnable = false;
		leftConfig.forwardSoftLimitThreshold = encoderCounts;
		leftConfig.reverseSoftLimitEnable = false;
		leftConfig.reverseSoftLimitThreshold = 0;
		leftMotor.configAllSettings(leftConfig, 1000);
		leftMotor.setInverted(true);
		leftMotor.setNeutralMode(NeutralMode.Brake);

		rightMotor = new TalonFX(rightChannel);
		TalonFXConfiguration rightConfig = new TalonFXConfiguration();
		rightConfig.neutralDeadband = 0.001;
		rightConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration();
		rightConfig.supplyCurrLimit.enable = true;
		rightConfig.supplyCurrLimit.currentLimit = CURRENT_LIMIT;
		rightConfig.supplyCurrLimit.triggerThresholdTime = CURRENT_LIMIT_TIME;
		rightMotor.configAllSettings(rightConfig, 1000);
		rightMotor.setInverted(TalonFXInvertType.OpposeMaster);
		rightMotor.setNeutralMode(NeutralMode.Brake);
		rightMotor.follow(leftMotor);

		this.speed = speed;
	}

	public void setTargetPosition(int position) {
		if (this.position != position) {
			this.position = position;
			update = true;
		}
	}

	@Override
	public void tick() {
		if (update) {
			if (position == 1)
				leftMotor.set(ControlMode.PercentOutput, speed);
			else if (position == -1)
				leftMotor.set(ControlMode.PercentOutput, -speed);
			else
				leftMotor.set(ControlMode.PercentOutput, 0);
		}
	}
}
