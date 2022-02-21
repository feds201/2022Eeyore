package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.FilterConfiguration;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class SDSMk4FXModule implements ISwerveModule {

	// The threshold to reverse is 0.3 vs what you might expect (0.25) to prevent the wheels from losing control.
	public static final double REVERSE_THRESHOLD = 0.3;

	public static final double STEER_CURRENT_LIMIT = 25;
	public static final double STEER_CURRENT_LIMIT_TIME = 0.5;
	public static final double DRIVE_CURRENT_LIMIT = 25;
	public static final double DRIVE_CURRENT_LIMIT_TIME = 1.0;

	public static final double DRIVE_ENCODER_COUNTS = 8.41 * 2048;
	public static final double STEER_CENTRAL_ENCODER_COUNTS = 4096;
	public static final double STEER_MOTOR_ENCODER_COUNTS = 2048 * 12.8;

	public static final int RELATIVE_INIT_DELAY = 1000;

	private final TalonFX steer;
	private final TalonFX drive;
	private double angleOffset;

	private final long initTime;
	private boolean initialized = false;

	private double targetAngle = 0;
	private double targetSpeed = 0;
	private double realCurrentAngle = 0;
	private double effectiveCurrentAngle = 0;
	private double currentSpeed = 0;
	private boolean reversed = false;

	public SDSMk4FXModule(int steerChannel, int driveChannel, int encoderChannel, double angleOffset,
							SlotConfiguration pid, double maxRamp) {
		steer = new TalonFX(steerChannel);
		TalonFXConfiguration steerConfig = new TalonFXConfiguration();
		steerConfig.neutralDeadband = 0.001;
		steerConfig.remoteFilter0 = new FilterConfiguration();
		steerConfig.remoteFilter0.remoteSensorDeviceID = encoderChannel;
		steerConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonSRX_SelectedSensor;
		steerConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
		steerConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
		steerConfig.feedbackNotContinuous = false;
		steerConfig.slot0 = pid;
		steerConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration();
		steerConfig.supplyCurrLimit.enable = true;
		steerConfig.supplyCurrLimit.currentLimit = STEER_CURRENT_LIMIT;
		steerConfig.supplyCurrLimit.triggerThresholdTime = STEER_CURRENT_LIMIT_TIME;
		steer.configAllSettings(steerConfig);
		steer.selectProfileSlot(0, 0);
		steer.setInverted(true);
		steer.setNeutralMode(NeutralMode.Brake);

		drive = new TalonFX(driveChannel);
		TalonFXConfiguration driveConfig = new TalonFXConfiguration();
		driveConfig.neutralDeadband = 0.001;
		driveConfig.openloopRamp = maxRamp;
		driveConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
		driveConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration();
		driveConfig.supplyCurrLimit.enable = true;
		driveConfig.supplyCurrLimit.currentLimit = DRIVE_CURRENT_LIMIT;
		driveConfig.supplyCurrLimit.triggerThresholdTime = DRIVE_CURRENT_LIMIT_TIME;
		drive.configAllSettings(driveConfig);
		drive.setInverted(true);
		drive.setNeutralMode(NeutralMode.Coast);

		this.angleOffset = angleOffset;

		initTime = System.currentTimeMillis();
	}

	@Override
	public void setTargetVelocity(double angle, double speed) {
		targetAngle = (angle % 1 + 1) % 1;
		targetSpeed = speed;
	}

	// Written by Michael Kaatz (2022)
	@Override
	public void tick() {
		if (!initialized && System.currentTimeMillis() > initTime + RELATIVE_INIT_DELAY) {
			steer.setSelectedSensorPosition((STEER_CENTRAL_ENCODER_COUNTS - steer.getSelectedSensorPosition(1)) *
											STEER_MOTOR_ENCODER_COUNTS / STEER_CENTRAL_ENCODER_COUNTS);
			initialized = true;
		}
		if (initialized) {
			realCurrentAngle = steer.getSelectedSensorPosition() / STEER_MOTOR_ENCODER_COUNTS;
			effectiveCurrentAngle = ((realCurrentAngle - angleOffset) % 1 + (reversed ? 0.5 : 0) + 1) % 1;
			currentSpeed = drive.getSelectedSensorVelocity() / DRIVE_ENCODER_COUNTS * 10;

			// We don't want to move the wheels if we don't have to.
			if (targetSpeed != 0)
			{
				// The loop error is just the negative opposite of the continous error.
				double errorContinous = -(effectiveCurrentAngle - targetAngle);
				double errorLoop = -(1 - Math.abs(errorContinous)) * Math.signum(errorContinous);

				// Sign is preserved since it is used to determine which way we need to turn the wheel.
				double targetError;
				if (Math.abs(errorContinous) < Math.abs(errorLoop))
					targetError = errorContinous;
				else
					targetError = errorLoop;

				// In some cases it is better to reverse the direction of the drive wheel rather than spinning all the way around.
				if (Math.abs(targetError) > REVERSE_THRESHOLD)
				{
					reversed = !reversed;
					drive.setInverted(!reversed);
					// Quick way to recalculate the offset of the new angle from the target.
					targetError = -Math.signum(targetError) * (0.5 - Math.abs(targetError));
				}

				steer.set(ControlMode.Position, (realCurrentAngle + targetError) * STEER_MOTOR_ENCODER_COUNTS);
				drive.set(ControlMode.PercentOutput, targetSpeed);
			}
			else
			{
				steer.set(ControlMode.PercentOutput, 0);
				drive.set(ControlMode.PercentOutput, 0);
			}
		}
	}

	@Override
	public double getAngleOffset() {
		return angleOffset;
	}

	@Override
	public void setAngleOffsetAbsolute(double offset) {
		angleOffset = offset;
		angleOffset = (angleOffset % 1 + 1) % 1;
	}

	@Override
	public void setAngleOffsetRelative(double offset) {
		angleOffset += offset;
		angleOffset = (angleOffset % 1 + 1) % 1;
	}

	@Override
	public void align() {
		angleOffset += realCurrentAngle;
		angleOffset = (angleOffset % 1 + 1) % 1;
	}

	@Override
	public double getCurrentAngle() {
		return effectiveCurrentAngle;
	}

	@Override
	public double getCurrentSpeed() {
		return currentSpeed;
	}

	@Override
	public double getTargetAngle() {
		return targetAngle;
	}

	@Override
	public double getTargetSpeed() {
		return targetSpeed;
	}
}
