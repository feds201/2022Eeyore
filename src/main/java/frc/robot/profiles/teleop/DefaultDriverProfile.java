package frc.robot.profiles.teleop;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.config.AbsoluteSteeringConfig;
import frc.robot.profiles.ControlProfile;
import frc.robot.shooter.ShooterMode;
import frc.robot.swerve.RobotPose;

public class DefaultDriverProfile extends ControlProfile {

	public static final double THRESHOLD = 0.025;
	public static final double ABSOLUTE_STEERING_THRESHOLD = 0.2;
	public static final double SHOOTER_START_THRESHOLD = 0.6;
	public static final double SHOOTER_STOP_THRESHOLD = 0.4;

	private final XboxController driver;
	private final XboxController operator;
	private final RobotPose pose;
	private final SlotConfiguration steeringPid;

	private boolean shooterToggleTripped = false;

	private boolean swerveAlignTripped = false;
	private boolean configReloadTripped = false;

	private boolean fieldRelative = false;
	private boolean absoluteSteering = false;

	private long lastTime;
	private double iacc = 0;
	private double lastErr = 0;

	public DefaultDriverProfile(XboxController driver, XboxController operator,
								RobotPose pose, AbsoluteSteeringConfig config) {
		this.driver = driver;
		this.operator = operator;
		this.pose = pose;
		this.steeringPid = config.pid;
	}

	@Override
	public void update() {
		if (driver.getAButton()) {
			fieldRelative = false;
			absoluteSteering = false;
		} else if (driver.getBButton()) {
			fieldRelative = true;
			absoluteSteering = false;
		} else if (driver.getYButton()) {
			fieldRelative = true;
			absoluteSteering = false;
			//absoluteSteering = true;
		}

		double forward = -driver.getLeftY();
		double strafe = driver.getLeftX();
		double rotateY = -driver.getRightY();
		double rotateX = driver.getRightX();

		double linearAngle = -Math.atan2(forward, strafe) / Math.PI / 2 + 0.25;
		if (fieldRelative)
			linearAngle -= pose.angle;
		linearAngle = (linearAngle % 1 + 1) % 1;
		double linearSpeed = deadzone(Math.sqrt(forward * forward + strafe * strafe), THRESHOLD);

		double rotate;
		if (absoluteSteering) {
			double rotateMagnitude = deadzone(Math.sqrt(rotateY * rotateY + rotateX * rotateX),
												ABSOLUTE_STEERING_THRESHOLD);
			if (rotateMagnitude == 0) {
				rotate = 0;

				lastTime = System.currentTimeMillis();
				iacc = 0;
				lastErr = 0;
			} else {
				double rotateAngle = -Math.atan2(rotateY, rotateX) / Math.PI / 2 + 0.25;
				double currentAngle = (pose.angle % 1 + 1) % 1;

				double errorContinous = -(currentAngle - rotateAngle);
				double errorLoop = -(1 - Math.abs(errorContinous)) * Math.signum(errorContinous);

				double targetError;
				if (Math.abs(errorContinous) < Math.abs(errorLoop))
					targetError = errorContinous;
				else
					targetError = errorLoop;

				long currentTime = System.currentTimeMillis();
				double timeDeltaSeconds = (currentTime - lastTime) / 1000d;
				lastTime = currentTime;

				if (Math.abs(targetError) <= steeringPid.integralZone) {
					iacc += targetError * timeDeltaSeconds;
					if (Math.abs(iacc) > steeringPid.maxIntegralAccumulator)
						iacc = Math.signum(iacc) * steeringPid.maxIntegralAccumulator;
				}
				else
					iacc = 0;
				rotate = targetError * steeringPid.kP +
							iacc * steeringPid.kI +
							(targetError - lastErr) / timeDeltaSeconds * steeringPid.kD;
				lastErr = targetError;
			}
		} else {
			rotate = deadzone(rotateX, THRESHOLD) / 2;

			lastTime = System.currentTimeMillis();
			iacc = 0;
			lastErr = 0;
		}

		swerveLinearAngle = linearAngle;
		swerveLinearSpeed = linearSpeed;
		swerveRotate = rotate;

		if (driver.getRightBumperPressed())
			intakeActive = !intakeActive;
		intakeDeploy = intakeActive || driver.getLeftBumper();

		if (operator.getYButton())
			shooterMode = ShooterMode.HIGH_GOAL_VISION;
		else if (operator.getAButton())
			shooterMode = ShooterMode.LOW_GOAL;
		else if (operator.getBButton())
			shooterMode = ShooterMode.EJECT;

		shooterUnjam = operator.getXButton();

		if (operator.getPOV() == -1) {
			if (!shooterToggleTripped && operator.getLeftTriggerAxis() > SHOOTER_START_THRESHOLD) {
				shooterSpin = !shooterSpin;
				shooterToggleTripped = true;
			} else if (shooterToggleTripped && operator.getLeftTriggerAxis() < SHOOTER_STOP_THRESHOLD)
				shooterToggleTripped = false;
			shooterFire = operator.getRightTriggerAxis() > SHOOTER_START_THRESHOLD;

			climberUp = false;
			climberDown = false;
			climberHigh = false;
		} else {
			shooterSpin = false;
			shooterFire = false;
			shooterToggleTripped = false;

			if (operator.getPOV() == 0) {
				climberUp = true;
				climberDown = false;
				climberHigh = false;
			} else if (operator.getPOV() == 180) {
				climberUp = false;
				climberDown = true;
				climberHigh = false;
			} else if (operator.getPOV() == 90 || operator.getPOV() == 270) {
				climberUp = false;
				climberDown = false;
				climberHigh = true;
			} else {
				climberUp = false;
				climberDown = false;
				climberHigh = false;
			}
		}

		decreaseShooterDistance = operator.getLeftBumperPressed();
		increaseShooterDistance = operator.getRightBumperPressed();

		if (driver.getBackButton() && driver.getStartButton()) {
			swerveAlign = !swerveAlignTripped;
			swerveAlignTripped = true;
			swerveAlignRumble = true;
		} else {
			swerveAlign = false;
			swerveAlignTripped = false;
			swerveAlignRumble = false;
		}

		if (driver.getLeftStickButton() && driver.getRightStickButton()) {
			configReload = !configReloadTripped;
			configReloadTripped = true;
			configReloadRumble = true;
		} else {
			configReload = false;
			configReloadTripped = false;
			configReloadRumble = false;
		}
	}

	@Override
	public void reset() {
		super.reset();
		shooterToggleTripped = false;
	}
}
