package frc.robot.profiles.teleop;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.profiles.ControlProfile;
import frc.robot.shooter.ShooterMode;

public class DefaultDriverProfile extends ControlProfile {

	public static final double THRESHOLD = 0.025;
	public static final double SHOOTER_START_THRESHOLD = 0.6;
	public static final double SHOOTER_STOP_THRESHOLD = 0.4;

	private final XboxController driver;
	private final XboxController operator;

	private boolean shooterToggleTripped = false;

	public DefaultDriverProfile(XboxController driver, XboxController operator) {
		this.driver = driver;
		this.operator = operator;
	}

	@Override
	public void update() {
		double forward = -driver.getLeftY();
		double strafe = driver.getLeftX();
		double rotate = driver.getRightX();
		double linearAngle = -Math.atan2(forward, strafe) / Math.PI / 2 + 0.25;
		linearAngle = (linearAngle % 1 + 1) % 1;
		double linearSpeed = Math.sqrt(forward * forward + strafe * strafe);
		swerveLinearAngle = linearAngle;
		swerveLinearSpeed = deadzone(linearSpeed, THRESHOLD);
		swerveRotate = deadzone(rotate, THRESHOLD) / 2;

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

		swerveAlign = (driver.getBackButton() && driver.getStartButton()) &&
						(driver.getBackButtonPressed() || driver.getStartButtonPressed());
		swerveAlignRumble = driver.getBackButton() && driver.getStartButton();

		configReload = ((driver.getLeftStickButton() && driver.getRightStickButton()) &&
						(driver.getLeftStickButtonPressed() || driver.getRightStickButtonPressed()));
		configReloadRumble = driver.getLeftStickButton() && driver.getRightStickButton();
	}
}
