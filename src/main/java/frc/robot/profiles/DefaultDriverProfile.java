package frc.robot.profiles;

import edu.wpi.first.wpilibj.XboxController;

public class DefaultDriverProfile extends DriverProfile {

	public static final double THRESHOLD = 0.025;
	public static final double SHOOTER_START_THRESHOLD = 0.6;
	public static final double SHOOTER_STOP_THRESHOLD = 0.4;

	private boolean shooterToggleTripped = false;

	public void update(XboxController driver, XboxController operator) {

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

		if (!operator.getYButton() && !operator.getAButton()) {
			if (!shooterToggleTripped && operator.getLeftTriggerAxis() > SHOOTER_START_THRESHOLD) {
				shooterRev = !shooterRev;
				shooterToggleTripped = true;
			} else if (shooterToggleTripped && operator.getLeftTriggerAxis() < SHOOTER_STOP_THRESHOLD)
				shooterToggleTripped = false;
			shooterFire = operator.getRightTriggerAxis() > SHOOTER_START_THRESHOLD;

			climberUp = false;
			climberDown = false;
		} else {
			shooterRev = false;
			shooterFire = false;
			shooterToggleTripped = false;

			if (operator.getYButton()) {
				climberUp = true;
				climberDown = false;
			} else if (operator.getAButton()) {
				climberUp = false;
				climberDown = true;
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
