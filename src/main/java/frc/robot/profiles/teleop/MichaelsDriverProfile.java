package frc.robot.profiles.teleop;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.profiles.ControlProfile;

public class MichaelsDriverProfile extends ControlProfile {

	public static final double THRESHOLD = 0.025;
	public static final double SHOOTER_START_THRESHOLD = 0.6;
	public static final double SHOOTER_STOP_THRESHOLD = 0.4;

	private final XboxController driver;
	private final XboxController operator;

	private boolean shooterToggleTripped = false;

	public MichaelsDriverProfile(XboxController driver, XboxController operator) {
		this.driver = driver;
		this.operator = operator;
	}

	public void update() {
		double forward = -driver.getLeftY();
		double strafe = driver.getLeftX();
		double rotate = driver.getRightX();
		double linearAngle = -Math.atan2(forward, strafe) / Math.PI / 2 + 0.25;
		linearAngle = (linearAngle % 1 + 1) % 1;
		double linearSpeed = Math.sqrt(forward * forward + strafe * strafe);
		swerveLinearAngle = linearAngle;
		swerveLinearSpeed = Math.pow(deadzone(linearSpeed, THRESHOLD), 2);
		swerveRotate = deadzone(rotate, THRESHOLD) / 2;

		if (driver.getLeftBumperPressed())
			intakeDeploy = !intakeDeploy;
		intakeActive = driver.getRightBumper();

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
