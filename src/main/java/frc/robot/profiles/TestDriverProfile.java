package frc.robot.profiles;

import edu.wpi.first.wpilibj.XboxController;

public class TestDriverProfile extends DriverProfile {

	public static final double THRESHOLD = 0.025;
	public static final double SHOOTER_START_THRESHOLD = 0.6;
	public static final double SHOOTER_STOP_THRESHOLD = 0.4;

	private XboxController controller;

	private boolean shooterToggleTripped = false;

	public TestDriverProfile(XboxController controller) {
		this.controller = controller;
	}

	public void update() {
		double forward = -controller.getLeftY();
		double strafe = controller.getLeftX();
		double rotate = controller.getRightX();
		double linearAngle = -Math.atan2(forward, strafe) / Math.PI / 2 + 0.25;
		linearAngle = (linearAngle % 1 + 1) % 1;
		double linearSpeed = Math.sqrt(forward * forward + strafe * strafe);
		swerveLinearAngle = linearAngle;
		swerveLinearSpeed = Math.pow(deadzone(linearSpeed, THRESHOLD), 2);
		swerveRotate = deadzone(rotate, THRESHOLD) / 2;

		if (controller.getLeftBumperPressed())
			intakeDeploy = !intakeDeploy;
		intakeActive = controller.getRightBumper();

		if (!controller.getYButton() && !controller.getAButton()) {
			if (!shooterToggleTripped && controller.getLeftTriggerAxis() > SHOOTER_START_THRESHOLD) {
				shooterRev = !shooterRev;
				shooterToggleTripped = true;
			} else if (shooterToggleTripped && controller.getLeftTriggerAxis() < SHOOTER_STOP_THRESHOLD)
				shooterToggleTripped = false;
			shooterFire = controller.getRightTriggerAxis() > SHOOTER_START_THRESHOLD;

			climberUp = false;
			climberDown = false;
		} else {
			shooterRev = false;
			shooterFire = false;
			shooterToggleTripped = false;

			if (controller.getYButton()) {
				climberUp = true;
				climberDown = false;
			} else if (controller.getAButton()) {
				climberUp = false;
				climberDown = true;
			}
		}

		decreaseShooterDistance = controller.getXButtonPressed();
		increaseShooterDistance = controller.getBButtonPressed();

		swerveAlign = (controller.getBackButton() && controller.getStartButton()) &&
						(controller.getBackButtonPressed() || controller.getStartButtonPressed());
		swerveAlignRumble = controller.getBackButton() && controller.getStartButton();

		configReload = ((controller.getLeftStickButton() && controller.getRightStickButton()) &&
						(controller.getLeftStickButtonPressed() || controller.getRightStickButtonPressed()));
		configReloadRumble = controller.getLeftStickButton() && controller.getRightStickButton();
	}
}
