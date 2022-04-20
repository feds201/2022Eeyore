package frc.robot.profiles.teleop;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.profiles.ControlProfile;
import frc.robot.shooter.ShooterMode;

public class TestDriverProfile extends ControlProfile {

	public static final double THRESHOLD = 0.1;
	public static final double SHOOTER_START_THRESHOLD = 0.6;
	public static final double SHOOTER_STOP_THRESHOLD = 0.4;

	private XboxController controller;

	private boolean shooterToggleTripped = false;

	private boolean swerveAlignTripped = false;
	private boolean configReloadTripped = false;

	public TestDriverProfile(XboxController controller) {
		this.controller = controller;
	}

	@Override
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

		if (controller.getYButton())
			shooterMode = ShooterMode.HIGH_GOAL_VISION;
		else if (controller.getAButton())
			shooterMode = ShooterMode.LOW_GOAL;
		else if (controller.getBButton())
			shooterMode = ShooterMode.EJECT;

		shooterUnjam = controller.getXButton();

		if (!shooterToggleTripped && controller.getLeftTriggerAxis() > SHOOTER_START_THRESHOLD) {
			shooterSpin = !shooterSpin;
			shooterToggleTripped = true;
		} else if (shooterToggleTripped && controller.getLeftTriggerAxis() < SHOOTER_STOP_THRESHOLD)
			shooterToggleTripped = false;
		shooterFire = controller.getRightTriggerAxis() > SHOOTER_START_THRESHOLD;

		if (controller.getPOV() == 0)
			climber = 1;
		else if (controller.getPOV() == 180)
			climber = -1;
		else
			climber = 0;
		climberHigh = controller.getPOV() == 90 || controller.getPOV() == 270;

		decreaseShooterDistance = controller.getLeftStickButtonPressed();
		increaseShooterDistance = controller.getRightStickButtonPressed();

		if (controller.getBackButton() && controller.getStartButton()) {
			swerveAlign = !swerveAlignTripped;
			swerveAlignTripped = true;
			swerveAlignRumble = true;
		} else {
			swerveAlign = false;
			swerveAlignTripped = false;
			swerveAlignRumble = false;
		}

		if (controller.getLeftStickButton() && controller.getRightStickButton()) {
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
