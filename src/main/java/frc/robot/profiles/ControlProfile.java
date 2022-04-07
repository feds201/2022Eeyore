package frc.robot.profiles;

import frc.robot.shooter.ShooterMode;

public abstract class ControlProfile {

	protected double swerveLinearAngle = 0;
	protected double swerveLinearSpeed = 0;
	protected double swerveRotate = 0;
	protected boolean swerveAlign = false;
	protected boolean swerveAlignRumble = false;

	protected boolean intakeDeploy = false;
	protected boolean intakeActive = false;

	protected ShooterMode shooterMode = ShooterMode.HIGH_GOAL_VISION;
	protected boolean shooterSpin = false;
	protected boolean shooterFire = false;
	protected boolean shooterUnjam = false;
	protected boolean decreaseShooterDistance = false;
	protected boolean increaseShooterDistance = false;

	protected boolean climberUp = false;
	protected boolean climberDown = false;
	protected boolean climberHigh = false;

	protected boolean orientRobot = false;

	protected boolean configReload = false;
	protected boolean configReloadRumble = false;

	protected final double deadzone(double input, double threshold) {
		if (Math.abs(input) < threshold)
			return 0;
		return Math.signum(input) * (Math.abs(input) - threshold) / (1 - threshold);
	}

	public abstract void update();

	public void reset() {
		swerveLinearAngle = 0;
		swerveLinearSpeed = 0;
		swerveRotate = 0;
		swerveAlign = false;
		swerveAlignRumble = false;

		intakeDeploy = false;
		intakeActive = false;

		shooterMode = ShooterMode.HIGH_GOAL_VISION;
		shooterSpin = false;
		shooterFire = false;
		shooterUnjam = false;
		decreaseShooterDistance = false;
		increaseShooterDistance = false;

		climberUp = false;
		climberDown = false;
		climberHigh = false;

		orientRobot = false;

		configReload = false;
		configReloadRumble = false;
	}

	public double getSwerveLinearAngle() {
		return swerveLinearAngle;
	}

	public double getSwerveLinearSpeed() {
		return swerveLinearSpeed;
	}

	public double getSwerveRotate() {
		return swerveRotate;
	}

	public boolean getSwerveAlign() {
		return swerveAlign;
	}

	public boolean getSwerveAlignRumble() {
		return swerveAlignRumble;
	}

	public boolean getIntakeDeploy() {
		return intakeDeploy;
	}

	public boolean getIntakeActive() {
		return intakeActive;
	}

	public ShooterMode getShooterMode() {
		return shooterMode;
	}

	public boolean getShooterSpin() {
		return shooterSpin;
	}

	public boolean getShooterFire() {
		return shooterFire;
	}

	public boolean getShooterUnjam() {
		return shooterUnjam;
	}

	public boolean getDecreaseShooterDistance() {
		return decreaseShooterDistance;
	}

	public boolean getIncreaseShooterDistance() {
		return increaseShooterDistance;
	}

	public boolean getClimberUp() {
		return climberUp;
	}

	public boolean getClimberDown() {
		return climberDown;
	}

	public boolean getClimberHigh() {
		return climberHigh;
	}

	public boolean getOrientRobot() {
		return orientRobot;
	}

	public boolean getConfigReload() {
		return configReload;
	}

	public boolean getConfigReloadRumble() {
		return configReloadRumble;
	}
}
