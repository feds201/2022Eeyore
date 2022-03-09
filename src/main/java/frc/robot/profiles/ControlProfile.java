package frc.robot.profiles;

public abstract class ControlProfile {

	protected double swerveLinearAngle = 0;
	protected double swerveLinearSpeed = 0;
	protected double swerveRotate = 0;
	protected boolean swerveAlign = false;
	protected boolean swerveAlignRumble = false;

	protected boolean intakeDeploy = false;
	protected boolean intakeActive = false;

	protected boolean shooterSpin = false;
	protected boolean shooterFire = false;
	protected boolean decreaseShooterDistance = false;
	protected boolean increaseShooterDistance = false;

	protected boolean climberUp = false;
	protected boolean climberDown = false;

	protected boolean configReload = false;
	protected boolean configReloadRumble = false;

	protected final double deadzone(double input, double threshold) {
		if (Math.abs(input) < threshold)
			return 0;
		return Math.signum(input) * (Math.abs(input) - threshold) / (1 - threshold);
	}

	public abstract void update();

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

	public boolean getShooterSpin() {
		return shooterSpin;
	}

	public boolean getShooterFire() {
		return shooterFire;
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

	public boolean getConfigReload() {
		return configReload;
	}

	public boolean getConfigReloadRumble() {
		return configReloadRumble;
	}
}
