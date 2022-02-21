package frc.robot.profiles;

import edu.wpi.first.wpilibj.XboxController;

public abstract class DriverProfile {

	protected double swerveLinearAngle = 0;
	protected double swerveLinearSpeed = 0;
	protected double swerveRotate = 0;
	protected boolean swerveAlign = false;
	protected boolean swerveAlignRumble = false;

	protected boolean intakeDeploy = false;
	protected boolean intakeActive = false;

	protected boolean shooterRev = false;
	protected boolean shooterFire = false;

	protected boolean climberUp = false;
	protected boolean climberDown = false;

	protected boolean configReload = false;
	protected boolean configReloadRumble = false;

	protected final double deadzone(double input, double threshold) {
		if (Math.abs(input) < threshold)
			return 0;
		return Math.signum(input) * (Math.abs(input) - threshold) / (1 - threshold);
	}

	public abstract void update(XboxController driver, XboxController operator);

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

	public boolean getShooterRev() {
		return shooterRev;
	}

	public boolean getShooterFire() {
		return shooterFire;
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
