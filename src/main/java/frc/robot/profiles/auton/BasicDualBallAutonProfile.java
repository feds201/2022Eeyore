package frc.robot.profiles.auton;

import frc.robot.profiles.ControlProfile;

public class BasicDualBallAutonProfile extends ControlProfile {

	public static final double STEP1 = 2;
	public static final double STEP1_SPEED = 0.2;

	public static final double STEP2 = 3;

	public static final double STEP3 = 4.5;
	public static final double STEP3_SPEED = 0.5;

	public static final double STEP4 = 8;

	private final double period;

	private double time = 0;

	public BasicDualBallAutonProfile(double period) {
		this.period = period;
	}

	public void update() {
		if (time < STEP1) {
			swerveLinearAngle = 0;
			swerveLinearSpeed = STEP1_SPEED;

			intakeDeploy = true;
			intakeActive = true;
		} else if (time < STEP2) {
			swerveLinearSpeed = 0;
		} else if (time < STEP3) {
			swerveRotate = STEP3_SPEED;

			intakeDeploy = false;
			intakeActive = false;
		} else if (time < STEP4) {
			swerveRotate = 0;

			shooterRev = true;
			shooterFire = true;
		} else {
			swerveLinearAngle = 0;
			swerveLinearSpeed = 0;
			swerveRotate = 0;

			intakeDeploy = false;
			intakeActive = false;

			shooterRev = false;
			shooterFire = false;
		}

		time += period;
	}
}
