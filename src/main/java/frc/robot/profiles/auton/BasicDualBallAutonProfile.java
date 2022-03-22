package frc.robot.profiles.auton;

import frc.robot.profiles.ControlProfile;
import frc.robot.shooter.ShooterMode;

public class BasicDualBallAutonProfile extends ControlProfile {

	public static final double STEP1 = 1.5;
	public static final double STEP1_SPEED = 0.35;

	public static final double STEP2 = 3;

	public static final double STEP3 = 5;
	public static final double STEP3_SPEED = 0.2;

	public static final double STEP4 = 8;

	public static final double STEP5 = 13;

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

			shooterMode = ShooterMode.HIGH_GOAL_VISION;
			shooterSpin = true;

			intakeDeploy = false;
			intakeActive = false;
		} else if (time < STEP4) {
			swerveRotate = 0;
		} else if (time < STEP5) {
			shooterFire = true;
		} else {
			swerveLinearAngle = 0;
			swerveLinearSpeed = 0;
			swerveRotate = 0;

			intakeDeploy = false;
			intakeActive = false;

			shooterSpin = false;
			shooterFire = false;
		}

		time += period;
	}

	@Override
	public void reset() {
		super.reset();
		time = 0;
	}
}
