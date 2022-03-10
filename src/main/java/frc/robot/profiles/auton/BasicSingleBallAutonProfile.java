package frc.robot.profiles.auton;

import frc.robot.profiles.ControlProfile;
import frc.robot.shooter.ShooterMode;

public class BasicSingleBallAutonProfile extends ControlProfile {

	public static final double STEP1 = 2;
	public static final double STEP1_SPEED = 0.25;

	public static final double STEP2 = 3;

	public static final double STEP3 = 5;

	private final double period;

	private double time = 0;

	public BasicSingleBallAutonProfile(double period) {
		this.period = period;
	}

	public void update() {
		if (time < STEP1) {
			swerveLinearAngle = 0.5;
			swerveLinearSpeed = STEP1_SPEED;
		} else if (time < STEP2) {
			swerveLinearSpeed = 0;
		} else if (time < STEP3) {
			shooterMode = ShooterMode.HIGH_GOAL_VISION;
			shooterSpin = true;
			shooterFire = true;
		} else {
			swerveLinearAngle = 0;
			swerveLinearSpeed = 0;
			swerveRotate = 0;

			shooterSpin = false;
			shooterFire = false;
		}

		time += period;
	}
}
