package frc.robot.profiles.auton.planned;

import frc.robot.shooter.ShooterMode;
import frc.robot.swerve.RobotPose;
import frc.robot.vision.BallVision;

public class AdvancedQuintAutonProfile extends PlannedAutonProfile {

	private final double period;

	private final BallVision ballVision;

	private int step = 0;
	private double time = 0;

	public AdvancedQuintAutonProfile(double period, RobotPose pose, AutonPlan plan,
										BallVision ballVision) {
		super(pose, plan);
		this.period = period;
		this.ballVision = ballVision;
	}

	@Override
	public void update() {
		if (step == 0) {
			// A1
			super.update();
			intakeDeploy = true;
			intakeActive = true;
			if (Math.abs(getAngleError()) < 0.125 && getPositionError() < 2) {
				intakeDeploy = false;
				intakeActive = false;
				step++;
			}
		}
		if (step == 1) {
			// A2
			nextPoint();
			super.update();
			shooterMode = ShooterMode.HIGH_GOAL_VISION;
			shooterSpin = true;
			time = 1;
			step++;
		} else if (step == 2) {
			super.update();
			time -= period;
			if (time <= 0) {
				shooterFire = true;
				step++;
				time = 1.5;
			}
		} else if (step == 3) {
			super.update();
			time -= period;
			if (time <= 0) {
				shooterSpin = false;
				shooterFire = false;
				step++;
			}
		}
		if (step == 4) {
			// B1
			nextPoint();
			super.update();
			step++;
		} else if (step == 5) {
			super.update();
			if (Math.abs(getAngleError()) < 0.125 && getPositionError() < 6) {
				intakeDeploy = false;
				intakeActive = false;
				step++;
			} else {
				intakeDeploy = true;
				intakeActive = true;
			}
		}
		if (step == 6) {
			// B2
			nextPoint();
			super.update();
			step++;
		} else if (step == 7) {
			super.update();
			if (Math.abs(getAngleError()) < 0.25) {
				shooterMode = ShooterMode.HIGH_GOAL_VISION;
				shooterSpin = true;
				step++;
				time = 1;
			}
		} else if (step == 8) {
			super.update();
			time -= period;
			if (time <= 0) {
				shooterFire = true;
				step++;
				time = 1;
			}
		} else if (step == 9) {
			super.update();
			time -= period;
			if (time <= 0) {
				shooterSpin = false;
				shooterFire = false;
				step++;
			}
		}
		if (step == 10) {
			// C1
			nextPoint();
			super.update();
			step++;
		} else if (step == 11) {
			super.update();
			if (Math.abs(getAngleError()) < 0.25) {
				intakeDeploy = true;
				if (Math.abs(getAngleError()) < 0.125 && getPositionError() < 2) {
					step++;
				}
			}
		} else if (step == 12) {
			nextPoint();
			super.update();
			intakeActive = true;
			step++;
			time = 0.5;
		} else if (step == 13) {
			super.update();
			if (Math.abs(getAngleError()) < 0.125) {
				if (time <= 0 || getForwardError() < 2) {
					swerveLinearAngle = 0;
					swerveLinearSpeed = 0;
					swerveRotate = 0;
					step++;
					time = 1;
				} else {
					if (ballVision.hasTarget()) {
						double x = ballVision.getCorrection();
						double y = Math.cos(swerveLinearAngle * Math.PI * 2) * swerveLinearSpeed;
						swerveLinearAngle = -Math.atan2(y, x) / Math.PI / 2 + 0.25;
						swerveLinearSpeed = Math.sqrt(x * x + y * y);
					} else {
						time -= period;
						swerveLinearAngle = 0;
						swerveLinearSpeed = 0;
					}
				}
			}
		} else if (step == 14) {
			time -= period;
			if (time <= 0) {
				intakeDeploy = false;
				intakeActive = false;
				step++;
			}
		}
		if (step == 15) {
			// C2
			nextPoint();
			super.update();
			step++;
		} else if (step == 16) {
			super.update();
			if (Math.abs(getAngleError()) < 0.25 && getPositionError() < 6) {
				shooterMode = ShooterMode.HIGH_GOAL_VISION;
				shooterSpin = true;
				step++;
				time = 0.6;
			}
		} else if (step == 17) {
			super.update();
			time -= period;
			if (time <= 0) {
				shooterFire = true;
				step++;
				time = 1.5;
			}
		} else if (step == 18) {
			super.update();
			time -= period;
			if (time <= 0) {
				shooterSpin = false;
				shooterFire = false;
				step++;
			}
		} else if (step == 19) {
			swerveLinearAngle = 0;
			swerveLinearSpeed = 0;
			swerveRotate = 0;
		}
	}

	@Override
	public void reset() {
		super.reset();
		step = 0;
	}
}
