package frc.robot.profiles.auton.planned;

import frc.robot.swerve.RobotPose;

public class AdvancedQuintAutonProfile extends PlannedAutonProfile {

	private boolean moving = false;
	private int time = 0;

	public AdvancedQuintAutonProfile(RobotPose pose, AutonPlan plan) {
		super(pose, plan);
	}

	@Override
	public void update() {
		moving = true;
		if (moving)
			super.update();
	}
}
