package frc.robot.profiles.auton.planned;

import frc.robot.swerve.RobotPose;

public class AdvancedQuintAutonProfile extends PlannedAutonProfile {

	private boolean moving = false;
	private int time = 0;

	public AdvancedQuintAutonProfile(RobotPose pose, AutonPlanPoint[] points) {
		super(pose, points);
	}

	@Override
	public void update() {
		moving = true;
		if (moving)
			super.update();
	}
}
