package frc.robot.profiles.auton.planned;

import frc.robot.profiles.ControlProfile;
import frc.robot.profiles.auton.planned.AutonPlan.AutonPlanPoint;
import frc.robot.swerve.RobotPose;

public class PlannedAutonProfile extends ControlProfile {

	private final RobotPose pose;
	private final AutonPlan plan;
	private boolean first = true;
	private int index = 0;

	private double distanceToTarget;
	private double angleError;

	public PlannedAutonProfile(RobotPose pose, AutonPlan plan) {
		if (pose == null)
			throw new IllegalArgumentException("pose is null");
		if (plan == null)
			throw new IllegalArgumentException("plan is null");
		if (plan.points.length == 0)
			throw new IllegalArgumentException("plan is empty");
		this.pose = pose;
		this.plan = plan;
	}

	@Override
	public void update() {
		if (first) {
			pose.x = plan.start.x;
			pose.y = plan.start.y;
			pose.angle = plan.start.angle;
			first = false;
		}

		AutonPlanPoint point = plan.points[index];

		double xError = point.x - pose.x;
		double yError = point.y - pose.y;
		angleError = point.angle - pose.angle;
		distanceToTarget = Math.sqrt(xError * xError + yError * yError);

		double directionToTarget = ((-Math.atan2(yError, xError) / Math.PI / 2 + 0.25 - pose.angle) % 1 + 1) % 1;

		swerveLinearAngle = directionToTarget;
		if (distanceToTarget >= point.linearRamp)
			swerveLinearSpeed = point.linearCruise;
		else
			swerveLinearSpeed = point.linearCruise * (distanceToTarget / point.linearRamp);

		if (Math.abs(angleError) >= point.rotateRamp)
			swerveRotate = Math.copySign(point.rotateCruise, angleError);
		else
			swerveRotate = point.rotateCruise * (angleError / point.rotateRamp);
	}

	public void nextPoint() {
		if (index < plan.points.length - 1)
			index++;
	}

	public double getPositionError() {
		return distanceToTarget;
	}

	public double getAngleError() {
		return angleError;
	}
}
