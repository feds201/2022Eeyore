package frc.robot.profiles.auton.planned;

import frc.robot.profiles.ControlProfile;
import frc.robot.swerve.RobotPose;

public class PlannedAutonProfile extends ControlProfile {

	private final RobotPose pose;
	private final AutonPlanPoint[] points;
	private int index = 0;

	public PlannedAutonProfile(RobotPose pose, AutonPlanPoint[] points) {
		if (pose == null)
			throw new IllegalArgumentException("pose is null");
		if (points == null)
			throw new IllegalArgumentException("points is null");
		if (points.length == 0)
			throw new IllegalArgumentException("points is empty");
		this.pose = pose;
		this.points = points;
	}

	@Override
	public void update() {
		AutonPlanPoint point = points[index];

		double xError = point.x - pose.x;
		double yError = point.y - pose.y;
		double angleError = point.angle - pose.angle;
		double distanceToTarget = Math.sqrt(xError * xError + yError * yError);

		double directionToTarget = -Math.atan2(yError, xError) / Math.PI / 2 + 0.25 - pose.angle;
		directionToTarget = (directionToTarget % 1 + 1) % 1;

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
		if (index < points.length - 1)
			index++;
	}

	protected double getError() {
		double xDiff = points[index].x - pose.x;
		double yDiff = points[index].y - pose.y;
		double angleDiff = points[index].angle - pose.angle;
		return Math.sqrt(xDiff * xDiff + yDiff * yDiff + angleDiff * angleDiff);
	}
}
