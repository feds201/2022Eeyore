package frc.robot.swerve;

import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.config.SwerveDriveConfig;

public class FourCornerSwerveDrive implements ISwerveDrive {

	private final ISwerveModule frontLeft;
	private final ISwerveModule frontRight;
	private final ISwerveModule backLeft;
	private final ISwerveModule backRight;

	private final Pigeon2 pigeon;
	private double lastYaw;
	private double gyroFactor;

	private double moduleX;
	private double moduleY;
	private double moduleUnitX;
	private double moduleUnitY;
	private double wheelDistance;

	private long lastTime;
	private double maxLinearAccel;
	private double maxRotateAccel;

	private double targetLinearAngle = 0;
	private double targetLinearSpeed = 0;
	private double targetRotate = 0;

	private double currentTargetLinearAngle = 0;
	private double currentTargetLinearSpeed = 0;
	private double currentTargetRotate = 0;

	private RobotPose pose;

	public FourCornerSwerveDrive(ISwerveModule frontLeft, ISwerveModule frontRight,
									ISwerveModule backLeft, ISwerveModule backRight,
									int pigeonChannel, SwerveDriveConfig config) {
		if (frontLeft == null)
			throw new IllegalArgumentException("frontLeft is null");
		if (frontRight == null)
			throw new IllegalArgumentException("frontRight is null");
		if (backLeft == null)
			throw new IllegalArgumentException("backLeft is null");
		if (backRight == null)
			throw new IllegalArgumentException("backRight is null");

		this.frontLeft = frontLeft;
		this.frontRight = frontRight;
		this.backLeft = backLeft;
		this.backRight = backRight;
		pigeon = new Pigeon2(pigeonChannel);

		pose = new RobotPose();

		configureDrive(config);
	}

	// Written by Michael Kaatz (2022)
	@Override
	public void setTargetVelocity(double linearAngle, double linearSpeed, double rotate) {
		targetLinearAngle = (linearAngle % 1 + 1) % 1;
		targetLinearSpeed = linearSpeed;
		targetRotate = rotate;
	}

	@Override
	public double[] getAlignments() {
		return new double[] { frontLeft.getAngleOffset(), frontRight.getAngleOffset(),
								backLeft.getAngleOffset(), backRight.getAngleOffset() };
	}

	@Override
	public void setAlignmentsAbsolute(double[] alignments) {
		if (alignments == null || alignments.length < 4)
			throw new IllegalArgumentException("length is less than 4");
		frontLeft.setAngleOffsetAbsolute(alignments[0]);
		frontRight.setAngleOffsetAbsolute(alignments[1]);
		backLeft.setAngleOffsetAbsolute(alignments[2]);
		backRight.setAngleOffsetAbsolute(alignments[3]);
	}

	@Override
	public void setAlignmentsRelative(double[] alignments) {
		if (alignments == null || alignments.length < 4)
			throw new IllegalArgumentException("length is less than 4");
		frontLeft.setAngleOffsetRelative(alignments[0]);
		frontRight.setAngleOffsetRelative(alignments[1]);
		backLeft.setAngleOffsetRelative(alignments[2]);
		backRight.setAngleOffsetRelative(alignments[3]);
	}

	@Override
	public void align() {
		frontLeft.align();
		frontRight.align();
		backLeft.align();
		backRight.align();
	}

	@Override
	public void configure(SwerveDriveConfig config) {
		frontLeft.configure(config.moduleConfig);
		frontRight.configure(config.moduleConfig);
		backLeft.configure(config.moduleConfig);
		backRight.configure(config.moduleConfig);
		configureDrive(config);
	}

	private void configureDrive(SwerveDriveConfig config) {
		gyroFactor = config.gyroFactor;

		moduleX = config.width / 2;
		moduleY = config.length / 2;
		wheelDistance = config.wheelDistance;

		double divisor = Math.sqrt(moduleX * moduleX + moduleY * moduleY);
		this.moduleUnitX = moduleX / divisor;
		this.moduleUnitY = moduleY / divisor;

		maxLinearAccel = config.maxLinearAccel;
		maxRotateAccel = config.maxRotateAccel;
		lastTime = System.currentTimeMillis();
	}

	@Override
	public double getTargetLinearAngle() {
		return targetLinearAngle;
	}

	@Override
	public double getTargetLinearSpeed() {
		return targetLinearSpeed;
	}

	@Override
	public double getTargetRotate() {
		return targetRotate;
	}

	@Override
	public RobotPose getPose() {
		return pose;
	}

	@Override
	public void tick() {
		long currentTime = System.currentTimeMillis();
		double timeDeltaSeconds = (currentTime - lastTime) / 1000d;
		lastTime = currentTime;

		{
			double targetX = Math.sin(targetLinearAngle * Math.PI * 2) * targetLinearSpeed;
			double targetY = Math.cos(targetLinearAngle * Math.PI * 2) * targetLinearSpeed;
			double currentX = Math.sin(currentTargetLinearAngle * Math.PI * 2) * currentTargetLinearSpeed;
			double currentY = Math.cos(currentTargetLinearAngle * Math.PI * 2) * currentTargetLinearSpeed;
			double translatedTargetX = targetX - currentX;
			double translatedTargetY = targetY - currentY;

			double accelAngleRadians = Math.atan2(translatedTargetY, translatedTargetX);
			double deltaX = Math.cos(accelAngleRadians) * maxLinearAccel * timeDeltaSeconds;
			double deltaY = Math.sin(accelAngleRadians) * maxLinearAccel * timeDeltaSeconds;

			currentX += Math.signum(translatedTargetX) * Math.min(Math.abs(deltaX), Math.abs(translatedTargetX));
			currentY += Math.signum(translatedTargetY) * Math.min(Math.abs(deltaY), Math.abs(translatedTargetY));
			currentTargetLinearAngle = -Math.atan2(currentY, currentX) / Math.PI / 2 + 0.25;
			currentTargetLinearSpeed = Math.sqrt(currentX * currentX + currentY * currentY);

			double deltaRotate = targetRotate - currentTargetRotate;
			currentTargetRotate += Math.signum(deltaRotate) * Math.min(maxRotateAccel * timeDeltaSeconds, Math.abs(deltaRotate));
		}

		double yaw = pigeon.getYaw() / 360;
		double yawDiff = -(yaw - lastYaw);
		lastYaw = yaw;

		{
			double effectiveLinearAngle = currentTargetLinearAngle;
			double effectiveLinearSpeed = currentTargetLinearSpeed;
			double effectiveRotate = currentTargetRotate;

			if (effectiveRotate == 0 && effectiveLinearSpeed != 0)
				effectiveRotate = -yawDiff * 360 * gyroFactor;

			double[] frontLeftVelocity = calculateModuleVelocity(effectiveLinearAngle, effectiveLinearSpeed, effectiveRotate,
																	-moduleUnitX, moduleUnitY);
			double[] frontRightVelocity = calculateModuleVelocity(effectiveLinearAngle, effectiveLinearSpeed, effectiveRotate,
																	moduleUnitX, moduleUnitY);
			double[] backLeftVelocity = calculateModuleVelocity(effectiveLinearAngle, effectiveLinearSpeed, effectiveRotate,
																	-moduleUnitX, -moduleUnitY);
			double[] backRightVelocity = calculateModuleVelocity(effectiveLinearAngle, effectiveLinearSpeed, effectiveRotate,
																	moduleUnitX, -moduleUnitY);

			// A motor can only go at 100% speed so we have to reduce them if one goes faster.
			double maxSpeed = 0;
			if (Math.abs(frontLeftVelocity[1]) > maxSpeed)
				maxSpeed = Math.abs(frontLeftVelocity[1]);
			if (Math.abs(frontRightVelocity[1]) > maxSpeed)
				maxSpeed = Math.abs(frontRightVelocity[1]);
			if (Math.abs(backLeftVelocity[1]) > maxSpeed)
				maxSpeed = Math.abs(backLeftVelocity[1]);
			if (Math.abs(backRightVelocity[1]) > maxSpeed)
				maxSpeed = Math.abs(backRightVelocity[1]);

			if (maxSpeed > 1) {
				frontLeftVelocity[1] /= maxSpeed;
				frontRightVelocity[1] /= maxSpeed;
				backLeftVelocity[1] /= maxSpeed;
				backRightVelocity[1] /= maxSpeed;
			}

			frontLeft.setTargetVelocity(frontLeftVelocity[0], frontLeftVelocity[1]);
			frontRight.setTargetVelocity(frontRightVelocity[0], frontRightVelocity[1]);
			backLeft.setTargetVelocity(backLeftVelocity[0], backLeftVelocity[1]);
			backRight.setTargetVelocity(backRightVelocity[0], backRightVelocity[1]);
		}

		frontLeft.tick();
		frontRight.tick();
		backLeft.tick();
		backRight.tick();

		if (frontLeft.getCurrentSpeed() > 0.01 || backRight.getCurrentSpeed() > 0.01) {
			double eps = 0.01;

			double xSum = 0;
			double ySum = 0;
			boolean ccw = false;
			int points = 0;
			double[] center;

			center = calculateZeroVelocityCenter(frontLeft, -moduleX, moduleY, frontRight, moduleX, moduleY);
			if (center[2] > eps && center[3] > eps && center[4] > eps) {
				xSum += center[0];
				ySum += center[1];
				ccw = center[5] == 1;
				points++;
			}
			center = calculateZeroVelocityCenter(backRight, moduleX, -moduleY, frontRight, moduleX, moduleY);
			if (center[2] > eps && center[3] > eps && center[4] > eps) {
				xSum += center[0];
				ySum += center[1];
				ccw = center[5] == 1;
				points++;
			}
			center = calculateZeroVelocityCenter(backLeft, -moduleX, -moduleY, backRight, moduleX, -moduleY);
			if (center[2] > eps && center[3] > eps && center[4] > eps) {
				xSum += center[0];
				ySum += center[1];
				ccw = center[5] == 1;
				points++;
			}
			center = calculateZeroVelocityCenter(backLeft, -moduleX, -moduleY, frontLeft, -moduleX, moduleY);
			if (center[2] > eps && center[3] > eps && center[4] > eps) {
				xSum += center[0];
				ySum += center[1];
				ccw = center[5] == 1;
				points++;
			}
			center = calculateZeroVelocityCenter(frontLeft, -moduleX, moduleY, backRight, moduleX, -moduleY);
			if (center[2] > eps && center[3] > eps && center[4] > eps) {
				xSum += center[0];
				ySum += center[1];
				ccw = center[5] == 1;
				points++;
			}
			center = calculateZeroVelocityCenter(backLeft, -moduleX, -moduleY, frontRight, moduleX, moduleY);
			if (center[2] > eps && center[3] > eps && center[4] > eps) {
				xSum += center[0];
				ySum += center[1];
				ccw = center[5] == 1;
				points++;
			}

			double xDiff = 0;
			double yDiff = 0;
			double angleDiff = 0;
			if (points > 0) {
				double centerX = xSum / points;
				double centerY = ySum / points;
				double positionAngleDeltaRadians;

				if (frontLeft.getCurrentSpeed() > backRight.getCurrentSpeed()) {
					double xFromModule = centerX + moduleX;
					double yFromModule = centerY - moduleY;
					double positionModuleRadius = Math.sqrt(xFromModule * xFromModule + yFromModule * yFromModule);
					positionAngleDeltaRadians = frontLeft.getCurrentSpeed() * wheelDistance
													/ positionModuleRadius * timeDeltaSeconds;
				} else {
					double xFromModule = centerX - moduleX;
					double yFromModule = centerY + moduleY;
					double positionModuleRadius = Math.sqrt(xFromModule * xFromModule + yFromModule * yFromModule);
					positionAngleDeltaRadians = backRight.getCurrentSpeed() * wheelDistance
													/ positionModuleRadius * timeDeltaSeconds;
				}

				double positionAngleRadians = Math.atan2(-centerY, -centerX);
				if (ccw)
					positionAngleRadians += positionAngleDeltaRadians;
				else
					positionAngleRadians -= positionAngleDeltaRadians;

				double positionRadius = Math.sqrt(centerX * centerX + centerY * centerY);
				double newX = -Math.cos(positionAngleRadians) * positionRadius;
				double newY = -Math.sin(positionAngleRadians) * positionRadius;

				xDiff = -newX + centerX;
				yDiff = -newY + centerY;
				angleDiff = positionAngleDeltaRadians / Math.PI / 2 * ((ccw) ? -1 : 1);
			} else {
				xDiff += Math.sin(frontLeft.getCurrentAngle() * Math.PI * 2)
							* frontLeft.getCurrentSpeed() * wheelDistance * timeDeltaSeconds;
				yDiff += Math.cos(frontLeft.getCurrentAngle() * Math.PI * 2)
							* frontLeft.getCurrentSpeed() * wheelDistance * timeDeltaSeconds;
				xDiff += Math.sin(frontRight.getCurrentAngle() * Math.PI * 2)
							* frontRight.getCurrentSpeed() * wheelDistance * timeDeltaSeconds;
				yDiff += Math.cos(frontRight.getCurrentAngle() * Math.PI * 2)
							* frontRight.getCurrentSpeed() * wheelDistance * timeDeltaSeconds;
				xDiff += Math.sin(backLeft.getCurrentAngle() * Math.PI * 2)
							* backLeft.getCurrentSpeed() * wheelDistance * timeDeltaSeconds;
				yDiff += Math.cos(backLeft.getCurrentAngle() * Math.PI * 2)
							* backLeft.getCurrentSpeed() * wheelDistance * timeDeltaSeconds;
				xDiff += Math.sin(backRight.getCurrentAngle() * Math.PI * 2)
							* backRight.getCurrentSpeed() * wheelDistance * timeDeltaSeconds;
				yDiff += Math.cos(backRight.getCurrentAngle() * Math.PI * 2)
							* backRight.getCurrentSpeed() * wheelDistance * timeDeltaSeconds;

				xDiff /= 4;
				yDiff /= 4;
			}

			if (xDiff != 0 || yDiff != 0) {
				double xyDiffPolarAngle = ((-Math.atan2(yDiff, xDiff) / Math.PI / 2 + 0.25 + pose.angle) % 1 + 1) % 1;
				double xyDiffPolarRadius = Math.sqrt(xDiff * xDiff + yDiff * yDiff);
				double fieldXDiff = Math.sin(xyDiffPolarAngle * Math.PI * 2) * xyDiffPolarRadius;
				double fieldYDiff = Math.cos(xyDiffPolarAngle * Math.PI * 2) * xyDiffPolarRadius;

				pose.x += fieldXDiff;
				pose.y += fieldYDiff;
				pose.linearAngle = ((-Math.atan2(fieldYDiff, fieldXDiff) / Math.PI / 2 + 0.25) % 1 + 1) % 1;
				pose.linearSpeed = xyDiffPolarRadius;
			} else {
				pose.linearAngle = 0;
				pose.linearSpeed = 0;
			}
			pose.angle += yawDiff;
			pose.rotate = angleDiff;
		}
	}

	// Written by Michael Kaatz (2022)
	private static double[] calculateModuleVelocity(double linearAngle, double linearSpeed, double rotate,
													double x, double y) {
		if (linearSpeed == 0 && rotate == 0)
			return new double[] { 0, 0 };

		// What we consider 0 degrees is actually 90 so the arctan args are actually
		// reversed.
		double turnAngle = Math.atan2(x, y) / Math.PI / 2 + 0.25;

		// I wrote this code awhile back and I have no idea how it works but it does.
		double x1 = Math.cos(turnAngle * Math.PI * 2) * rotate + Math.cos(linearAngle * Math.PI * 2) * linearSpeed;
		double y1 = Math.sin(turnAngle * Math.PI * 2) * rotate + Math.sin(linearAngle * Math.PI * 2) * linearSpeed;
		double targetAngle = Math.atan2(y1, x1) / Math.PI / 2;
		targetAngle = (targetAngle % 1 + 1) % 1;
		double targetSpeed = Math.sqrt(x1 * x1 + y1 * y1);

		// I don't like boxing so I use an array versus a Tuple.
		return new double[] { targetAngle, targetSpeed };
	}

	private static double[] calculateZeroVelocityCenter(ISwerveModule module1, double x1, double y1,
														ISwerveModule module2, double x2, double y2) {
		double yDiff = y2 - y1;
		double xDiff = x2 - x1;

		double module1Angle = module1.getCurrentAngle();
		double module2Angle = module2.getCurrentAngle();

		double moduleAngle = Math.atan2(yDiff, xDiff) / Math.PI / 2;
		double angle1 = 0;
		double angle2 = 0;

		boolean side = ((module2Angle - module1Angle) % 1 + 1) % 1 > 0.5;
		boolean down = ((module2Angle + moduleAngle) % 1 + 1) % 1 > 0.5;
		if (side) {
			// Left side
			if (down) {
				angle1 = 1 - module1Angle - moduleAngle;
				angle2 = module2Angle + moduleAngle - 0.5;
			} else {
				angle1 = 0.5 - module1Angle - moduleAngle;
				angle2 = module2Angle + moduleAngle;
			}
		} else {
			// Right side
			if (down) {
				angle1 = module1Angle + moduleAngle - 0.5;
				angle2 = 1 - module2Angle - moduleAngle;
			} else {
				angle1 = module1Angle + moduleAngle;
				angle2 = 0.5 - module2Angle - moduleAngle;
			}
		}
		angle1 = (Math.abs(angle1) % 1 + 1) % 1;
		angle2 = (angle2 % 1 + 1) % 1;
		double angle3 = 0.5 - angle1 - angle2;

		double distance3 = Math.sqrt(xDiff * xDiff + yDiff * yDiff);
		double radius1 = Math.sin(angle2 * Math.PI * 2) * distance3 / Math.sin(angle3 * Math.PI * 2);

		boolean ccw = side ^ down;
		double x;
		double y;
		if (ccw) {
			x = Math.sin((module1Angle + 0.25) * Math.PI * 2) * radius1 - x1;
			y = Math.cos((module1Angle + 0.25) * Math.PI * 2) * radius1 - y1;
		} else {
			x = Math.sin((module1Angle - 0.25) * Math.PI * 2) * radius1 - x1;
			y = Math.cos((module1Angle - 0.25) * Math.PI * 2) * radius1 - y1;
		}

		return new double[] { -x, -y, angle1, angle2, angle3, ccw ? 1 : 0 };
	}
}
