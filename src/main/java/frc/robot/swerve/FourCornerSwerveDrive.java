package frc.robot.swerve;

import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.config.SwerveDriveConfig;

public class FourCornerSwerveDrive implements ISwerveDrive {

	private final ISwerveModule frontLeft;
	private final ISwerveModule frontRight;
	private final ISwerveModule backLeft;
	private final ISwerveModule backRight;

	private final Pigeon2 pigeon;
	private boolean straight = false;
	private double lastYaw;
	private double gyroFactor;

	private double moduleX;
	private double moduleY;
	private double moduleUnitX;
	private double moduleUnitY;

	private long lastTime;
	private double maxLinearAccel;
	private double maxRotateAccel;

	private double targetLinearAngle = 0;
	private double targetLinearSpeed = 0;
	private double targetRotate = 0;

	private double currentTargetLinearAngle = 0;
	private double currentTargetLinearSpeed = 0;
	private double currentTargetRotate = 0;

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

		configureDrive(config);
	}

	// Written by Michael Kaatz (2022)
	@Override
	public void setTargetVelocity(double linearAngle, double linearSpeed, double rotate) {
		if (rotate == 0 && targetLinearSpeed != 0) {
			if (straight) {
				double yaw = pigeon.getYaw();
				rotate = (yaw - lastYaw) * gyroFactor;
				lastYaw = yaw;
			} else {
				lastYaw = pigeon.getYaw();
				straight = true;
			}
		} else {
			straight = false;
		}

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

		{
			double effectiveLinearAngle = currentTargetLinearAngle;
			double effectiveLinearSpeed = currentTargetLinearSpeed;
			double effectiveRotate = currentTargetRotate;

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
}
