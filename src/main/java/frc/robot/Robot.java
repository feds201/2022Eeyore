// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PersistentException;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.config.SwerveDriveConfig;
import frc.robot.config.SwerveModuleConfig;
import frc.robot.profiles.DefaultDriverProfile;
import frc.robot.profiles.DriverProfile;
import frc.robot.swerve.FourCornerSwerveDrive;
import frc.robot.swerve.ISwerveDrive;
import frc.robot.swerve.ISwerveModule;
import frc.robot.swerve.SDSMk4FXModule;

public class Robot extends TimedRobot {

	public static final double SWERVE_MAX_RAMP = 1.0;
	public static final double SWERVE_GYRO_FACTOR = 1.0;

	public static final double SHOOTER_TOP_SPEED = 16425;
	public static final double SHOOTER_BOTTOM_SPEED = 9855;
	public static final double SHOOTER_FEEDER_SPEED = 0.25;
	public static final double SHOOTER_LOWER_THRESHOLD = 0.95;
	public static final double SHOOTER_UPPER_THRESHOLD = 1.05;

	public static final int SWERVE_FRONT_LEFT_STEER = 21;
	public static final int SWERVE_FRONT_LEFT_DRIVE = 22;
	public static final int SWERVE_FRONT_LEFT_ENCODER = 1;

	public static final int SWERVE_FRONT_RIGHT_STEER = 11;
	public static final int SWERVE_FRONT_RIGHT_DRIVE = 12;
	public static final int SWERVE_FRONT_RIGHT_ENCODER = 4;

	public static final int SWERVE_BACK_LEFT_STEER = 31;
	public static final int SWERVE_BACK_LEFT_DRIVE = 32;
	public static final int SWERVE_BACK_LEFT_ENCODER = 3;

	public static final int SWERVE_BACK_RIGHT_STEER = 41;
	public static final int SWERVE_BACK_RIGHT_DRIVE = 42;
	public static final int SWERVE_BACK_RIGHT_ENCODER = 2;

	public static final String SWERVE_ALIGNMENT_FILE = "swerve.ini";

	public static final int SHOOTER_TOP_ID = 60;
	public static final int SHOOTER_BOTTOM_ID = 61;
	public static final int SHOOTER_FEEDER_ID = 62;

	private final DriverProfile[] profiles = {
		new DefaultDriverProfile()
	};
	private DriverProfile activeProfile = profiles[0];

	private XboxController driverController;
	private XboxController operatorController;

	private ISwerveDrive swerveDrive;
	private ShooterVision shooterVision;
	private Shooter shooter;

	public Robot() {
		super(0.05);
	}

	@Override
	public void robotInit() {
		try
		{
			NetworkTableInstance.getDefault().getTable("swervealignment")
				.loadEntries(Filesystem.getOperatingDirectory() + "/" + SWERVE_ALIGNMENT_FILE);
			System.out.println("Successfully loaded swerve drive alignment");
		}
		catch (PersistentException e)
		{
			System.err.println("Error loading swerve drive alignment");
			System.err.println(e);
		}

		TalonSRX talon1 = new TalonSRX(SWERVE_FRONT_LEFT_ENCODER);
		TalonSRX talon2 = new TalonSRX(SWERVE_FRONT_RIGHT_ENCODER);
		TalonSRX talon3 = new TalonSRX(SWERVE_BACK_LEFT_ENCODER);
		TalonSRX talon4 = new TalonSRX(SWERVE_BACK_RIGHT_ENCODER);
		talon1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
		talon2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
		talon3.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
		talon4.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
		talon1.setSensorPhase(false);
		talon2.setSensorPhase(false);
		talon3.setSensorPhase(false);
		talon4.setSensorPhase(false);
		talon1.setInverted(false);
		talon2.setInverted(false);
		talon3.setInverted(false);
		talon4.setInverted(false);

		{
			NetworkTable table = NetworkTableInstance.getDefault().getTable("swervealignment");
			SlotConfiguration swervePID = new SlotConfiguration();
			swervePID.closedLoopPeriod = 1;
			swervePID.kP = 0.1;
			swervePID.kI = 0.000;
			swervePID.maxIntegralAccumulator = 0.000;
			swervePID.kD = 0.000;
			swervePID.kF = 0;
			SwerveDriveConfig driveConfig = new SwerveDriveConfig();
			driveConfig.moduleConfig = new SwerveModuleConfig();
			driveConfig.moduleConfig.pid = swervePID;
			driveConfig.moduleConfig.maxRamp = 1.0;
			driveConfig.moduleConfig.steerBrake = true;
			driveConfig.moduleConfig.steerCurrentLimitEnabled = true;
			driveConfig.moduleConfig.steerCurrentLimit = 25;
			driveConfig.moduleConfig.steerCurrentLimitTime = 0.5;
			driveConfig.moduleConfig.driveCurrentLimitEnabled = true;
			driveConfig.moduleConfig.driveCurrentLimit = 25;
			driveConfig.moduleConfig.driveCurrentLimitTime = 1.0;
			driveConfig.gyroFactor = 0.000;
			ISwerveModule frontLeft = new SDSMk4FXModule(SWERVE_FRONT_LEFT_STEER, SWERVE_FRONT_LEFT_DRIVE,
															SWERVE_FRONT_LEFT_ENCODER, table.getEntry("index0").getDouble(0),
															driveConfig.moduleConfig);
			ISwerveModule frontRight = new SDSMk4FXModule(SWERVE_FRONT_RIGHT_STEER, SWERVE_FRONT_RIGHT_DRIVE,
															SWERVE_FRONT_RIGHT_ENCODER, table.getEntry("index1").getDouble(0),
															driveConfig.moduleConfig);
			ISwerveModule backLeft = new SDSMk4FXModule(SWERVE_BACK_LEFT_STEER, SWERVE_BACK_LEFT_DRIVE,
															SWERVE_BACK_LEFT_ENCODER, table.getEntry("index2").getDouble(0),
															driveConfig.moduleConfig);
			ISwerveModule backRight = new SDSMk4FXModule(SWERVE_BACK_RIGHT_STEER, SWERVE_BACK_RIGHT_DRIVE,
															SWERVE_BACK_RIGHT_ENCODER, table.getEntry("index3").getDouble(0),
															driveConfig.moduleConfig);
			swerveDrive = new FourCornerSwerveDrive(frontLeft, frontRight, backLeft, backRight,
													new ADXRS450_Gyro(Port.kOnboardCS0), 30, 30, driveConfig);
		}

		SlotConfiguration shooterVisionPID = new SlotConfiguration();
		shooterVisionPID.kP = 0.005;
		shooterVisionPID.kI = 0.000;
		shooterVisionPID.maxIntegralAccumulator = 0.000;
		shooterVisionPID.kD = 0.000;
		shooterVision = new ShooterVision(shooterVisionPID);

		SlotConfiguration shooterPID = new SlotConfiguration();
		shooterPID.closedLoopPeriod = 1;
		shooterPID.kP = 0.001;
		shooterPID.kI = 0.001;
		shooterPID.maxIntegralAccumulator = 4096;
		shooterPID.integralZone = 4096;
		shooterPID.kD = 0.000;
		shooterPID.kF = 1023 / Shooter.FALCON_MAX_SPEED;
		shooter = new Shooter(SHOOTER_TOP_ID, SHOOTER_BOTTOM_ID, SHOOTER_FEEDER_ID,
								SHOOTER_LOWER_THRESHOLD, SHOOTER_UPPER_THRESHOLD,
								SHOOTER_FEEDER_SPEED, shooterPID);

		driverController = new XboxController(0);
		operatorController = new XboxController(1);
	}

	@Override
	public void robotPeriodic() {
		swerveDrive.tick();
		shooterVision.tick();
		shooter.tick();
	}

	@Override
	public void autonomousInit() {}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {}

	@Override
	public void teleopPeriodic() {
		activeProfile.update(driverController, operatorController);

		double swerveRotate = activeProfile.getSwerveRotate();
		if (activeProfile.getShooterRev()) {
			shooterVision.setActive(true);
			shooter.setSpeed(SHOOTER_TOP_SPEED, SHOOTER_BOTTOM_SPEED);
			if (shooterVision.hasTarget())
				swerveRotate = shooterVision.getCorrection();
		} else {
			shooterVision.setActive(false);
			shooter.setSpeed(0, 0);
		}
		shooter.setFire(activeProfile.getShooterFire());

		swerveDrive.setTargetVelocity(activeProfile.getSwerveLinearAngle(),
										activeProfile.getSwerveLinearSpeed(),
										swerveRotate);
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void testInit() {}

	@Override
	public void testPeriodic() {
		teleopPeriodic();

		if (activeProfile.getSwerveAlignSet()) {
			swerveDrive.align();
			double[] alignments = swerveDrive.getAlignments();
			try {
				NetworkTable table = NetworkTableInstance.getDefault().getTable("swervealignment");
				table.getEntry("index0").setDouble(alignments[0]);
				table.getEntry("index1").setDouble(alignments[1]);
				table.getEntry("index2").setDouble(alignments[2]);
				table.getEntry("index3").setDouble(alignments[3]);
				table.saveEntries(Filesystem.getOperatingDirectory() + "/" + SWERVE_ALIGNMENT_FILE);
				System.out.println("Successfully saved swerve drive alignment");
			} catch (PersistentException e) {
				System.err.println("Error saving swerve drive alignment");
				System.err.println(e);
			}
		}
		driverController.setRumble(RumbleType.kLeftRumble, activeProfile.getSwerveAlignRumble() ? 1 : 0);
	}
}
