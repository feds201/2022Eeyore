// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PersistentException;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.config.IntakeConfig;
import frc.robot.config.ClimberConfig;
import frc.robot.config.ShooterConfig;
import frc.robot.config.ShooterVisionConfig;
import frc.robot.config.SwerveDriveConfig;
import frc.robot.profiles.DefaultDriverProfile;
import frc.robot.profiles.DriverProfile;
import frc.robot.profiles.MichaelsDriverProfile;
import frc.robot.profiles.TestDriverProfile;
import frc.robot.swerve.FourCornerSwerveDrive;
import frc.robot.swerve.ISwerveDrive;
import frc.robot.swerve.ISwerveModule;
import frc.robot.swerve.SDSMk4FXModule;

public class Robot extends TimedRobot {

	public static final double PERIOD = 0.05;

	public static final String SWERVE_CONFIG_FILE = "swerveconfig.ini";
	public static final String SWERVE_ALIGNMENT_FILE = "swerve.ini";
	public static final String INTAKE_CONFIG_FILE = "intakeconfig.ini";
	public static final String SHOOTER_VISION_CONFIG_FILE = "shootervisionconfig.ini";
	public static final String SHOOTER_VISION_POINTS_FILE = "shootervisionpoints.json";
	public static final String SHOOTER_CONFIG_FILE = "shooterconfig.ini";
	public static final String CLIMBER_CONFIG_FILE = "climberconfig.ini";

	public static final int PCM_CHANNEL = 8;

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

	public static final int SWERVE_PIGEON = 1;

	public static final int INTAKE_SOLENOID_DEPLOY = 0;
	public static final int INTAKE_SOLENOID_STANDBY = 1;
	public static final int INTAKE_MOTOR = 6;

	public static final int SHOOTER_TOP_ID = 60;
	public static final int SHOOTER_BOTTOM_ID = 61;
	public static final int SHOOTER_FEEDER_ID = 62;

	public static final int CLIMBER_LEFT_ID = 51;
	public static final int CLIMBER_RIGHT_ID = 52;

	public static final int INDICATOR_LIGHTS_PORT = 0;
	public static final int INDICATOR_LIGHTS_COUNT = 104;

	private DriverProfile[] profiles;
	private DriverProfile activeProfile;

	private XboxController driverController;
	private XboxController operatorController;

	private ISwerveDrive swerveDrive;
	private BallPickup intake;
	private ShooterVision shooterVision;
	private Shooter shooter;
	private Climber climber;
	private IndicatorLights indicatorLights;
	private UsbCamera driverCamera;

	private SwerveDriveConfig swerveDriveConfig;
	private IntakeConfig intakeConfig;
	private ShooterVisionConfig shooterVisionConfig;
	private ShooterConfig shooterConfig;
	private ClimberConfig climberConfig;

	public Robot() {
		super(PERIOD);
	}

	@Override
	public void robotInit() {
		try {
			NetworkTableInstance.getDefault().getTable("swervealignment")
				.loadEntries(Filesystem.getOperatingDirectory() + "/" + SWERVE_ALIGNMENT_FILE);
			System.out.println("Successfully loaded swerve drive alignment");
		} catch (PersistentException e) {
			System.err.println("Error loading swerve drive alignment");
			System.err.println(e);
		}
		try {
			loadConfigs();
			System.out.println("Successfully loaded subsystem configuration files");
		} catch (PersistentException e) {
			System.err.println("Error loading subsystem configuration files");
			System.err.println(e);
		}

		TalonSRX talon1 = new TalonSRX(SWERVE_FRONT_LEFT_ENCODER);
		TalonSRX talon2 = new TalonSRX(SWERVE_FRONT_RIGHT_ENCODER);
		TalonSRX talon3 = new TalonSRX(SWERVE_BACK_LEFT_ENCODER);
		TalonSRX talon4 = new TalonSRX(SWERVE_BACK_RIGHT_ENCODER);
		configEncoderTalon(talon1);
		configEncoderTalon(talon2);
		configEncoderTalon(talon3);
		configEncoderTalon(talon4);

		{
			NetworkTable table = NetworkTableInstance.getDefault().getTable("swervealignment");
			ISwerveModule frontLeft = new SDSMk4FXModule(SWERVE_FRONT_LEFT_STEER, SWERVE_FRONT_LEFT_DRIVE,
															SWERVE_FRONT_LEFT_ENCODER, table.getEntry("index0").getDouble(0),
															swerveDriveConfig.moduleConfig);
			ISwerveModule frontRight = new SDSMk4FXModule(SWERVE_FRONT_RIGHT_STEER, SWERVE_FRONT_RIGHT_DRIVE,
															SWERVE_FRONT_RIGHT_ENCODER, table.getEntry("index1").getDouble(0),
															swerveDriveConfig.moduleConfig);
			ISwerveModule backLeft = new SDSMk4FXModule(SWERVE_BACK_LEFT_STEER, SWERVE_BACK_LEFT_DRIVE,
															SWERVE_BACK_LEFT_ENCODER, table.getEntry("index2").getDouble(0),
															swerveDriveConfig.moduleConfig);
			ISwerveModule backRight = new SDSMk4FXModule(SWERVE_BACK_RIGHT_STEER, SWERVE_BACK_RIGHT_DRIVE,
															SWERVE_BACK_RIGHT_ENCODER, table.getEntry("index3").getDouble(0),
															swerveDriveConfig.moduleConfig);
			swerveDrive = new FourCornerSwerveDrive(frontLeft, frontRight, backLeft, backRight,
													SWERVE_PIGEON, 30, 30, PERIOD, swerveDriveConfig);
		}

		intake = new BallPickup(PCM_CHANNEL, INTAKE_SOLENOID_DEPLOY, INTAKE_SOLENOID_STANDBY, INTAKE_MOTOR, intakeConfig);

		shooterVision = new ShooterVision(PERIOD, shooterVisionConfig);
		shooter = new Shooter(SHOOTER_TOP_ID, SHOOTER_BOTTOM_ID, SHOOTER_FEEDER_ID,
								shooterConfig);

		climber = new Climber(CLIMBER_LEFT_ID, CLIMBER_RIGHT_ID, climberConfig);

		indicatorLights = new IndicatorLights(INDICATOR_LIGHTS_PORT, INDICATOR_LIGHTS_COUNT);

		driverCamera = CameraServer.startAutomaticCapture();
		driverCamera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 15);

		driverController = new XboxController(0);
		operatorController = new XboxController(1);

		profiles = new DriverProfile[] {
			new DefaultDriverProfile(driverController, operatorController),
			new TestDriverProfile(driverController),
			new MichaelsDriverProfile(driverController, operatorController)
		};
		activeProfile = profiles[0];
	}

	private static void configEncoderTalon(TalonSRX talon) {
		talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
		talon.setSensorPhase(false);
		talon.setInverted(false);
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255);
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
	}

	@Override
	public void robotPeriodic() {
		swerveDrive.tick();
		intake.tick();
		shooterVision.tick();
		shooter.tick();
		climber.tick();
		indicatorLights.tick();
	}

	@Override
	public void autonomousInit() {}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {}

	@Override
	public void teleopPeriodic() {
		applyProfile(activeProfile);
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

		if (activeProfile.getSwerveAlign()) {
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

		if (activeProfile.getConfigReload()) {
			try {
				loadConfigs();
				applyConfigs();
				System.err.println("Successfully reloaded subsystem configuration files");
			} catch (PersistentException e) {
				System.err.println("Error loading subsystem configuration files");
				System.err.println(e);
			}
		}
		driverController.setRumble(RumbleType.kRightRumble, activeProfile.getConfigReloadRumble() ? 1 : 0);
	}

	private void applyProfile(DriverProfile profile) {
		if (profile.getDecreaseShooterDistance())
			shooterVision.adjustDistance(-1);
		else if (profile.getIncreaseShooterDistance())
			shooterVision.adjustDistance(+1);

		double swerveRotate = profile.getSwerveRotate();
		if (profile.getShooterRev()) {
			shooterVision.setActive(true);
			double[] shooterSpeeds = shooterVision.getShooterSpeeds();
			shooter.setSpeed(shooterSpeeds[0], shooterSpeeds[1]);
			if (shooterVision.hasTarget())
				swerveRotate = shooterVision.getYawCorrection();
		} else {
			shooterVision.setActive(false);
			shooter.setSpeed(0, 0);
		}
		shooter.setFire(profile.getShooterFire());

		swerveDrive.setTargetVelocity(profile.getSwerveLinearAngle(),
										profile.getSwerveLinearSpeed(),
										swerveRotate);

		intake.setDeployed(profile.getIntakeDeploy());
		intake.setActive(profile.getIntakeActive());

		if (profile.getClimberUp())
			climber.setTargetPosition(1);
		else if (profile.getClimberDown())
			climber.setTargetPosition(-1);
		else
			climber.setTargetPosition(0);
	}

	private void loadConfigs() throws PersistentException {
		swerveDriveConfig = SwerveDriveConfig.load(Filesystem.getDeployDirectory() + "/" + SWERVE_CONFIG_FILE);
		intakeConfig = IntakeConfig.load(Filesystem.getDeployDirectory() + "/" + INTAKE_CONFIG_FILE);
		shooterVisionConfig = ShooterVisionConfig.load(Filesystem.getDeployDirectory() + "/" + SHOOTER_VISION_CONFIG_FILE,
														Filesystem.getDeployDirectory() + "/" + SHOOTER_VISION_POINTS_FILE);
		shooterConfig = ShooterConfig.load(Filesystem.getDeployDirectory() + "/" + SHOOTER_CONFIG_FILE);
		climberConfig = ClimberConfig.load(Filesystem.getDeployDirectory() + "/" + CLIMBER_CONFIG_FILE);
	}

	private void applyConfigs() {
		swerveDrive.configure(swerveDriveConfig);
		intake.configure(intakeConfig);
		shooterVision.configure(shooterVisionConfig);
		shooter.configure(shooterConfig);
		climber.configure(climberConfig);
	}
}
