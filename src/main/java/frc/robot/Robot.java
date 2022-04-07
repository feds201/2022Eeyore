// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PersistentException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.IndicatorLights.LEDPattern;
import frc.robot.IndicatorLights.LEDZone;
import frc.robot.config.AbsoluteSteeringConfig;
import frc.robot.config.ClimberConfig;
import frc.robot.config.IntakeConfig;
import frc.robot.config.ShooterConfig;
import frc.robot.config.SwerveDriveConfig;
import frc.robot.profiles.ControlProfile;
import frc.robot.profiles.auton.BasicDualBallAutonProfile;
import frc.robot.profiles.auton.BasicSingleBallAutonProfile;
import frc.robot.profiles.auton.planned.AdvancedQuintAutonProfile;
import frc.robot.profiles.auton.planned.AutonPlan;
import frc.robot.profiles.teleop.DefaultDriverProfile;
import frc.robot.profiles.teleop.TestDriverProfile;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.ShooterHardware;
import frc.robot.shooter.ShooterMode;
import frc.robot.shooter.ShooterVision;
import frc.robot.swerve.FourCornerSwerveDrive;
import frc.robot.swerve.ISwerveDrive;
import frc.robot.swerve.ISwerveModule;
import frc.robot.swerve.SDSMk4FXModule;

public class Robot extends TimedRobot {

	public static final double PERIOD = 0.02;

	public static final String SWERVE_CONFIG_FILE = "swerveconfig.ini";
	public static final String SWERVE_ALIGNMENT_FILE = "swerve.ini";
	public static final String ABSOLUTE_STEERING_CONFIG_FILE = "swerveconfig.ini";
	public static final String INTAKE_CONFIG_FILE = "intakeconfig.ini";
	public static final String SHOOTER_CONFIG_FILE = "shooterconfig.ini";
	public static final String SHOOTER_VISION_POINTS_FILE = "shootervisionpoints.json";
	public static final String CLIMBER_CONFIG_FILE = "climberconfig.ini";

	public static final String QUINT_AUTON_PLAN_FILE = "quintautonplan.json";

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
	public static final int CLIMBER_LIMIT_PORT = 0;

	public static final int INDICATOR_LIGHTS_PORT = 0;
	public static final int INDICATOR_LIGHTS_COUNT = 120;
	public static final int INDICATOR_LIGHTS_ENDGAME_TIME = 40;

	private ControlProfile[] driverProfiles;
	private ControlProfile activeDriverProfile;
	private ControlProfile[] autonProfiles;
	private ControlProfile activeAutonProfile;

	private XboxController driverController;
	private XboxController operatorController;

	private ISwerveDrive swerveDrive;
	private BallPickup intake;
	private Shooter shooter;
	private Climber climber;
	private IndicatorLights indicatorLights;
	private UsbCamera driverCamera;

	private SwerveDriveConfig swerveDriveConfig;
	private AbsoluteSteeringConfig absoluteSteeringConfig;
	private IntakeConfig intakeConfig;
	private ShooterConfig shooterConfig;
	private ClimberConfig climberConfig;

	private AutonPlan quintAutonPlan;

	private SendableChooser<Integer> driverSelector = new SendableChooser<>();
	private SendableChooser<Integer> autonSelector = new SendableChooser<>();

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
		try {
			quintAutonPlan = AutonPlan.load(Filesystem.getDeployDirectory() + "/" + QUINT_AUTON_PLAN_FILE);
			System.out.println("Successfully loaded auton plans");
		} catch (IOException e) {
			System.err.println("Error loading auton plans");
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
													SWERVE_PIGEON, swerveDriveConfig);
		}

		intake = new BallPickup(PCM_CHANNEL, INTAKE_SOLENOID_DEPLOY, INTAKE_SOLENOID_STANDBY, INTAKE_MOTOR, intakeConfig);

		shooter = new Shooter(new ShooterHardware(SHOOTER_TOP_ID, SHOOTER_BOTTOM_ID, SHOOTER_FEEDER_ID,
								shooterConfig.hardwareConfig), new ShooterVision(shooterConfig.visionConfig),
								shooterConfig);

		climber = new Climber(CLIMBER_LEFT_ID, CLIMBER_RIGHT_ID, CLIMBER_LIMIT_PORT, climberConfig);

		indicatorLights = new IndicatorLights(INDICATOR_LIGHTS_PORT, INDICATOR_LIGHTS_COUNT);

		driverCamera = CameraServer.startAutomaticCapture();
		driverCamera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 15);

		driverController = new XboxController(0);
		operatorController = new XboxController(1);

		driverProfiles = new ControlProfile[] {
			new DefaultDriverProfile(driverController, operatorController,
										swerveDrive.getPose(), absoluteSteeringConfig),
			new TestDriverProfile(driverController)
		};
		activeDriverProfile = driverProfiles[0];
		driverSelector.setDefaultOption("Default", 0);
		driverSelector.addOption("Test", 1);

		autonProfiles = new ControlProfile[] {
			new BasicDualBallAutonProfile(PERIOD),
			new BasicSingleBallAutonProfile(PERIOD),
			new AdvancedQuintAutonProfile(PERIOD, swerveDrive.getPose(), quintAutonPlan)
		};
		activeAutonProfile = autonProfiles[0];
		autonSelector.setDefaultOption("Basic 2-Ball", 0);
		autonSelector.addOption("Basic 1-Ball", 1);
		autonSelector.addOption("5-Ball A", 2);

		SmartDashboard.putData(driverSelector);
		SmartDashboard.putData(autonSelector);
	}

	private static void configEncoderTalon(TalonSRX talon) {
		talon.configFactoryDefault();
		talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
		talon.configFeedbackNotContinuous(true, 50);
		talon.setSensorPhase(true);
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
		activeDriverProfile = driverProfiles[driverSelector.getSelected()];
		activeAutonProfile = autonProfiles[autonSelector.getSelected()];

		swerveDrive.tick();
		intake.tick();
		shooter.tick();
		climber.tick();

		if (isEnabled()) {
			if (shooter.getSpin()) {
				if (shooter.getMode() == ShooterMode.HIGH_GOAL_VISION) {
					if (shooter.hasTarget()) {
						if (shooter.isReady()) {
							indicatorLights.set(LEDZone.BASE, LEDPattern.SOLID, Color.kLime);
							indicatorLights.set(LEDZone.LEFT, LEDPattern.PASS, null);
							indicatorLights.set(LEDZone.RIGHT, LEDPattern.PASS, null);
						} else {
							indicatorLights.set(LEDZone.BASE, LEDPattern.PASS, null);
							indicatorLights.set(LEDZone.LEFT, LEDPattern.REVERSE, Color.kYellow);
							indicatorLights.set(LEDZone.RIGHT, LEDPattern.FORWARD, Color.kYellow);
						}
						indicatorLights.set(LEDZone.ACCENT, LEDPattern.SOLID, Color.kLime);
					} else {
						indicatorLights.set(LEDZone.BASE, LEDPattern.SOLID, Color.kRed);
						indicatorLights.set(LEDZone.ACCENT, LEDPattern.BLINK, Color.kYellow);
						indicatorLights.set(LEDZone.LEFT, LEDPattern.PASS, null);
						indicatorLights.set(LEDZone.RIGHT, LEDPattern.PASS, null);
					}
				} else if (shooter.getMode() == ShooterMode.LOW_GOAL) {
					if (shooter.isReady()) {
						indicatorLights.set(LEDZone.BASE, LEDPattern.SOLID, Color.kLime);
						indicatorLights.set(LEDZone.LEFT, LEDPattern.PASS, null);
						indicatorLights.set(LEDZone.RIGHT, LEDPattern.PASS, null);
					} else {
						indicatorLights.set(LEDZone.BASE, LEDPattern.PASS, null);
						indicatorLights.set(LEDZone.LEFT, LEDPattern.REVERSE, Color.kYellow);
						indicatorLights.set(LEDZone.RIGHT, LEDPattern.FORWARD, Color.kYellow);
					}
					indicatorLights.set(LEDZone.ACCENT, LEDPattern.SOLID, Color.kRed);
				} else if (shooter.getMode() == ShooterMode.EJECT) {
					if (shooter.isReady()) {
						indicatorLights.set(LEDZone.BASE, LEDPattern.SOLID, Color.kLime);
						indicatorLights.set(LEDZone.LEFT, LEDPattern.PASS, null);
						indicatorLights.set(LEDZone.RIGHT, LEDPattern.PASS, null);
					} else {
						indicatorLights.set(LEDZone.BASE, LEDPattern.PASS, null);
						indicatorLights.set(LEDZone.LEFT, LEDPattern.REVERSE, Color.kYellow);
						indicatorLights.set(LEDZone.RIGHT, LEDPattern.FORWARD, Color.kYellow);
					}
					indicatorLights.set(LEDZone.ACCENT, LEDPattern.BLINK, Color.kRed);
				}
				indicatorLights.set(LEDZone.TIPS, LEDPattern.PASS, null);
				indicatorLights.set(LEDZone.TOP, LEDPattern.PASS, null);
				indicatorLights.set(LEDZone.BOTTOM, LEDPattern.PASS, null);
				indicatorLights.set(LEDZone.CENTER, LEDPattern.PASS, null);
			} else {
				Alliance alliance = DriverStation.getAlliance();
				if (alliance == Alliance.Red)
					indicatorLights.set(LEDZone.BASE, LEDPattern.SOLID, Color.kRed);
				else if (alliance == Alliance.Blue)
					indicatorLights.set(LEDZone.BASE, LEDPattern.SOLID, Color.kBlue);
				else
					indicatorLights.set(LEDZone.BASE, LEDPattern.SOLID, Color.kGray);
				if (DriverStation.getMatchTime() > 0 &&
					DriverStation.getMatchTime() <= INDICATOR_LIGHTS_ENDGAME_TIME &&
					!DriverStation.isAutonomousEnabled()) {
					indicatorLights.set(LEDZone.ACCENT, LEDPattern.BLINK, Color.kOrange);
				} else {
					indicatorLights.set(LEDZone.ACCENT, LEDPattern.PASS, null);
				}
				indicatorLights.set(LEDZone.LEFT, LEDPattern.PASS, null);
				indicatorLights.set(LEDZone.RIGHT, LEDPattern.PASS, null);
				indicatorLights.set(LEDZone.TIPS, LEDPattern.PASS, null);
				indicatorLights.set(LEDZone.TOP, LEDPattern.PASS, null);
				indicatorLights.set(LEDZone.BOTTOM, LEDPattern.PASS, null);
				indicatorLights.set(LEDZone.CENTER, LEDPattern.PASS, null);
			}
		} else {
			Alliance alliance = DriverStation.getAlliance();
			if (alliance == Alliance.Red)
				indicatorLights.set(LEDZone.BASE, LEDPattern.SOLID, Color.kDarkRed);
			else if (alliance == Alliance.Blue)
				indicatorLights.set(LEDZone.BASE, LEDPattern.SOLID, Color.kDarkBlue);
			else
				indicatorLights.set(LEDZone.BASE, LEDPattern.SOLID, Color.kGray);
			indicatorLights.set(LEDZone.LEFT, LEDPattern.PASS, null);
			indicatorLights.set(LEDZone.RIGHT, LEDPattern.PASS, null);
			indicatorLights.set(LEDZone.ACCENT, LEDPattern.PASS, null);
			indicatorLights.set(LEDZone.TIPS, LEDPattern.PASS, null);
			indicatorLights.set(LEDZone.TOP, LEDPattern.PASS, null);
			indicatorLights.set(LEDZone.BOTTOM, LEDPattern.PASS, null);
			indicatorLights.set(LEDZone.CENTER, LEDPattern.PASS, null);
		}
		indicatorLights.tick();

		{
			NetworkTable table = NetworkTableInstance.getDefault().getTable("/swerve");
			table.getEntry("x").setDouble(swerveDrive.getPose().x);
			table.getEntry("y").setDouble(swerveDrive.getPose().y);
			table.getEntry("angle").setDouble(swerveDrive.getPose().angle * 360);
		}
		{
			NetworkTable table = NetworkTableInstance.getDefault().getTable("/shooter");
			double[] target = shooter.getTarget();
			table.getEntry("x").setDouble(target[0]);
			table.getEntry("y").setDouble(target[1]);
		}
	}

	@Override
	public void autonomousInit() {
		activeAutonProfile.reset();
	}

	@Override
	public void autonomousPeriodic() {
		applyProfile(activeAutonProfile);
	}

	@Override
	public void teleopInit() {
		activeDriverProfile.reset();
	}

	@Override
	public void teleopPeriodic() {
		applyProfile(activeDriverProfile);
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

		if (activeDriverProfile.getSwerveAlign()) {
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
		driverController.setRumble(RumbleType.kLeftRumble, activeDriverProfile.getSwerveAlignRumble() ? 1 : 0);

		if (activeDriverProfile.getConfigReload()) {
			try {
				loadConfigs();
				applyConfigs();
				System.err.println("Successfully reloaded subsystem configuration files");
			} catch (PersistentException e) {
				System.err.println("Error loading subsystem configuration files");
				System.err.println(e);
			}
		}
		driverController.setRumble(RumbleType.kRightRumble, activeDriverProfile.getConfigReloadRumble() ? 1 : 0);
	}

	private void applyProfile(ControlProfile profile) {
		profile.update();

		shooter.setMode(profile.getShooterMode());

		if (profile.getDecreaseShooterDistance())
			shooter.adjustDistance(-1);
		else if (profile.getIncreaseShooterDistance())
			shooter.adjustDistance(+1);

		shooter.setSpin(profile.getShooterSpin());
		double swerveRotate = profile.getSwerveRotate();
		if (shooter.getSpin() && shooter.hasTarget() &&
			shooter.getMode() == ShooterMode.HIGH_GOAL_VISION)
			swerveRotate = shooter.getYawCorrection();
		shooter.setFire(profile.getShooterFire());
		shooter.setUnjam(profile.getShooterUnjam());

		swerveDrive.setTargetVelocity(profile.getSwerveLinearAngle(),
										profile.getSwerveLinearSpeed(),
										swerveRotate);

		intake.setDeployed(profile.getIntakeDeploy());
		intake.setActive(profile.getIntakeActive());

		if (profile.getClimberUp())
			climber.setTargetPosition(1);
		else if (profile.getClimberDown())
			climber.setTargetPosition(-1);
		else if (profile.getClimberHigh())
			climber.setTargetPosition(2);
		else
			climber.setTargetPosition(0);

		if (profile.getOrientRobot())
			swerveDrive.getPose().angle = 0;
	}

	private void loadConfigs() throws PersistentException {
		swerveDriveConfig = SwerveDriveConfig.load(Filesystem.getDeployDirectory() + "/" + SWERVE_CONFIG_FILE);
		absoluteSteeringConfig = AbsoluteSteeringConfig.load(Filesystem.getDeployDirectory() + "/" + ABSOLUTE_STEERING_CONFIG_FILE);
		intakeConfig = IntakeConfig.load(Filesystem.getDeployDirectory() + "/" + INTAKE_CONFIG_FILE);
		shooterConfig = ShooterConfig.load(Filesystem.getDeployDirectory() + "/" + SHOOTER_CONFIG_FILE,
											Filesystem.getDeployDirectory() + "/" + SHOOTER_VISION_POINTS_FILE);
		climberConfig = ClimberConfig.load(Filesystem.getDeployDirectory() + "/" + CLIMBER_CONFIG_FILE);
	}

	private void applyConfigs() {
		swerveDrive.configure(swerveDriveConfig);
		intake.configure(intakeConfig);
		shooter.configure(shooterConfig);
		climber.configure(climberConfig);
	}
}
