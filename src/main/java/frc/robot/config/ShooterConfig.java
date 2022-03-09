package frc.robot.config;

import java.io.File;
import java.io.IOException;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PersistentException;
import frc.robot.config.ShooterVisionConfig.ShooterVisionPoint;

public class ShooterConfig {

	public ShooterHardwareConfig hardwareConfig;
	public ShooterVisionConfig visionConfig;

	public double lowGoalSpeedTop;
	public double lowGoalSpeedBottom;

	public double ejectSpeedTop;
	public double ejectSpeedBottom;

	public static ShooterConfig load(String file, String pointsFile) throws PersistentException {
		NetworkTable table = NetworkTableInstance.getDefault().getTable("/config/shooter");
		ShooterConfig config = new ShooterConfig();
		table.loadEntries(file);

		{
			config.lowGoalSpeedTop = table.getEntry("lowgoalspeedtop").getDouble(0);
			config.lowGoalSpeedBottom = table.getEntry("lowgoalspeedbottom").getDouble(0);

			config.ejectSpeedTop = table.getEntry("ejectspeedtop").getDouble(0);
			config.ejectSpeedBottom = table.getEntry("ejectspeedbottom").getDouble(0);
		}

		{
			config.hardwareConfig = new ShooterHardwareConfig();

			config.hardwareConfig.topPid = new SlotConfiguration();
			config.hardwareConfig.topPid.closedLoopPeriod = (int) table.getEntry("hardware.toppid.period").getDouble(1);
			config.hardwareConfig.topPid.kP = table.getEntry("hardware.toppid.kp").getDouble(0);
			config.hardwareConfig.topPid.kI = table.getEntry("hardware.toppid.ki").getDouble(0);
			config.hardwareConfig.topPid.integralZone = table.getEntry("hardware.toppid.izone").getDouble(0);
			config.hardwareConfig.topPid.maxIntegralAccumulator = table.getEntry("hardware.toppid.maxiacc").getDouble(0);
			config.hardwareConfig.topPid.kD = table.getEntry("hardware.toppid.kd").getDouble(0);
			config.hardwareConfig.topPid.kF = table.getEntry("hardware.toppid.kf").getDouble(0);

			config.hardwareConfig.bottomPid = new SlotConfiguration();
			config.hardwareConfig.bottomPid.closedLoopPeriod = (int) table.getEntry("hardware.bottompid.period").getDouble(1);
			config.hardwareConfig.bottomPid.kP = table.getEntry("hardware.bottompid.kp").getDouble(0);
			config.hardwareConfig.bottomPid.kI = table.getEntry("hardware.bottompid.ki").getDouble(0);
			config.hardwareConfig.bottomPid.integralZone = table.getEntry("hardware.bottompid.izone").getDouble(0);
			config.hardwareConfig.bottomPid.maxIntegralAccumulator = table.getEntry("hardware.bottompid.maxiacc").getDouble(0);
			config.hardwareConfig.bottomPid.kD = table.getEntry("hardware.bottompid.kd").getDouble(0);
			config.hardwareConfig.bottomPid.kF = table.getEntry("hardware.bottompid.kf").getDouble(0);

			config.hardwareConfig.fireThresholdLower = table.getEntry("hardware.firethresholdlower").getDouble(0);
			config.hardwareConfig.fireThresholdUpper = table.getEntry("hardware.firethresholdupper").getDouble(2);
			config.hardwareConfig.feederSpeed = table.getEntry("hardware.feederspeed").getDouble(0);
			config.hardwareConfig.shooterBrake = table.getEntry("hardware.shooterbrake").getBoolean(false);

			config.hardwareConfig.shooterCurrentLimitEnabled = table.getEntry("hardware.shootercurrentlimitenabled").getBoolean(false);
			config.hardwareConfig.shooterCurrentLimit = table.getEntry("hardware.shootercurrentlimit").getDouble(0);
			config.hardwareConfig.shooterCurrentLimitTime = table.getEntry("hardware.shootercurrentlimittime").getDouble(0);

			config.hardwareConfig.feederCurrentLimitEnabled = table.getEntry("hardware.feedercurrentlimitenabled").getBoolean(false);
			config.hardwareConfig.feederCurrentLimit = table.getEntry("hardware.feedercurrentlimit").getDouble(0);
			config.hardwareConfig.feederCurrentLimitTime = table.getEntry("hardware.feedercurrentlimittime").getDouble(0);
		}

		try {
			config.visionConfig = new ShooterVisionConfig();

			config.visionConfig.pid = new SlotConfiguration();
			config.visionConfig.pid.closedLoopPeriod = (int) table.getEntry("vision.pid.period").getDouble(1);
			config.visionConfig.pid.kP = table.getEntry("vision.pid.kp").getDouble(0);
			config.visionConfig.pid.kI = table.getEntry("vision.pid.ki").getDouble(0);
			config.visionConfig.pid.integralZone = table.getEntry("vision.pid.izone").getDouble(0);
			config.visionConfig.pid.maxIntegralAccumulator = table.getEntry("vision.pid.maxiacc").getDouble(0);
			config.visionConfig.pid.kD = table.getEntry("vision.pid.kd").getDouble(0);
			config.visionConfig.pid.kF = table.getEntry("vision.pid.kf").getDouble(0);

			config.visionConfig.distanceOffset = table.getEntry("vision.distanceoffset").getDouble(0);

			JsonNode speeds = new ObjectMapper().readTree(new File(pointsFile)).get("speeds");
			config.visionConfig.points = new ShooterVisionPoint[speeds.size()];
			for (int i = 0; i < speeds.size(); i++) {
				ShooterVisionPoint point = new ShooterVisionPoint();
				point.distance = speeds.get(i).get("distance").numberValue().doubleValue();
				point.topSpeed = speeds.get(i).get("topspeed").numberValue().doubleValue();
				point.bottomSpeed = speeds.get(i).get("bottomspeed").numberValue().doubleValue();
				config.visionConfig.points[i] = point;
			}
		} catch (IOException e) {
			throw new PersistentException("io exception reading json");
		}

		return config;
	}
}
