package frc.robot.config;

import java.io.File;
import java.io.IOException;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PersistentException;

public class ShooterVisionConfig {

	public SlotConfiguration pid;

	public double lowGoalSpeedTop;
	public double lowGoalSpeedBottom;

	public ShooterVisionPoint[] points;
	public double distanceOffset;

	public static ShooterVisionConfig load(String configFile, String pointsFile) throws PersistentException {
		ShooterVisionConfig config = new ShooterVisionConfig();

		{
			NetworkTable table = NetworkTableInstance.getDefault().getTable("/config/shootervision");
			table.loadEntries(configFile);

			config.pid = new SlotConfiguration();
			config.pid.kP = table.getEntry("pid.kp").getDouble(0);
			config.pid.kI = table.getEntry("pid.ki").getDouble(0);
			config.pid.integralZone = table.getEntry("pid.izone").getDouble(0);
			config.pid.maxIntegralAccumulator = table.getEntry("pid.maxiacc").getDouble(0);
			config.pid.kD = table.getEntry("pid.kd").getDouble(0);
			config.pid.kF = table.getEntry("pid.kf").getDouble(0);

			config.lowGoalSpeedTop = table.getEntry("lowgoalspeedtop").getDouble(0);
			config.lowGoalSpeedBottom = table.getEntry("lowgoalspeedbottom").getDouble(0);

			config.distanceOffset = table.getEntry("distanceoffset").getDouble(0);
		}

		try {
			JsonNode speeds = new ObjectMapper().readTree(new File(pointsFile)).get("speeds");
			config.points = new ShooterVisionPoint[speeds.size()];
			for (int i = 0; i < speeds.size(); i++) {
				ShooterVisionPoint point = new ShooterVisionPoint();
				point.distance = speeds.get(i).get("distance").numberValue().doubleValue();
				point.topSpeed = speeds.get(i).get("topspeed").numberValue().doubleValue();
				point.bottomSpeed = speeds.get(i).get("bottomspeed").numberValue().doubleValue();
				config.points[i] = point;
			}
		} catch (IOException e) {
			throw new PersistentException("io exception reading json");
		}

		return config;
	}

	public static class ShooterVisionPoint {

		public double distance;
		public double topSpeed;
		public double bottomSpeed;
	}
}
