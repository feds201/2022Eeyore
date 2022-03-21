package frc.robot.profiles.auton.planned;

import java.io.File;
import java.io.IOException;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

public class AutonPlan {

	public AutonStartPoint start;
	public AutonPlanPoint[] points;

	public static AutonPlan load(String file) throws IOException {
		JsonNode root = new ObjectMapper().readTree(new File(file));
		AutonPlan result = new AutonPlan();
		{
			JsonNode node = root.get("start");
			AutonStartPoint point = new AutonStartPoint();

			point.x = node.get("x").numberValue().doubleValue();
			point.y = node.get("y").numberValue().doubleValue();
			point.angle = node.get("angle").numberValue().doubleValue();

			result.start = point;
		}
		{
			JsonNode array = root.get("points");
			AutonPlanPoint[] plan = new AutonPlanPoint[array.size()];
			for (int i = 0; i < array.size(); i++) {
				AutonPlanPoint point = new AutonPlanPoint();

				point.x = array.get(i).get("x").numberValue().doubleValue();
				point.y = array.get(i).get("y").numberValue().doubleValue();
				point.angle = array.get(i).get("angle").numberValue().doubleValue();

				point.linearCruise = array.get(i).get("linearcruise").numberValue().doubleValue();
				point.rotateCruise = array.get(i).get("rotatecruise").numberValue().doubleValue();

				point.linearRamp = array.get(i).get("linearramp").numberValue().doubleValue();
				point.rotateRamp = array.get(i).get("rotateramp").numberValue().doubleValue();

				plan[i] = point;
			}
			result.points = plan;
		}
		return result;
	}

	public static class AutonStartPoint {

		public double x;
		public double y;
		public double angle;
	}

	public static class AutonPlanPoint {

		public double x;
		public double y;
		public double angle;

		public double linearCruise;
		public double rotateCruise;

		public double linearRamp;
		public double rotateRamp;
	}
}
