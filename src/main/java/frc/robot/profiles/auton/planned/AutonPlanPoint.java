package frc.robot.profiles.auton.planned;

import java.io.File;
import java.io.IOException;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

public class AutonPlanPoint {

	public double x;
	public double y;
	public double angle;

	public double linearCruise;
	public double rotateCruise;

	public double linearRamp;
	public double rotateRamp;

	public static AutonPlanPoint[] load(String file) throws IOException {
		JsonNode points = new ObjectMapper().readTree(new File(file)).get("points");
		AutonPlanPoint[] plan = new AutonPlanPoint[points.size()];
		for (int i = 0; i < points.size(); i++) {
			AutonPlanPoint point = new AutonPlanPoint();

			point.x = points.get(i).get("x").numberValue().doubleValue();
			point.y = points.get(i).get("y").numberValue().doubleValue();
			point.angle = points.get(i).get("angle").numberValue().doubleValue();

			point.linearCruise = points.get(i).get("linearcruise").numberValue().doubleValue();
			point.rotateCruise = points.get(i).get("rotatecruise").numberValue().doubleValue();

			point.linearRamp = points.get(i).get("linearramp").numberValue().doubleValue();
			point.rotateRamp = points.get(i).get("rotateramp").numberValue().doubleValue();

			plan[i] = point;
		}
		return plan;
	}
}
