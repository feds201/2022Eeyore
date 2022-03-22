package frc.robot.vision;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Subsystem;
import frc.robot.config.BallVisionConfig;

public class BallVision implements Subsystem {

	private final BallVisionThread thread;
	private final Object lock = new Object();

	private boolean hasTarget = false;
	private double error;

	private final CvSink input;
	private final RedBallPipeline red;
	private final BlueBallPipeline blue;

	private Alliance alliance = Alliance.Red;

	private SlotConfiguration pid;
	private long lastTime;
	private double iacc = 0;
	private double lastErr = 0;

	public BallVision(VideoSource input, RedBallPipeline red, BlueBallPipeline blue, BallVisionConfig config) {
		if (input == null)
			throw new IllegalArgumentException("input is null");
		if (red == null)
			throw new IllegalArgumentException("red is null");
		if (blue == null)
			throw new IllegalArgumentException("blue is null");
		this.input = new CvSink("Auton CvSink");
		this.red = red;
		this.blue = blue;

		this.input.setSource(input);
		configure(config);

		thread = new BallVisionThread();
		thread.start();
	}

	public void setAlliance(Alliance alliance) {
		synchronized (lock) {
			this.alliance = alliance;
		}
	}

	public boolean hasTarget() {
		synchronized (lock) {
			return hasTarget;
		}
	}

	public double getCorrection() {
		long currentTime = System.currentTimeMillis();
		double timeDeltaSeconds = (currentTime - lastTime) / 1000d;
		lastTime = currentTime;

		double error;
		synchronized (lock) {
			error = this.error;
		}
		if (Math.abs(error) <= pid.integralZone) {
			iacc += error * timeDeltaSeconds;
			if (Math.abs(iacc) > pid.maxIntegralAccumulator)
				iacc = Math.signum(iacc) * pid.maxIntegralAccumulator;
		}
		else
			iacc = 0;
		double output = error * pid.kP + iacc * pid.kI + (error - lastErr) / timeDeltaSeconds * pid.kD;
		lastErr = error;
		return output;
	}

	public void configure(BallVisionConfig config) {
		pid = config.pid;

		lastTime = System.currentTimeMillis();
		iacc = 0;
		lastErr = 0;
	}

	private class BallVisionThread extends Thread {

		public BallVisionThread() {
			setDaemon(true);
		}

		@Override
		public void run() {
			Mat mat = new Mat();
			Alliance localAlliance = alliance;
			while (!isInterrupted()) {
				Rect lowestRect = null;
				long result = input.grabFrame(mat);
				if (result != 0) {
					ArrayList<MatOfPoint> list;
					if (localAlliance == Alliance.Red) {
						red.process(mat);
						list = red.filterContoursOutput();
					} else {
						blue.process(mat);
						list = blue.filterContoursOutput();
					}
					if (!list.isEmpty()) {
						for (MatOfPoint contour : list) {
							Rect rect = Imgproc.boundingRect(contour);
							if (lowestRect == null || rect.y > lowestRect.y)
								lowestRect = rect;
						}
					}
				}
				synchronized (lock) {
					if (lowestRect != null) {
						hasTarget = true;
						error = lowestRect.x + lowestRect.width / 2 - 160;
					} else {
						hasTarget = false;
						error = 0;
					}
					localAlliance = alliance;
				}
			}
		}
	}
}
