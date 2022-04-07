package frc.robot.shooter;

import frc.robot.Subsystem;
import frc.robot.config.ShooterConfig;

public class Shooter implements Subsystem {

	private final ShooterHardware hardware;
	private final ShooterVision vision;

	private double lowGoalSpeedTop;
	private double lowGoalSpeedBottom;

	private double ejectSpeedTop;
	private double ejectSpeedBottom;

	private ShooterMode mode = ShooterMode.HIGH_GOAL_VISION;
	private boolean spin = false;
	private boolean fire = false;

	public Shooter(ShooterHardware hardware, ShooterVision vision,
					ShooterConfig config) {
		if (hardware == null)
			throw new IllegalArgumentException("hardware is null");
		if (vision == null)
			throw new IllegalArgumentException("vision is null");
		this.hardware = hardware;
		this.vision = vision;

		configureController(config);
	}

	public void setMode(ShooterMode mode) {
		if (mode == null)
			throw new IllegalArgumentException("mode is null");
		this.mode = mode;
	}

	public ShooterMode getMode() {
		return mode;
	}

	public void setSpin(boolean spin) {
		this.spin = spin;
	}

	public boolean getSpin() {
		return spin;
	}

	public boolean isSpinning() {
		return hardware.isSpinning();
	}

	public boolean isReady() {
		if (mode == ShooterMode.HIGH_GOAL_VISION)
			return hardware.isReady() && vision.isAligned();
		else
			return hardware.isReady();
	}

	public void setFire(boolean fire) {
		this.fire = fire;
	}

	public boolean isFiring() {
		return hardware.isFiring();
	}

	public void setUnjam(boolean unjam) {
		hardware.setUnjam(unjam);
	}

	public void adjustDistance(int offsetDelta) {
		vision.adjustDistance(offsetDelta);
	}

	public boolean hasTarget() {
		return vision.hasTarget();
	}

	public double[] getTarget() {
		return vision.getTarget();
	}

	public double getYawCorrection() {
		return vision.getYawCorrection();
	}

	public boolean isAligned() {
		return vision.isAligned();
	}

	@Override
	public void tick() {
		vision.setActive(spin && mode == ShooterMode.HIGH_GOAL_VISION);
		vision.tick();

		if (spin) {
			if (mode == ShooterMode.HIGH_GOAL_VISION) {
				double[] speeds = vision.getShooterSpeeds();
				hardware.setSpeed(speeds[0], speeds[1]);
			} else if (mode == ShooterMode.LOW_GOAL) {
				hardware.setSpeed(lowGoalSpeedTop, lowGoalSpeedBottom);
			} else if (mode == ShooterMode.EJECT) {
				hardware.setSpeed(ejectSpeedTop, ejectSpeedBottom);
			}
		} else {
			hardware.setSpeed(0, 0);
		}
		if (mode == ShooterMode.HIGH_GOAL_VISION && !vision.isAligned())
			hardware.setFire(false);
		else
			hardware.setFire(fire);

		hardware.tick();
	}

	public void configure(ShooterConfig config) {
		hardware.configure(config.hardwareConfig);
		vision.configure(config.visionConfig);
		configureController(config);
	}

	private void configureController(ShooterConfig config) {
		lowGoalSpeedTop = config.lowGoalSpeedTop;
		lowGoalSpeedBottom = config.lowGoalSpeedBottom;
		ejectSpeedTop = config.ejectSpeedTop;
		ejectSpeedBottom = config.ejectSpeedBottom;
	}
}
