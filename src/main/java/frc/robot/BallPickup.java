package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class BallPickup {

	private boolean deployed = false;
	private boolean active = false;
	private DoubleSolenoid piston;
	private TalonSRX motor;
	private final double intakeSpeed;
	private boolean updateOutput = true;

	public BallPickup(int pcmChannel, int deploySolenoidId, int standbySolenoidId, int motorChannel, double intakeSpeed) {
		if (intakeSpeed < 0 || intakeSpeed > 1)
			throw new IllegalArgumentException("intake speed out of bounds");
		piston = new DoubleSolenoid(pcmChannel, PneumaticsModuleType.CTREPCM, deploySolenoidId, standbySolenoidId);
		this.intakeSpeed = intakeSpeed;
		motor = new TalonSRX(motorChannel);
	}

	public void setDeployed(boolean deployed) {
		if (deployed != this.deployed) {
			this.deployed = deployed;
			updateOutput = true;
		}
	}

	public void setActive(boolean active) {
		if (active != this.active) {
			this.active = active;
			updateOutput = true;
		}
	}

	public void tick() {
		if (updateOutput) {
			if (active)
				motor.set(ControlMode.PercentOutput, intakeSpeed);
			else
				motor.set(ControlMode.PercentOutput, 0);
			if (deployed)
				piston.set(Value.kForward);
			else
				piston.set(Value.kReverse);
		}
	}
}