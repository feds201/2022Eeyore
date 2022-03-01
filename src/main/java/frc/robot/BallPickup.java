package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.config.IntakeConfig;

public class BallPickup implements Subsystem {

	private boolean deployed = false;
	private boolean active = false;
	private DoubleSolenoid piston;
	private TalonSRX motor;
	private double intakeSpeed;
	private boolean updateOutput = true;

	public BallPickup(int pcmChannel, int deploySolenoidId, int standbySolenoidId, int motorChannel, IntakeConfig config) {
		piston = new DoubleSolenoid(pcmChannel, PneumaticsModuleType.CTREPCM, deploySolenoidId, standbySolenoidId);
		motor = new TalonSRX(motorChannel);
		configure(config);
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

	@Override
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
			updateOutput = false;
		}
	}

	public void configure(IntakeConfig config) {
		TalonSRXConfiguration motorConfig = new TalonSRXConfiguration();
		motorConfig.neutralDeadband = 0.001;
		if (config.currentLimitEnabled) {
			motorConfig.peakCurrentLimit = (int)config.currentLimit;
			motorConfig.peakCurrentDuration = (int)(config.currentLimitTime * 1000);
			motorConfig.continuousCurrentLimit = (int)config.currentLimit;
		}
		motor.configAllSettings(motorConfig);
		motor.setNeutralMode(config.brake ? NeutralMode.Brake : NeutralMode.Coast);
		motor.setInverted(true);

		intakeSpeed = config.speed;
	}
}