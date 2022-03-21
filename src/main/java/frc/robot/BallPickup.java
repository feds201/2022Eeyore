package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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
				piston.set(Value.kReverse);
			else
				piston.set(Value.kForward);
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
		motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
		motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
		motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
		motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
		motor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
		motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255);
		motor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
		motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
		motor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);

		intakeSpeed = config.speed;
	}
}