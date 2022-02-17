package frc.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

public interface Section {

	public List<TalonFX> getInstruments();
}
