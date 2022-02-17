package frc.robot;

import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

public class MusicPlayer implements Subsystem {

	private final Orchestra orchestra;
	private final Queue<String> queue;
	private boolean play = false;

	public MusicPlayer(List<TalonFX> instruments) {
		orchestra = new Orchestra(instruments);
		queue = new LinkedList<>();
	}

	public MusicPlayer() {
		orchestra = new Orchestra();
		queue = new LinkedList<>();
	}

	public void addInstrument(TalonFX instrument) {
		orchestra.addInstrument(instrument);
	}

	public void addInstruments(List<TalonFX> instruments) {
		for (TalonFX instrument : instruments)
			orchestra.addInstrument(instrument);
	}

	public void addSection(Section section) {
		addInstruments(section.getInstruments());
	}

	public void setPlay(boolean play) {
		this.play = play;
	}

	public boolean getPlay() {
		return play;
	}

	public void queue(String track) {
		queue.add(track);
	}

	@Override
	public void tick() {
		if (!play && orchestra.isPlaying())
			orchestra.stop();
		else if (play && !orchestra.isPlaying()) {
			String track = queue.poll();
			if (track != null) {
				orchestra.loadMusic(track);
				orchestra.play();
				queue.add(track);
			}
		}
	}
}
