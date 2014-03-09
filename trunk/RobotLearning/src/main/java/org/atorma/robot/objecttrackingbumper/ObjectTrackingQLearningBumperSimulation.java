package org.atorma.robot.objecttrackingbumper;

import java.text.SimpleDateFormat;
import java.util.Date;

import org.atorma.robot.simplebumper.SimbadBumper;

public class ObjectTrackingQLearningBumperSimulation {

	public static void main(String[] args) {
		SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH.mm");
		String fileName = "object tracking Q-learning bumper " + sdf.format(new Date()) + ".csv";
		ObjectTrackingQLearningBumper qLearning = new ObjectTrackingQLearningBumper(fileName);
		SimbadBumper robot = new SimbadBumper(qLearning);
		robot.startSimulationGUI();
	}
}
