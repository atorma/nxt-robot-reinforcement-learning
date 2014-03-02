package org.atorma.robot.objecttrackingbumper;

import org.atorma.robot.simplebumper.SimbadBumper;

public class ObjectTrackingBumperSimulation {

	public static void main(String[] args) {
		ObjectTrackingQLearningBumper qLearning = new ObjectTrackingQLearningBumper("object tracking Q-learning bumper.csv");
		SimbadBumper robot = new SimbadBumper(qLearning);
		robot.startSimulationGUI();
	}
}
