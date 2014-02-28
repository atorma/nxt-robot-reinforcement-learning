package org.atorma.robot.objecttrackingbumper;

import org.atorma.robot.simplebumper.SimbadBumper;

public class SimbadQLearningSimulation {

	public static void main(String[] args) {
		ObjectTrackingQLearningBumper qLearning = new ObjectTrackingQLearningBumper();
		SimbadBumper robot = new SimbadBumper(qLearning);
		robot.startSimulationGUI();
	}
}
