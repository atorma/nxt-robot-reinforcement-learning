package org.atorma.robot.objecttrackingbumper;

import org.atorma.robot.simplebumper.SimbadBumper;

public class SimbadQLearningSimulation {

	public static void main(String[] args) {
		QLearningBumper qLearning = new QLearningBumper();
		SimbadBumper robot = new SimbadBumper(qLearning);
		robot.startSimulationGUI();
	}
}
