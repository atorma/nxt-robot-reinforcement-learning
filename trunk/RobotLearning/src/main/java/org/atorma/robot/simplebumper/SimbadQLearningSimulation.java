package org.atorma.robot.simplebumper;

public class SimbadQLearningSimulation {

	public static void main(String[] args) {
		QLearningBumper qLearning = new QLearningBumper("simple Q-learning bumber.csv");
		SimbadBumper robot = new SimbadBumper(qLearning);
		robot.startSimulationGUI();
	}
}
