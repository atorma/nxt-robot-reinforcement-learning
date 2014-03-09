package org.atorma.robot.simplebumper;

import java.text.SimpleDateFormat;
import java.util.Date;

public class QLearningBumperSimulation {

	public static void main(String[] args) {
		SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH.mm");
		String fileName = "simple Q-learning " + sdf.format(new Date()) + ".csv";
		QLearningBumper qLearning = new QLearningBumper(fileName);
		SimbadBumper robot = new SimbadBumper(qLearning);
		robot.startSimulationGUI();
	}
}
