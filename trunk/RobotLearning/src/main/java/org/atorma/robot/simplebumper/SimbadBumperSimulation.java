package org.atorma.robot.simplebumper;

import java.text.SimpleDateFormat;
import java.util.Date;

public class SimbadBumperSimulation {

	public static void main(String[] args) {
		String experimentName = "OT Q-learning 3x45 deg sectors";
		QLearningBumper qLearning = new QLearningBumper(createFileName(experimentName));
		
		SimbadBumper robot = new SimbadBumper(qLearning);
		robot.startSimulationGUI();
	}
	
	private static String createFileName(String experimentName) {
		SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH.mm.ss");
		String fileName = experimentName + " " + sdf.format(new Date()) + ".csv";
		return fileName;
	}
}
