package org.atorma.robot.objecttrackingbumper.qlearning;

import java.text.SimpleDateFormat;
import java.util.Date;

import org.atorma.robot.simplebumper.SimbadBumper;

public class ObjectTrackingQLearningBumperSimulation {

	public static void main(String[] args) {
		SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH.mm");
		String fileName = "object tracking q-learning 3 sectors lambda 0.1 " + sdf.format(new Date()) + ".csv";
		ObjectTrackingQLearningBumper qLearning = new ObjectTrackingQLearningBumper(fileName);
		SimbadBumper robot = new SimbadBumper(qLearning);
		robot.startSimulationGUI();
	}
}
