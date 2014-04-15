package org.atorma.robot.simplebumper;

import java.text.SimpleDateFormat;
import java.util.Date;

import org.atorma.robot.DiscreteRobotController;
import org.atorma.robot.objecttrackingbumper.*;

public class SimbadBumperSimulation {

	public static void main(String[] args) {
		String experimentName = "UCT 3x45";
		DiscreteRobotController learningAndControl = new QLearningUctPlanningBumper(createFileName(experimentName));
		
		SimbadBumper robot = new SimbadBumper(learningAndControl);
		robot.startSimulationGUI();
	}
	
	private static String createFileName(String experimentName) {
		SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH.mm.ss");
		String fileName = experimentName + " " + sdf.format(new Date()) + ".csv";
		return fileName;
	}
}
