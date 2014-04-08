package org.atorma.robot.simplebumper;

import java.text.SimpleDateFormat;
import java.util.Date;

import org.atorma.robot.objecttrackingbumper.ObjectTrackingQLearningBumper;

public class SimbadBumperSimulation {

	public static void main(String[] args) {
		String experimentName = "OTQL 3x45";
		ObjectTrackingQLearningBumper learningAndControl = new ObjectTrackingQLearningBumper(createFileName(experimentName));
		
		SimbadBumper robot = new SimbadBumper(learningAndControl);
		robot.startSimulationGUI();
	}
	
	private static String createFileName(String experimentName) {
		SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH.mm.ss");
		String fileName = experimentName + " " + sdf.format(new Date()) + ".csv";
		return fileName;
	}
}
