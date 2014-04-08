package org.atorma.robot.simplebumper;

import java.text.SimpleDateFormat;
import java.util.Date;

import org.atorma.robot.objecttrackingbumper.PrioritizedSweepingBumper;

public class SimbadBumperSimulation {

	public static void main(String[] args) {
		String experimentName = "PS 3x45";
		PrioritizedSweepingBumper learningAndControl = new PrioritizedSweepingBumper(createFileName(experimentName));
		
		SimbadBumper robot = new SimbadBumper(learningAndControl);
		robot.startSimulationGUI();
	}
	
	private static String createFileName(String experimentName) {
		SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH.mm.ss");
		String fileName = experimentName + " " + sdf.format(new Date()) + ".csv";
		return fileName;
	}
}
