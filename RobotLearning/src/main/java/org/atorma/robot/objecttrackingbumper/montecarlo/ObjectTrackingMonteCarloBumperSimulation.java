package org.atorma.robot.objecttrackingbumper.montecarlo;

import java.text.SimpleDateFormat;
import java.util.Date;

import org.atorma.robot.simplebumper.SimbadBumper;

public class ObjectTrackingMonteCarloBumperSimulation {

	public static void main(String[] args) {
		SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH.mm");
		String fileName = "object tracking uct bumper 1 sectors " + sdf.format(new Date()) + ".csv";
		ObjectTrackingMonteCarloBumper learningControl = new ObjectTrackingMonteCarloBumper(fileName);
		SimbadBumper robot = new SimbadBumper(learningControl);
		robot.startSimulationGUI();
	}
}
