package org.atorma.robot.objecttrackingbumper.prioritizedsweeping;

import java.text.SimpleDateFormat;
import java.util.Date;

import org.atorma.robot.simplebumper.SimbadBumper;

public class ObjectTrackingPrioritzedSweepingBumperSimulation {

	public static void main(String[] args) {
		SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH.mm");
		String fileName = "PS prior prob 3 sectors " + sdf.format(new Date()) + ".csv";
		ObjectTrackingPrioritizedSweepingBumper learningControl = new ObjectTrackingPrioritizedSweepingBumper(fileName);
		SimbadBumper robot = new SimbadBumper(learningControl);
		robot.startSimulationGUI();
	}
}
