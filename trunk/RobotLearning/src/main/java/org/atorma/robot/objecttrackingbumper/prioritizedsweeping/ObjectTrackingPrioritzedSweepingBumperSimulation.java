package org.atorma.robot.objecttrackingbumper.prioritizedsweeping;

import org.atorma.robot.simplebumper.SimbadBumper;

public class ObjectTrackingPrioritzedSweepingBumperSimulation {

	public static void main(String[] args) {
		ObjectTrackingPrioritizedSweepingBumper learningControl = new ObjectTrackingPrioritizedSweepingBumper("object tracking sweeping bumper.csv");
		SimbadBumper robot = new SimbadBumper(learningControl);
		robot.startSimulationGUI();
	}
}
