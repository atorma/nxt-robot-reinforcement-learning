package org.atorma.robot.simplebumper;

import java.text.SimpleDateFormat;
import java.util.Date;

import lejos.pc.comm.NXTCommException;

import org.atorma.robot.DiscreteRobotController;
import org.atorma.robot.communications.PcToNxtCommunication;
import org.atorma.robot.objecttrackingbumper.ObjectTrackingQLearningBumper;
import org.atorma.robot.objecttrackingbumper.QLearningUctPlanningBumper;

public class NxtPcControl {

	public static void main(String[] args) {
		String experimentName = "NXT OTQL 3x60";
		String fileName = createFileName(experimentName);
		try {
			DiscreteRobotController controller = new ObjectTrackingQLearningBumper(fileName);
			new PcToNxtCommunication("Toveri", controller);
		} catch (NXTCommException e) {
			e.printStackTrace();
		}
	}
	
	private static String createFileName(String experimentName) {
		SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH.mm.ss");
		String fileName = experimentName + " " + sdf.format(new Date()) + ".csv";
		return fileName;
	}
}
