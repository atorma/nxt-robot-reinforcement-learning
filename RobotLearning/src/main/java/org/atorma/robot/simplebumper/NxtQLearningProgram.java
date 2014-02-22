package org.atorma.robot.simplebumper;

import lejos.pc.comm.NXTCommException;

import org.atorma.robot.communications.PcToNxtCommunication;

public class NxtQLearningProgram {

	public static void main(String[] args) {
		
		try {
			PcToNxtCommunication comms = new PcToNxtCommunication("Toveri");
			QLearningBumper bumper = new QLearningBumper(comms);
			bumper.run();
		} catch (NXTCommException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
