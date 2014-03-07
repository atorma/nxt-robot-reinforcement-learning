package itest.org.atorma.robot.communications;

import java.util.Arrays;
import java.util.Random;

import lejos.pc.comm.NXTCommException;

import org.atorma.robot.DiscreteRobotController;
import org.atorma.robot.communications.PcToNxtCommunication;

public class CommunicationsTests {
	
	
	public static void main(String[] args) throws NXTCommException {

		DiscreteRobotController policy = new DiscreteRobotController() {
			
			private Random random = new Random();
			
			@Override
			public int getActionId(double[] state) {
				System.out.println("Received state: " + Arrays.toString(state));
				return random.nextInt();
			}
		};
		
		new PcToNxtCommunication("Toveri", policy);

	}
	
}
