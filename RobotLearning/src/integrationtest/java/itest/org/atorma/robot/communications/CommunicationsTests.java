package itest.org.atorma.robot.communications;

import java.util.Arrays;
import java.util.Random;

import lejos.pc.comm.NXTCommException;

import org.atorma.robot.DiscreteActionController;
import org.atorma.robot.communications.PcToNxtCommunication;

public class CommunicationsTests {
	
	
	public static void main(String[] args) throws NXTCommException {

		DiscreteActionController policy = new DiscreteActionController() {
			
			private Random random = new Random();
			
			@Override
			public int getActionId(double[] state) {
				System.out.println("Received state: " + Arrays.toString(state));
				return random.nextInt();
			}
		};
		
		PcToNxtCommunication comm = new PcToNxtCommunication("Toveri", policy);

	}
	
}
