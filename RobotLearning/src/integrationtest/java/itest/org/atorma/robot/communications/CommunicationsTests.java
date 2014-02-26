package itest.org.atorma.robot.communications;

import java.util.Random;

import lejos.pc.comm.NXTCommException;

import org.atorma.robot.DiscreteActionPolicy;
import org.atorma.robot.communications.PcToNxtCommunication;

public class CommunicationsTests {
	
	
	public static void main(String[] args) throws NXTCommException {

		DiscreteActionPolicy policy = new DiscreteActionPolicy() {
			
			private Random random;
			
			@Override
			public int getActionId(double[] state) {
				System.out.println("Received state: " + state);
				return random.nextInt();
			}
		};
		
		PcToNxtCommunication comm = new PcToNxtCommunication("Toveri", policy);

	}
	
}
