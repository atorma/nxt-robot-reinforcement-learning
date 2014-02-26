package itest.org.atorma.robot.communications;

import java.util.Random;

import lejos.pc.comm.NXTCommException;

import org.atorma.robot.communications.PcToNxtCommunication;
import org.atorma.robot.communications.ActionIdProvider;

public class CommunicationsTests {
	
	
	public static void main(String[] args) throws NXTCommException {

		ActionIdProvider policy = new ActionIdProvider() {
			
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
