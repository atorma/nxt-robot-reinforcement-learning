package itest.org.atorma.robot.communications;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import lejos.pc.comm.NXTCommException;

import org.atorma.robot.PolicyIdMap;
import org.atorma.robot.communications.PcToNxtCommunication;
import org.atorma.robot.communications.StateAndAction;

public class CommunicationsTests {

	public static void main(String[] args) throws NXTCommException {
		PcToNxtCommunication comm = new PcToNxtCommunication("Toveri");
		
		List<StateAndAction> received = new ArrayList<StateAndAction>();
		while (true) {
			comm.drainStatesAndActionsInto(received);
			for (StateAndAction sa : received) {
				System.out.println("State values: " + Arrays.toString(sa.getStateValues()));
				System.out.println("Action values: " + Arrays.toString(sa.getActionValues()));
			}
			received.clear();
			
			if (System.currentTimeMillis()%1000 == 0) {
				comm.updatePolicy(new PolicyIdMap());
			}
		}
	}
	
}
