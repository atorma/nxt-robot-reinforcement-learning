package org.atorma.robot;

import org.atorma.robot.communications.NxtToPcCommunications;

import lejos.nxt.Button;

public abstract class NxtRobot implements Runnable {
	
	private NxtToPcCommunications comms = new NxtToPcCommunications();

	public abstract State getCurrentState();
	
	public abstract NxtAction getAction(int actionId);
	
	@Override
	public void run() {		
		while (!Button.ENTER.isDown()) {
			
			State currentState = getCurrentState();
			int actionId = comms.receiveActionIdForCurrentState(currentState);
			NxtAction action = getAction(actionId);
			action.perform();

		}
		
		comms.disconnect();
	}
	
}
