package org.atorma.robot;

import org.atorma.robot.communications.NxtToPcCommunications;

import lejos.nxt.Button;

public abstract class NxtRobot implements Runnable {
	
	private NxtToPcCommunications comm = new NxtToPcCommunications();

	
	public abstract State getCurrentState();
	
	public abstract NxtAction getAction(State state);
	
	public abstract void updatePolicy(PolicyIdMap policy);
	
	@Override
	public void run() {		
		while (!Button.ENTER.isDown()) {
			State currentState = getCurrentState();
			NxtAction action = getAction(currentState);
			
			comm.pushStateAndAction(currentState, action);
			action.perform();
			
			if (comm.isPolicyAvailable()) {
				updatePolicy(comm.takePolicy());
			}
			
		}
		
		comm.disconnect();
	}
	
}
