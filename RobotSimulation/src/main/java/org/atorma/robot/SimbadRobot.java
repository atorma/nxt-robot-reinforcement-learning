package org.atorma.robot;

import javax.vecmath.Vector3d;

import org.atorma.robot.communications.SimbadCommunications;

import simbad.sim.Agent;

public abstract class SimbadRobot extends Agent {
	
	private final SimbadCommunications comms = new SimbadCommunications();

	public SimbadRobot(Vector3d startingPosition, String name) {
		super(startingPosition, name);
	}
	
	public abstract State getCurrentState();
	
	public abstract SimbadAction getAction(State state);
	
	public abstract void updatePolicy(PolicyIdMap policy);

	
	/** This method is called by the simulator engine on reset. */
	@Override
    public void initBehavior() {
        // nothing particular in this case
    }

    /** This method is called cyclically (20 times per second)  by the simulator engine. */
	@Override
    public void performBehavior() {

		State currentState = getCurrentState();
		SimbadAction action = getAction(currentState);
		
		comms.pushStateAndAction(currentState, action);
		action.perform();
		
		if (comms.isPolicyAvailable()) {
			updatePolicy(comms.takePolicy());
		}
    }

	public SimbadCommunications getComms() {
		return comms;
	}
	
	
}
