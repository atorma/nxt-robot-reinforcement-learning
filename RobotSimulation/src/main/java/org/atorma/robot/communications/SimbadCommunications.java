package org.atorma.robot.communications;

import java.util.Collection;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

import org.atorma.robot.PolicyIdMap;
import org.atorma.robot.SimbadAction;
import org.atorma.robot.State;

/**
 * Simulates connections between PC and the robot.
 */
public class SimbadCommunications implements RobotCommunications {

	private PolicyIdMap policy;
	private BlockingQueue<StateAndAction> statesAndActions = new LinkedBlockingQueue<>();

	@Override
	public synchronized void updatePolicy(PolicyIdMap policy) {
		this.policy = policy; 
	}

	@Override
	public void drainStatesAndActionsInto(Collection<StateAndAction> target) {
		statesAndActions.drainTo(target);
	}

	@Override
	public StateAndAction takeStateAndAction() {
		StateAndAction values = null;
		while (values == null) {
			try {
				values = statesAndActions.take();
			} catch (InterruptedException e) {}
		}
		//System.out.println("State taken, now " + statesAndActions.size());
		return values;
	}

	@Override
	public void disconnect() {		
	}

	
	public void pushStateAndAction(State currentState, SimbadAction action) {
		statesAndActions.add(new StateAndAction(currentState.getValues(), action.getValues()));
		//System.out.println("State queued, now " + statesAndActions.size());
	}

	public synchronized boolean isPolicyAvailable() {
		return policy != null;
	}

	public synchronized PolicyIdMap takePolicy() {
		PolicyIdMap temp = policy;
		policy = null;
		return temp;
	}

}
