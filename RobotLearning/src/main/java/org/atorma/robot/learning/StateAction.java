package org.atorma.robot.learning;

import org.atorma.robot.DiscreteAction;
import org.atorma.robot.State;

public class StateAction {

	private final State state;
	private final DiscreteAction action;
	
	public StateAction(State state, DiscreteAction action) {
		this.state = state;
		this.action = action;
	}

	public State getState() {
		return state;
	}

	public DiscreteAction getAction() {
		return action;
	}
	
	
}
