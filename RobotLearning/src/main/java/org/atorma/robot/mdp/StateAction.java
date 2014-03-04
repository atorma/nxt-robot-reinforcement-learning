package org.atorma.robot.mdp;

import org.atorma.robot.mdp.DiscreteAction;
import org.atorma.robot.mdp.State;

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
