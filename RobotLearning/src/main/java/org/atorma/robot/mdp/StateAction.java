package org.atorma.robot.mdp;

import org.atorma.robot.mdp.DiscreteAction;
import org.atorma.robot.mdp.State;

public class StateAction<S extends State, A extends DiscreteAction> {

	private final S state;
	private final A action;
	
	public StateAction(S state, A action) {
		this.state = state;
		this.action = action;
	}

	public S getState() {
		return state;
	}

	public A getAction() {
		return action;
	}
	
	
}
