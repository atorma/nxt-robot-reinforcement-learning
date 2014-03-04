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

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((action == null) ? 0 : action.hashCode());
		result = prime * result + ((state == null) ? 0 : state.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		StateAction<?, ?> other = (StateAction<?, ?>) obj;
		if (action == null) {
			if (other.action != null)
				return false;
		} else if (!action.equals(other.action))
			return false;
		if (state == null) {
			if (other.state != null)
				return false;
		} else if (!state.equals(other.state))
			return false;
		return true;
	}
	
	
	
}
