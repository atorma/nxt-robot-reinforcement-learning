package org.atorma.robot.mdp;


public class Transition<S extends State, A extends DiscreteAction> {
	
	private final S fromState;
	private final A byAction;
	private final S toState;
	
	public Transition(S fromState, A byAction, S toState) {
		if (fromState == null || byAction == null || toState == null) {
			throw new NullPointerException();
		}
		this.fromState = fromState;
		this.byAction = byAction;
		this.toState = toState;
	}

	public S getFromState() {
		return fromState;
	}

	public A getAction() {
		return byAction;
	}

	public S getToState() {
		return toState;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result
				+ ((byAction == null) ? 0 : byAction.hashCode());
		result = prime * result
				+ ((fromState == null) ? 0 : fromState.hashCode());
		result = prime * result + ((toState == null) ? 0 : toState.hashCode());
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
		Transition<?, ?> other = (Transition<?, ?>) obj;
		if (byAction == null) {
			if (other.byAction != null)
				return false;
		} else if (!byAction.equals(other.byAction))
			return false;
		if (fromState == null) {
			if (other.fromState != null)
				return false;
		} else if (!fromState.equals(other.fromState))
			return false;
		if (toState == null) {
			if (other.toState != null)
				return false;
		} else if (!toState.equals(other.toState))
			return false;
		return true;
	}

	
}
