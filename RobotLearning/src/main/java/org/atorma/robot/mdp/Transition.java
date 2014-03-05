package org.atorma.robot.mdp;


public class Transition {
	
	private final State fromState;
	private final DiscreteAction byAction;
	private final State toState;
	
	public Transition(State fromState, DiscreteAction byAction, State toState) {
		if (fromState == null || byAction == null || toState == null) {
			throw new NullPointerException();
		}
		this.fromState = fromState;
		this.byAction = byAction;
		this.toState = toState;
	}
	
	public Transition(StateAction fromStateAction, State toState) {
		this(fromStateAction.getState(), fromStateAction.getAction(), toState);
	}

	public State getFromState() {
		return fromState;
	}

	public DiscreteAction getAction() {
		return byAction;
	}

	public State getToState() {
		return toState;
	}
	
	public StateAction getFromStateAction() {
		return new StateAction(fromState, byAction);
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
		Transition other = (Transition) obj;
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
