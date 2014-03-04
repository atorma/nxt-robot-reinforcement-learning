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

	public State getFromState() {
		return fromState;
	}

	public DiscreteAction getAction() {
		return byAction;
	}

	public State getToState() {
		return toState;
	}

	
	
}
