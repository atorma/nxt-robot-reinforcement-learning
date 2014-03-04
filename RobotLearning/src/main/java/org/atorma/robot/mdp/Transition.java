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

	
	
}
