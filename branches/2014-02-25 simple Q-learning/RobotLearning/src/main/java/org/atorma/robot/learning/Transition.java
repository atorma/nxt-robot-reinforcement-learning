package org.atorma.robot.learning;

import org.atorma.robot.Action;
import org.atorma.robot.State;

public class Transition {
	
	private final State fromState;
	private final Action byAction;
	private final State toState;
	
	public Transition(State fromState, Action byAction, State toState) {
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

	public Action getAction() {
		return byAction;
	}

	public State getToState() {
		return toState;
	}

	
	
}
