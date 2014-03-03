package org.atorma.robot.learning;

import org.atorma.robot.DiscreteAction;
import org.atorma.robot.State;

public class TransitionWithReward extends Transition {
	
	private final double reward;

	public TransitionWithReward(State fromState, DiscreteAction byAction, State toState, double reward) {
		super(fromState, byAction, toState);
		this.reward = reward;
	}

	public double getReward() {
		return reward;
	}

	
}
