package org.atorma.robot.mdp;


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
