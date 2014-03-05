package org.atorma.robot.mdp;


public class TransitionReward extends Transition {
	
	private final double reward;

	public TransitionReward(State fromState, DiscreteAction byAction, State toState, double reward) {
		super(fromState, byAction, toState);
		this.reward = reward;
	}

	public double getReward() {
		return reward;
	}

	
}
