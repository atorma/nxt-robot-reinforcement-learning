package org.atorma.robot.mdp;


public class TransitionWithReward<S extends State, A extends DiscreteAction> extends Transition<S, A> {
	
	private final double reward;

	public TransitionWithReward(S fromState, A byAction, S toState, double reward) {
		super(fromState, byAction, toState);
		this.reward = reward;
	}

	public double getReward() {
		return reward;
	}

	
}
