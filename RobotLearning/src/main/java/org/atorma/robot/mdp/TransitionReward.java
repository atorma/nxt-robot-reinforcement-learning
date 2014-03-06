package org.atorma.robot.mdp;


public class TransitionReward extends Transition {
	
	private final double reward;

	public TransitionReward(State fromState, DiscreteAction byAction, State toState, double reward) {
		super(fromState, byAction, toState);
		this.reward = reward;
	}
	
	public TransitionReward(StateAction startStateAction, State endState, double reward) {
		super(startStateAction, endState);
		this.reward = reward;
	}
	
	public TransitionReward(Transition transition, double reward) {
		this(transition.getFromState(), transition.getAction(), transition.getToState(), reward);
	}

	public double getReward() {
		return reward;
	}

	
}
