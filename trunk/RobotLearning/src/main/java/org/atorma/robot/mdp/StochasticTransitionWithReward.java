package org.atorma.robot.mdp;


public class StochasticTransitionWithReward extends TransitionWithReward {

	private final double probability;

	public StochasticTransitionWithReward(State fromState, DiscreteAction byAction, State toState, 
			double reward, double probability) {
		
		super(fromState, byAction, toState, reward);
		this.probability = probability;
	}
	
	public StochasticTransitionWithReward(Transition tr, double reward, double probability) {
		this(tr.getFromState(), tr.getAction(), tr.getToState(), reward, probability);
	}
	
	public StochasticTransitionWithReward(TransitionWithReward tr, double probability) {
		this(tr.getFromState(), tr.getAction(), tr.getToState(), tr.getReward(), probability);
	}

	public double getProbability() {
		return probability;
	}

	
}
