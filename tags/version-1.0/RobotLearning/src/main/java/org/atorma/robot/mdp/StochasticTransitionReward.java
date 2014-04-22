package org.atorma.robot.mdp;


public class StochasticTransitionReward extends TransitionReward {

	private final double probability;

	public StochasticTransitionReward(State fromState, DiscreteAction byAction, State toState, 
			double reward, double probability) {
		
		super(fromState, byAction, toState, reward);
		this.probability = probability;
	}
	
	public StochasticTransitionReward(Transition tr, double reward, double probability) {
		this(tr.getFromState(), tr.getAction(), tr.getToState(), reward, probability);
	}
	
	public StochasticTransitionReward(TransitionReward tr, double probability) {
		this(tr.getFromState(), tr.getAction(), tr.getToState(), tr.getReward(), probability);
	}

	public double getProbability() {
		return probability;
	}

	
}
