package org.atorma.robot.mdp;


public class StochasticTransitionWithReward<S extends State, A extends DiscreteAction> extends TransitionWithReward<S, A> {

	private final double probability;

	public StochasticTransitionWithReward(S fromState, A byAction, S toState, 
			double reward, double probability) {
		
		super(fromState, byAction, toState, reward);
		this.probability = probability;
	}
	
	public StochasticTransitionWithReward(Transition<S, A> tr, double reward, double probability) {
		this(tr.getFromState(), tr.getAction(), tr.getToState(), reward, probability);
	}
	
	public StochasticTransitionWithReward(TransitionWithReward<S, A> tr, double probability) {
		this(tr.getFromState(), tr.getAction(), tr.getToState(), tr.getReward(), probability);
	}

	public double getProbability() {
		return probability;
	}

	
}
