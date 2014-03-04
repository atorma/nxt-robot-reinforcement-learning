package org.atorma.robot.mdp;

import org.atorma.robot.DiscreteAction;
import org.atorma.robot.State;

public class StochasticTransitionWithReward extends TransitionWithReward {

	private final double probability;

	public StochasticTransitionWithReward(State fromState, DiscreteAction byAction, State toState, 
			double reward, double probability) {
		
		super(fromState, byAction, toState, reward);
		this.probability = probability;
	}

	public double getProbability() {
		return probability;
	}

	
}
