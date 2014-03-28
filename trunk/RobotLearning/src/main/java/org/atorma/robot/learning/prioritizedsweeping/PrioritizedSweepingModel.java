package org.atorma.robot.learning.prioritizedsweeping;

import java.util.Set;

import org.atorma.robot.mdp.*;

public interface PrioritizedSweepingModel {
	
	Set<? extends DiscreteAction> getAllowedActions(State state);

	Set<StochasticTransitionReward> getOutgoingTransitions(StateAction fromStateAction);
		
	Set<StochasticTransitionReward> getIncomingTransitions(State toState);
	
	/**
	 * Updates the model with a transition. The transition can be an observed one or
	 * a sample for setting prior probabilities.
	 */
	void updateModel(TransitionReward transition);
	
}
