package org.atorma.robot.learning;

import java.util.Set;

import org.atorma.robot.mdp.*;

public interface DiscreteActionModel {
	
	Set<? extends DiscreteAction> getAllActions();

	Set<StochasticTransitionReward> getOutgoingTransitions(StateAction fromStateAction);
	
	Set<StochasticTransitionReward> getIncomingTransitions(State toState);
	
	/**
	 * Updates the model with a transition. The transition can be an observed one or
	 * a sample for setting prior probabilities.
	 */
	void updateModel(TransitionReward transition);
	
}
