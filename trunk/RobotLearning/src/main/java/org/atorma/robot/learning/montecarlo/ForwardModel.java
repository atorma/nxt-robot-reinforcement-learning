package org.atorma.robot.learning.montecarlo;

import java.util.Set;

import org.atorma.robot.mdp.*;

public interface ForwardModel {

	Set<? extends DiscreteAction> getAllowedActions(State state);
	
	/** 
	 * Simulates a transition with reward when taking an action in a state.
	 * The transition can be deterministic or a sample from the distribution 
	 * of possible transitions. 
	 */
	TransitionReward simulateAction(StateAction fromStateAction);
	
	void updateModel(TransitionReward transition);
}
