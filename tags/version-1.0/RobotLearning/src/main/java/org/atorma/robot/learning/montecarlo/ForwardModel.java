package org.atorma.robot.learning.montecarlo;

import java.util.Set;

import org.atorma.robot.mdp.*;

public interface ForwardModel {

	/**
	 * Returns the allowed actions in the given state.
	 * Empty set if the state is a terminal state.
	 */
	Set<? extends DiscreteAction> getAllowedActions(State state);
	
	/** 
	 * Simulates a transition with reward when taking an action in a state.
	 * The transition can be deterministic or a sample from the distribution 
	 * of possible transitions. 
	 * Null if the starting state is a terminal state.
	 */
	TransitionReward simulateAction(StateAction fromStateAction);

}
