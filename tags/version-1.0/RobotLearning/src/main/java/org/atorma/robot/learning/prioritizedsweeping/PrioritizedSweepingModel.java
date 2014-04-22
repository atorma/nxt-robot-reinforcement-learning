package org.atorma.robot.learning.prioritizedsweeping;

import java.util.Set;

import org.atorma.robot.mdp.*;

public interface PrioritizedSweepingModel {
	
	Set<? extends DiscreteAction> getAllowedActions(State state);

	Set<StochasticTransitionReward> getOutgoingTransitions(StateAction fromStateAction);
		
	Set<StochasticTransitionReward> getIncomingTransitions(State toState);
	
}
