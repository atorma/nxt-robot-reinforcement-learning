package org.atorma.robot.learning.prioritizedsweeping;

import java.util.Set;

import org.atorma.robot.mdp.*;

public interface PrioritizedSweepingModel {
	
	Set<? extends DiscreteAction> getAllActions();

	Set<StochasticTransitionReward> getOutgoingTransitions(StateAction stateAction);
	
	Set<StochasticTransitionReward> getIncomingTransitions(State state);
	
	void updateModel(TransitionReward observation);
	
}
