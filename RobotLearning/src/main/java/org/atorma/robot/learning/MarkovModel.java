package org.atorma.robot.learning;

import java.util.Set;

import org.atorma.robot.mdp.*;

public interface MarkovModel {

	Set<StochasticTransitionReward> getOutgoingTransitions(StateAction stateAction);
	
	Set<StochasticTransitionReward> getIncomingTransitions(State state);
	
	// TODO remove
	Set<StateAction> getPredecessors(State state);
	
	// TODO remove?
	double getTransitionProbability(Transition transition);
	
	void updateModel(TransitionReward observation);

	
	
	
	
}
