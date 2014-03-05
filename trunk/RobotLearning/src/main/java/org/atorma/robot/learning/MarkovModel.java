package org.atorma.robot.learning;

import java.util.Set;

import org.atorma.robot.mdp.*;

public interface MarkovModel {

	Set<StochasticTransitionReward> getTransitions(StateAction stateAction);
	
	Set<StateAction> getPredecessors(State state);
	
	double getTransitionProbability(Transition transition);
	
	void updateModel(TransitionReward observation);

	
	
	
	
}
