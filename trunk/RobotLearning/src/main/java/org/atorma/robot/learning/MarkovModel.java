package org.atorma.robot.learning;

import java.util.Set;

import org.atorma.robot.mdp.*;

public interface MarkovModel<S extends State, A extends DiscreteAction> {

	Set<StochasticTransitionWithReward<S, A>> getTransitions(StateAction<S, A> stateAction);
	
	Set<StateAction<S, A>> getPredecessors(State state);
	
	void updateModel(TransitionWithReward<S, A> observation);
	
	
	
}
