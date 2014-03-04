package org.atorma.robot.learning;

import java.util.Set;

import org.atorma.robot.mdp.*;

public interface DiscreteStochasticModel<S extends State, A extends DiscreteAction> {

	StochasticTransitionWithReward<S, A> simulateTransition(S state, A action);
	
	void updateModel(TransitionWithReward<S, A> observation);
	
	Set<StateAction<S, A>> getPredecessors(State state);
	
}
