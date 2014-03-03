package org.atorma.robot.learning;

import java.util.Set;

import org.atorma.robot.DiscreteAction;
import org.atorma.robot.State;

public interface DiscreteModel {

	TransitionWithReward simulateTransition(State state, DiscreteAction action);
	
	void updateModel(TransitionWithReward transition);
	
	Set<StateAction> getPredecessors(State state);
	
}
