package org.atorma.robot.learning;

import org.atorma.robot.mdp.StateIdActionId;

public interface DiscreteQFunction {

	double getValue(StateIdActionId stateIdActionId);
	
	void setValue(StateIdActionId stateIdActionId, double qValue);
	
	double getMaxValueForState(int stateId);

}
