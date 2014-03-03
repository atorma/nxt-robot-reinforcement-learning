package org.atorma.robot.learning;

public interface DiscreteQFunction {

	double getValue(StateIdActionId stateIdActionId);
	
	void setValue(StateIdActionId stateIdActionId, double qValue);
	
	double getMaxValueForState(int stateId);

}
