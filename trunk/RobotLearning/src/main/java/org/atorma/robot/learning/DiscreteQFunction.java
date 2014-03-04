package org.atorma.robot.learning;

import org.atorma.robot.mdp.DiscretizedStateAction;

public interface DiscreteQFunction {

	double getValue(DiscretizedStateAction stateIdActionId);
	
	void setValue(DiscretizedStateAction stateIdActionId, double qValue);
	
	double getMaxValueForState(int stateId);

}
