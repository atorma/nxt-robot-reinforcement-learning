package org.atorma.robot.learning;

import org.atorma.robot.mdp.DiscretizedStateAction;
import org.atorma.robot.policy.DiscretePolicy;

public interface QTable extends DiscretePolicy {

	double getValue(DiscretizedStateAction stateIdActionId);
	
	void setValue(DiscretizedStateAction stateIdActionId, double qValue);
	
	double getMaxValueForState(int stateId);

}
