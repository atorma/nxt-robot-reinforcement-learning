package org.atorma.robot.learning;

import java.util.Set;

import org.atorma.robot.mdp.DiscretizedStateAction;

public interface EligibilityTraces {

	double getDiscountFactor();

	double getTraceDecay();

	double getThresholdForRemoval();

	Set<DiscretizedStateAction> getNonZeroStateActions();

	double getTrace(DiscretizedStateAction stateIdActionId);

	void update(DiscretizedStateAction stateIdActionId);

	void clear();

}