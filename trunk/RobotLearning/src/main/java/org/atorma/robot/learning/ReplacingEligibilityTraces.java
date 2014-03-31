package org.atorma.robot.learning;

import java.util.Iterator;
import java.util.Map.Entry;

import org.atorma.robot.mdp.DiscretizedStateAction;

public class ReplacingEligibilityTraces extends AbstractEligibilityTraces {

	public ReplacingEligibilityTraces(double discountFactor, double traceDecay, double thresholdForRemoval) {
		super(discountFactor, traceDecay, thresholdForRemoval);
	}

	@Override
	public void update(DiscretizedStateAction stateIdActionId) {
		double newValue = 1;
		traces.put(stateIdActionId, newValue);
		
		for (Iterator<Entry<DiscretizedStateAction, Double>> iter = traces.entrySet().iterator(); iter.hasNext(); ) {
			
			Entry<DiscretizedStateAction, Double> entry = iter.next();
			if (entry.getKey().equals(stateIdActionId)) {
				continue;
			}
			
			newValue = traceDecay * discountFactor * entry.getValue();
			if (newValue < thresholdForRemoval || newValue <= 0.0) {
				iter.remove();
			} else {
				entry.setValue(newValue);
			}
		}
	}

}
