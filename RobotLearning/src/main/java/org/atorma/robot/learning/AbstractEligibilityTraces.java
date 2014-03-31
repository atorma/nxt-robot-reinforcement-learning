package org.atorma.robot.learning;

import java.util.*;

import org.atorma.robot.mdp.DiscretizedStateAction;

public abstract class AbstractEligibilityTraces implements EligibilityTraces {

	protected final double discountFactor;
	protected final double traceDecay;
	protected final double thresholdForRemoval;
	protected Map<DiscretizedStateAction, Double> traces = new HashMap<>();

	public AbstractEligibilityTraces(double discountFactor, double traceDecay, double thresholdForRemoval) {
		if (!(discountFactor >= 0 && discountFactor <= 1)) {
			throw new IllegalArgumentException("Discount factor must be within [0,1]");
		}
		if (!(traceDecay >= 0 && traceDecay <= 1)) {
			throw new IllegalArgumentException("Trace decay parameter must be within [0,1]");
		}
		
		this.discountFactor = discountFactor;
		this.traceDecay = traceDecay;
		this.thresholdForRemoval = thresholdForRemoval;
	}

	@Override
	public double getDiscountFactor() {
		return discountFactor;
	}

	@Override
	public double getTraceDecay() {
		return traceDecay;
	}

	@Override
	public double getThresholdForRemoval() {
		return thresholdForRemoval;
	}

	@Override
	public Set<DiscretizedStateAction> getNonZeroStateActions() {
		return traces.keySet();
	}

	@Override
	public double getTrace(DiscretizedStateAction stateIdActionId) {
		Double trace = traces.get(stateIdActionId);
		return trace != null ? trace : 0 ;
	}

	@Override
	public void clear() {
		traces.clear();
	}

}