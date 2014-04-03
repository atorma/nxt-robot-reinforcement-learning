package org.atorma.robot.learning;

import org.atorma.robot.mdp.DiscretizedStateAction;
import org.atorma.robot.mdp.DiscretizedTransitionReward;
import org.atorma.robot.policy.DiscretePolicy;

public class QLearning implements DiscretePolicy {
	
	public static final double DEFAULT_Q_VALUE = 0;

	private QTable qTable;
	private double learningRate;
	private EligibilityTraces traces;
	
	/**
	 * Standard Q-learning i.e. Q-learning without eligibility traces i.e. Q(0) . 
	 */
	public QLearning(double learningRate, double discountFactor, QTable qTable) {
		this.learningRate = learningRate;
		this.traces = new AccumulatingEligibilityTraces(discountFactor, 0, 0); 
		this.qTable = qTable;
	}
	
	/**
	 * Naive Q(lambda): Q-learning with eligibility traces that are not zeroed 
	 * in case of exploratory actions.
	 * <p>
	 * The user of this algorithm is responsible for resetting the traces for the
	 * next episode.  
	 */
	public QLearning(double learningRate, EligibilityTraces traces, QTable qTable) {
		this.learningRate = learningRate;
		this.traces = traces;
		this.qTable = qTable;
	}

	public void update(DiscretizedTransitionReward transition) {
		
		DiscretizedStateAction fromStateActionIds = transition.getFromStateIdActionId();
		traces.update(fromStateActionIds);
		
		double oldQ = qTable.getValue(fromStateActionIds);
		DiscretizedStateAction maxStateActionIds = new DiscretizedStateAction(transition.getToStateId(), getActionId(transition.getToStateId()));
		double maxQ = qTable.getValue(maxStateActionIds);
		double delta = transition.getReward() + traces.getDiscountFactor()*maxQ - oldQ;
		
		for (DiscretizedStateAction sa : traces.getNonZeroStateActions()) {
			double q = qTable.getValue(sa);
			double e = traces.getTrace(sa);
			double newQ  = q + learningRate * delta * e;
			qTable.setValue(sa, newQ);
		}
		
	}
	
	@Override
	public Integer getActionId(int stateId) {
		return qTable.getActionId(stateId);
	}

	
}
