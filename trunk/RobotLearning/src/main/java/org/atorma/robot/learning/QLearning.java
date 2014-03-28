package org.atorma.robot.learning;

import org.atorma.robot.mdp.DiscretizedStateAction;
import org.atorma.robot.mdp.DiscretizedTransitionReward;
import org.atorma.robot.policy.DiscretePolicy;

public class QLearning implements DiscretePolicy {
	
	public static final double DEFAULT_Q_VALUE = 0;

	private QTable qTable;
	
	private double learningRate;
	private double discountFactor;
	
	private double accumulatedReward = 0;

	
	public QLearning(double learningRate, double discountFactor, QTable qTable) {
		this.learningRate = learningRate;
		this.discountFactor = discountFactor;
		this.qTable = qTable;
	}

	public void update(DiscretizedTransitionReward transition) {
		accumulatedReward += transition.getReward();
		
		DiscretizedStateAction fromStateActionIds = transition.getFromStateIdActionId();
		double oldQ = qTable.getValue(fromStateActionIds);
		DiscretizedStateAction maxStateActionIds = new DiscretizedStateAction(transition.getToStateId(), getActionId(transition.getToStateId()));
		double maxQ = qTable.getValue(maxStateActionIds);
		double newQ = oldQ + learningRate*( transition.getReward() + discountFactor*maxQ - oldQ );
		
		qTable.setValue(fromStateActionIds, newQ);
	}
	
	@Override
	public Integer getActionId(int stateId) {
		return qTable.getActionId(stateId);
	}

	public double getAccumulatedReward() {
		return accumulatedReward;
	}

	
}
