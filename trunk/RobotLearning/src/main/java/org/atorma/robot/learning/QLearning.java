package org.atorma.robot.learning;

import org.atorma.robot.mdp.DiscretizedStateAction;
import org.atorma.robot.mdp.DiscretizedTransitionReward;
import org.atorma.robot.policy.DiscretePolicy;
import org.atorma.robot.policy.StateIdToActionIdMap;

public class QLearning implements DiscretePolicy, QTable {
	
	public static final double DEFAULT_Q_VALUE = 0;

	private HashMapQTable qTable = new HashMapQTable(DEFAULT_Q_VALUE);
	private StateIdToActionIdMap stateIdToBestActionIdMap = new StateIdToActionIdMap();
	
	private double learningRate;
	private double discountFactor;
	
	private double accumulatedReward = 0;

	
	public QLearning(double learningRate, double discountFactor) {
		this.learningRate = learningRate;
		this.discountFactor = discountFactor;
	}

	public void update(DiscretizedTransitionReward transition) {
		accumulatedReward += transition.getReward();
		
		qTable.addStateId(transition.getFromStateId());
		qTable.addStateId(transition.getToStateId());
		qTable.addActionId(transition.getByActionId());
		
		DiscretizedStateAction fromStateActionIds = transition.getFromStateIdActionId();
		double oldQ = qTable.getValue(fromStateActionIds);
		DiscretizedStateAction maxStateActionIds = new DiscretizedStateAction(transition.getToStateId(), getActionId(transition.getToStateId()));
		double maxQ = qTable.getValue(maxStateActionIds);
		double newQ = oldQ + learningRate*( transition.getReward() + discountFactor*maxQ - oldQ );
		
		qTable.put(fromStateActionIds, newQ);
		updateBestAction(transition.getFromStateId());
	}
	
	private void updateBestAction(int stateId) {
		DiscretizedStateAction best = qTable.getBestActionInState(stateId);
		stateIdToBestActionIdMap.put(stateId, best.getActionId());
	}
	
	/**
	 * Returns the learned best action id for the given state, or some action
	 * if all are equally good, or <tt>null</tt> if no actions known yet.
	 */
	@Override
	public Integer getActionId(int stateId) {
		Integer bestActionId = stateIdToBestActionIdMap.getActionId(stateId);
		if (bestActionId != null) {
			return bestActionId;
		} else if (!qTable.getActionIds().isEmpty()) {
			return qTable.getBestActionInState(stateId).getActionId();
		} else {
			return null;
		}
	}

	public double getAccumulatedReward() {
		return accumulatedReward;
	}

	@Override
	public DiscretizedStateAction getBestActionInState(int stateId) {
		if (stateIdToBestActionIdMap.containsKey(stateId)) {
			return new DiscretizedStateAction(stateId, stateIdToBestActionIdMap.getActionId(stateId));
		} else {
			return qTable.getBestActionInState(stateId);
		}
	}

	@Override
	public double getValue(DiscretizedStateAction stateIdActionId) {
		return qTable.getValue(stateIdActionId);
	}

	@Override
	public void setValue(DiscretizedStateAction stateIdActionId, double qValue) {
		qTable.setValue(stateIdActionId, qValue);
	}

	@Override
	public double getMaxValueInState(int stateId) {
		return qTable.getValue(getBestActionInState(stateId));
	}

	
}
