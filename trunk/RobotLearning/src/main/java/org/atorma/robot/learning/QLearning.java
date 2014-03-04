package org.atorma.robot.learning;

import java.util.*;

import org.atorma.robot.mdp.DiscretizedTransitionWithReward;
import org.atorma.robot.mdp.DiscretizedStateAction;
import org.atorma.robot.policy.DiscretePolicy;
import org.atorma.robot.policy.StateIdToActionIdMap;

public class QLearning implements DiscretePolicy {

	private Map<DiscretizedStateAction, Double> qTable = new HashMap<>();
	private StateIdToActionIdMap stateIdToBestActionIdMap = new StateIdToActionIdMap();
	
	private Set<Integer> stateIds = new HashSet<>();
	private Set<Integer> actionIds = new HashSet<>();
	
	private double learningRate;
	private double discountFactor;
	
	private double accumulatedReward = 0;
	
	private double defaultStateActionValue = 0;
	
	public QLearning(double learningRate, double discountFactor) {
		this.learningRate = learningRate;
		this.discountFactor = discountFactor;
	}

	public void update(DiscretizedTransitionWithReward transition) {
		accumulatedReward += transition.getReward();
		
		stateIds.add(transition.getFromStateId());
		stateIds.add(transition.getToStateId());
		actionIds.add(transition.getByActionId());
		
		DiscretizedStateAction fromStateActionIds = transition.getFromStateIdActionId();
		double oldQ = getQValue(fromStateActionIds);
		DiscretizedStateAction maxStateActionIds = new DiscretizedStateAction(transition.getToStateId(), getActionId(transition.getToStateId()));
		double maxQ = getQValue(maxStateActionIds);
		double newQ = oldQ + learningRate*( transition.getReward() + discountFactor*maxQ - oldQ );
		
		qTable.put(fromStateActionIds, newQ);
		updateBestAction(transition.getFromStateId());
	}
	

	private double getQValue(DiscretizedStateAction stateActionIds) {
		if (!qTable.containsKey(stateActionIds)) {
			qTable.put(stateActionIds, defaultStateActionValue);
		}
		return qTable.get(stateActionIds);
	}
	
	private void updateBestAction(int stateId) {
		double bestActionValue = Double.NEGATIVE_INFINITY;
		Integer bestActionId = null;
		
		for (int actionId : actionIds) {
			double q = getQValue(new DiscretizedStateAction(stateId, actionId));
			if (q > bestActionValue) {
				bestActionValue = q;
				bestActionId = actionId;
			}
		}

		stateIdToBestActionIdMap.put(stateId, bestActionId);
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
		} else if (!actionIds.isEmpty()) {
			return actionIds.iterator().next();
		} else {
			return null;
		}
	}
	
	public StateIdToActionIdMap getLearnedPolicyMap() {
		return stateIdToBestActionIdMap;
	}

	public double getAccumulatedReward() {
		return accumulatedReward;
	}

	
}
