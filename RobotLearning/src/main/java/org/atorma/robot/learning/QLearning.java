package org.atorma.robot.learning;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.atorma.robot.*;
import org.atorma.robot.discretization.VectorDiscretizer;

public class QLearning implements DiscretePolicy {

	private Map<StateIdActionId, Double> qTable = new HashMap<>();
	private StateIdToActionIdMap stateIdToBestActionIdMap = new StateIdToActionIdMap();
	
	private Set<Integer> stateIds = new HashSet<>();
	private Set<Integer> actionIds = new HashSet<>();
	private VectorDiscretizer stateDiscretizer;
	
	private RewardFunction rewardFunction;
	private double learningRate;
	private double discountFactor;
	
	private double accumulatedReward = 0;
	
	private double defaultStateActionValue = 0;
	
	public QLearning(VectorDiscretizer stateDiscretizer, RewardFunction rewardFunction, double learningRate, double discountFactor) {
		this.stateDiscretizer = stateDiscretizer;
		this.rewardFunction = rewardFunction;
		this.learningRate = learningRate;
		this.discountFactor = discountFactor;
	}


	public void update(Transition transition) {
		int fromStateId = stateDiscretizer.getId(transition.getFromState().getValues());
		int byActionId = transition.getAction().getId();
		int toStateId = stateDiscretizer.getId(transition.getToState().getValues());
		double reward = rewardFunction.getReward(transition);
		accumulatedReward += reward;
		
		stateIds.add(fromStateId);
		stateIds.add(toStateId);
		actionIds.add(byActionId);
		
		StateIdActionId fromStateActionIds = new StateIdActionId(fromStateId, byActionId);
		double oldQ = getQValue(fromStateActionIds);
		StateIdActionId maxStateActionIds = new StateIdActionId(toStateId, getActionId(toStateId));
		double maxQ = getQValue(maxStateActionIds);
		double newQ = oldQ + learningRate*( reward + discountFactor*maxQ - oldQ );
		
		qTable.put(fromStateActionIds, newQ);
		updatePolicy(fromStateId);
	}
	

	private double getQValue(StateIdActionId stateActionIds) {
		if (!qTable.containsKey(stateActionIds)) {
			qTable.put(stateActionIds, defaultStateActionValue);
		}
		return qTable.get(stateActionIds);
	}
	
	private void updatePolicy(int stateId) {
		ActionValue maxActionValue = getMaxActionValue(stateId);
		stateIdToBestActionIdMap.put(stateId, maxActionValue.actionId);
	}
	
	private ActionValue getMaxActionValue(int stateId) {
		ActionValue best = new ActionValue();
		best.value = Double.NEGATIVE_INFINITY;

		for (int actionId : actionIds) {
			StateIdActionId sai = new StateIdActionId(stateId, actionId);
			double q = getQValue(sai);
			if (q > best.value) {
				best.value = q;
				best.actionId = actionId;
			}
		}
		
		return best;
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


	private static class ActionValue {
		Integer actionId = null;
		Double value = null;
	}


	
}
