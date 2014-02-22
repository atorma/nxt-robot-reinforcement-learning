package org.atorma.robot.learning;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.atorma.robot.PolicyIdMap;
import org.atorma.robot.RewardFunction;
import org.atorma.robot.discretization.IdFunction;

public class QLearning {

	private Map<StateActionIds, Double> qTable = new HashMap<>();
	private PolicyIdMap learnedPolicy = new PolicyIdMap();
	
	private Set<Integer> stateIds = new HashSet<>();
	private Set<Integer> actionIds = new HashSet<>();
	
	private IdFunction stateIdMap;
	private IdFunction actionIdMap;
	private RewardFunction rewardFunction;
	private double learningRate;
	private double discountFactor;
	
	private double accumulatedReward = 0;
	
	private double defaultStateActionValue = 0;
	
	public QLearning(IdFunction stateIdMap, IdFunction actionIdMap, RewardFunction rewardFunction, double learningRate, double discountFactor) {
		this.stateIdMap = stateIdMap;
		this.actionIdMap = actionIdMap;
		this.rewardFunction = rewardFunction;
		this.learningRate = learningRate;
		this.discountFactor = discountFactor;
	}


	public void update(Transition transition) {
		int fromStateId = stateIdMap.getId(transition.getFromState().getValues());
		int byActionId = actionIdMap.getId(transition.getAction().getValues());
		int toStateId = stateIdMap.getId(transition.getToState().getValues());
		double reward = rewardFunction.getReward(transition);
		accumulatedReward += reward;
		
		stateIds.add(fromStateId);
		stateIds.add(toStateId);
		actionIds.add(byActionId);
		
		StateActionIds fromStateActionIds = new StateActionIds(fromStateId, byActionId);
		double oldQ = getQValue(fromStateActionIds);
		ActionValue maxActionValue = getMaxActionValue(toStateId);
		double newQ = oldQ + learningRate*( reward + discountFactor*maxActionValue.value - oldQ );
		
		qTable.put(fromStateActionIds, newQ);
		updatePolicy(fromStateId);
	}
	

	private double getQValue(StateActionIds stateActionIds) {
		if (!qTable.containsKey(stateActionIds)) {
			qTable.put(stateActionIds, defaultStateActionValue);
		}
		return qTable.get(stateActionIds);
	}
	
	private ActionValue getMaxActionValue(int stateId) {
		ActionValue best = new ActionValue();
		best.value = Double.NEGATIVE_INFINITY;

		for (int actionId : actionIds) {
			StateActionIds sai = new StateActionIds(stateId, actionId);
			double q = getQValue(sai);
			if (q > best.value) {
				best.value = q;
				best.actionId = actionId;
			}
		}
		
		return best;
	}
	
	private void updatePolicy(int stateId) {
		ActionValue maxActionValue = getMaxActionValue(stateId);
		learnedPolicy.put(stateId, maxActionValue.actionId);
	}
	
	public PolicyIdMap getLearnedPolicy() {
		return learnedPolicy;
	}

	public double getAccumulatedReward() {
		return accumulatedReward;
	}




	private static class ActionValue {
		Integer actionId = null;
		Double value = null;
	}
}
