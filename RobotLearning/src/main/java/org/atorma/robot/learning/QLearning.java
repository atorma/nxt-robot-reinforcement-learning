package org.atorma.robot.learning;

import java.util.*;

import org.atorma.robot.discretization.VectorDiscretizer;
import org.atorma.robot.mdp.*;
import org.atorma.robot.policy.DiscretePolicy;
import org.atorma.robot.policy.StateIdToActionIdMap;

public class QLearning<S extends State, A extends DiscreteAction> implements DiscretePolicy {

	private Map<StateIdActionId, Double> qTable = new HashMap<>();
	private StateIdToActionIdMap stateIdToBestActionIdMap = new StateIdToActionIdMap();
	
	private Set<Integer> stateIds = new HashSet<>();
	private Set<Integer> actionIds = new HashSet<>();
	private VectorDiscretizer stateDiscretizer;
	
	private RewardFunction<S, A> rewardFunction;
	private double learningRate;
	private double discountFactor;
	
	private double accumulatedReward = 0;
	
	private double defaultStateActionValue = 0;
	
	public QLearning(VectorDiscretizer stateDiscretizer, RewardFunction<S, A> rewardFunction, double learningRate, double discountFactor) {
		this.stateDiscretizer = stateDiscretizer;
		this.rewardFunction = rewardFunction;
		this.learningRate = learningRate;
		this.discountFactor = discountFactor;
	}


	public void update(Transition<S, A> transition) {
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
		updateBestAction(fromStateId);
	}
	

	private double getQValue(StateIdActionId stateActionIds) {
		if (!qTable.containsKey(stateActionIds)) {
			qTable.put(stateActionIds, defaultStateActionValue);
		}
		return qTable.get(stateActionIds);
	}
	
	private void updateBestAction(int stateId) {
		double bestActionValue = Double.NEGATIVE_INFINITY;
		Integer bestActionId = null;
		
		for (int actionId : actionIds) {
			double q = getQValue(new StateIdActionId(stateId, actionId));
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
