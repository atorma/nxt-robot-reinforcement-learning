package org.atorma.robot.learning.montecarlo;

import java.util.*;

import org.atorma.robot.discretization.StateDiscretizer;
import org.atorma.robot.learning.QTable;
import org.atorma.robot.mdp.*;
import org.atorma.robot.policy.DiscretePolicy;

public class FirstVisitOnPolicyMonteCarlo {

	private ForwardModel model;
	private StateDiscretizer stateDiscretizer;
	private DiscretePolicy policy;
	private QTable qTable;
	private int horizon;
	private double discountFactor;
	private Map<DiscretizedStateAction, Integer> timesUpdatedMap = new HashMap<>();
	
	private State startState;
	
	
	public FirstVisitOnPolicyMonteCarlo(ForwardModel model, StateDiscretizer stateDiscretizer, DiscretePolicy policy, QTable qTable, int horizon, double discountFactor) {
		this.model = model;
		this.stateDiscretizer = stateDiscretizer;
		this.policy = policy;
		this.qTable = qTable;
		this.horizon = horizon;
		this.discountFactor = discountFactor;
	}
	
	public void setRolloutStartState(State startState) {
		this.startState = startState;
	}
	
	public void performRollouts(int num) {
		
		if (startState == null) {
			return;
		}

		for (int i = 0; i < num; i++) {
			// Simulate the policy until the horizon or end state
			List<TransitionReward> visited = new ArrayList<>(horizon);		
			State state = startState;		
			int step = 0;
			while (step < horizon) {
				TransitionReward tr = simulatePolicy(state);
				if (tr != null) {
					visited.add(tr);
					state = tr.getToState();
				} else {
					break; // state is an end state
				}
				step++;
			}
			
			// Compute the total reward and returns following first visit to (s, a) 
			// in reverse order.
			Map<DiscretizedStateAction, Double> firstVisitRewards = new HashMap<>();
			double totalReward = 0;
			while(!visited.isEmpty()) {
				step--;
				int lastIndex = visited.size() - 1;
				TransitionReward tr = visited.get(lastIndex);
				visited.remove(lastIndex);
				
				totalReward += Math.pow(discountFactor, step) * tr.getReward();
				
				int fromStateId = stateDiscretizer.getId(tr.getFromState());
				int actionId = tr.getAction().getId();
				firstVisitRewards.put(new DiscretizedStateAction(fromStateId, actionId), totalReward);
			}
			
			// Update Q-value averages
			for (Map.Entry<DiscretizedStateAction, Double> saReward : firstVisitRewards.entrySet()) {
				DiscretizedStateAction sa = saReward.getKey();
				double oldQ = qTable.getValue(sa);
				int timesUpdated = timesUpdatedMap.get(sa) != null ? timesUpdatedMap.get(sa) : 0;
				timesUpdated++;
				double newQ = oldQ + 1/timesUpdated * (saReward.getValue() - oldQ);
				qTable.setValue(sa, newQ);
				timesUpdatedMap.put(sa, timesUpdated);
			}
		}
	}
	
	private TransitionReward simulatePolicy(State fromState) {
		Set<? extends DiscreteAction> actions = model.getAllowedActions(fromState);
		if (actions.isEmpty()) {
			return null;
		}
		
		int stateId = stateDiscretizer.getId(fromState);
		int actionId = policy.getActionId(stateId);
		DiscreteAction action = null;
		for (DiscreteAction a : actions) {
			if (a.getId() == actionId) {
				action = a;
			}
		}
		
		return model.simulateAction(new StateAction(fromState, action));
	}

	
}
