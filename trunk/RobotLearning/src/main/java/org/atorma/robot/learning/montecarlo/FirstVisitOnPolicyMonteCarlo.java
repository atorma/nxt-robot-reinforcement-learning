package org.atorma.robot.learning.montecarlo;

import java.util.*;

import org.atorma.robot.discretization.StateDiscretizer;
import org.atorma.robot.learning.DiscreteActionModel;
import org.atorma.robot.learning.QTable;
import org.atorma.robot.mdp.*;
import org.atorma.robot.policy.DiscretePolicy;

public class FirstVisitOnPolicyMonteCarlo {

	private DiscreteActionModel model;
	private StateDiscretizer stateDiscretizer;
	private DiscretePolicy policy;
	private QTable qTable;
	private int horizon;
	private double discountFactor;
	private Map<DiscretizedStateAction, Integer> timesUpdatedMap = new HashMap<>();
	
	private State startState;
	
	
	public FirstVisitOnPolicyMonteCarlo(DiscreteActionModel model, StateDiscretizer stateDiscretizer, DiscretePolicy policy, QTable qTable, int horizon, double discountFactor) {
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

		for (int i = 0; i < num; i++) {
			// Simulate the policy until the horizon or end state
			List<StochasticTransitionReward> visited = new ArrayList<>(horizon);		
			State state = startState;		
			int step = 0;
			while (step < horizon) {
				StochasticTransitionReward tr = simulatePolicy(state);
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
				StochasticTransitionReward tr = visited.get(lastIndex);
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
				int timesUpdated = timesUpdatedMap.get(sa) + 1;
				double newQ = oldQ + 1/timesUpdated * (saReward.getValue() - oldQ);
				qTable.setValue(sa, newQ);
				timesUpdatedMap.put(sa, timesUpdated);
			}
		}
	}
	
	private StochasticTransitionReward simulatePolicy(State fromState) {
		int stateId = stateDiscretizer.getId(fromState);
		int actionId = policy.getActionId(stateId);
		Set<StochasticTransitionReward> outgoing = model.getOutgoingTransitions(fromState);
		
		if (outgoing.isEmpty()) {
			return null;
		}
		
		for (StochasticTransitionReward tr : outgoing) {
			if (tr.getAction().getId() == actionId) {
				return tr;
			}
		}
		throw new IllegalStateException("Model's transitions do not contain action selected by policy");
	}
	
}
