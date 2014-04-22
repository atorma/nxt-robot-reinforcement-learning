package org.atorma.robot.learning.montecarlo;

import java.util.*;

import org.atorma.robot.discretization.StateDiscretizer;
import org.atorma.robot.learning.HashMapQTable;
import org.atorma.robot.learning.QTable;
import org.atorma.robot.mdp.*;
import org.atorma.robot.policy.DiscretePolicy;

public class FirstVisitOnPolicyMonteCarlo implements DiscretePolicy {

	private ForwardModel model;
	private StateDiscretizer stateDiscretizer;
	private DiscretePolicy policy;
	private int horizon;
	private QTable planningQValues;
	private double discountFactor;

	private Map<DiscretizedStateAction, Integer> stateActionVisits;
	private State startState;

	public FirstVisitOnPolicyMonteCarlo(
			ForwardModel model, 
			StateDiscretizer stateDiscretizer, 
			DiscretePolicy policy, 
			int horizon, 
			double discountFactor) {
		
		this.model = model;
		this.stateDiscretizer = stateDiscretizer;
		this.policy = policy;
		this.horizon = horizon;
		this.discountFactor = discountFactor;
	}
	
	public void setRolloutStartState(State startState) {
		this.startState = startState;
		stateActionVisits = new HashMap<>();
		planningQValues = new HashMapQTable(); 
	}
	
	public void performRollouts(int num) {
		for (int i = 0; i < num; i++) {
			performRollout(startState, 0, new HashSet<DiscretizedStateAction>());
		}
	}

	private double performRollout(State state, int depth, Set<DiscretizedStateAction> visitedInRollout) {
		if (model.getAllowedActions(state).isEmpty() || depth > horizon) {
			return 0; // terminal state, end recursion and return reward 0
		}
		
		int stateId = stateDiscretizer.getId(state);
		TransitionReward tr = simulatePolicy(state, stateId);
		
		int actionId = tr.getAction().getId();
		incrementVisits(stateId, actionId);
		DiscretizedStateAction stateIdActionId = new DiscretizedStateAction(stateId, actionId);
		boolean firstVisit = visitedInRollout.add(stateIdActionId);

		// Total discounted return following visit to (s,a). Note: recursive.
		double ret = tr.getReward() + discountFactor*performRollout(tr.getToState(), depth + 1, visitedInRollout);
		
		// Update planning Q-value
		if (firstVisit) {
			double oldQ = planningQValues.getValue(stateIdActionId);
			int nsa = getNumberOfVisits(stateId, actionId);
			double newQ = oldQ + 1.0/nsa * (ret - oldQ);
			planningQValues.setValue(stateIdActionId, newQ);
		}
		
		return ret;
	}
	
	private TransitionReward simulatePolicy(State fromState, int stateId) {
		int actionId = policy.getActionId(stateId);
		DiscreteAction action = null;
		for (DiscreteAction a : model.getAllowedActions(fromState)) {
			if (a.getId() == actionId) {
				action = a;
			}
		}
		return model.simulateAction(new StateAction(fromState, action));
	}
	
	public int getNumberOfVisits(int stateId, int actionId) {
		DiscretizedStateAction stateAction = new DiscretizedStateAction(stateId, actionId);
		if (stateActionVisits.containsKey(stateAction)) {
			return stateActionVisits.get(stateAction);
		} else {
			return 0;
		}
	}
	
	private void incrementVisits(int stateId, int actionId) {
		int visits = getNumberOfVisits(stateId, actionId);
		stateActionVisits.put(new DiscretizedStateAction(stateId, actionId), visits + 1);
	}

	@Override
	public Integer getActionId(int stateId) {
		return planningQValues.getActionId(stateId);
	}
	
	
}
