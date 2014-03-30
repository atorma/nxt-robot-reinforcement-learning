package org.atorma.robot.learning.montecarlo;

import static java.lang.Math.*;

import java.util.*;

import org.atorma.robot.discretization.StateDiscretizer;
import org.atorma.robot.learning.HashMapQTable;
import org.atorma.robot.learning.QTable;
import org.atorma.robot.mdp.*;
import org.atorma.robot.policy.DiscretePolicy;

/**
 * Upper confidence bounds in tree.
 */
public class FirstVisitUctPlanning implements DiscretePolicy {

	private ForwardModel model;
	private StateDiscretizer stateDiscretizer;
	private QTable longTermQValues;
	private QTable uctQValues;
	private int horizon;
	private double uctConstant;
	private double discountFactor;
	
	private Map<DiscretizedStateAction, Integer> stateActionVisits;
	private Map<Integer, Integer> stateVisits;
	
	private State startState;

	private Random random = new Random();
	

	public FirstVisitUctPlanning(UctPlanningParameters parameters) {
		this.model = parameters.model;
		this.stateDiscretizer = parameters.stateDiscretizer;
		this.longTermQValues = parameters.qTable;
		this.horizon = parameters.horizon;
		this.uctConstant = parameters.uctConstant;
		this.discountFactor = parameters.discountFactor;
		
		//stateVisits = new HashMap<>();
		//stateActionVisits = new HashMap<>();
		uctQValues = new HashMapQTable(Double.NEGATIVE_INFINITY); // infinity marks (s,a) for which we don't yet have an UCT Q-value
	}
	
	public void setRolloutStartState(State startState) {
		this.startState = startState;
		stateVisits = new HashMap<>();
		stateActionVisits = new HashMap<>();
		uctQValues = new HashMapQTable(Double.NEGATIVE_INFINITY); // infinity marks (s,a) for which we don't yet have an UCT Q-value
	}
	
	public void performRollouts(int num) {
		for (int i = 0; i < num; i++) {
			performRollout();
		}
	}
	
	private void performRollout() {	
		State state = startState;	
		double totalReward = rolloutAndUpdateUctQValues(state, 0, new HashSet<DiscretizedStateAction>());
	}
	
	private double rolloutAndUpdateUctQValues(State state, int depth, Set<DiscretizedStateAction> visitedInRollout) {
		if (model.getAllowedActions(state).isEmpty() || depth > horizon) {
			return 0; // terminal state, end recursion and return reward 0
		}
		
		int stateId = stateDiscretizer.getId(state);
		incrementVisits(stateId);
		
		// Always perform unexplored action first
		// Otherwise select action based on UCT exploration policy
		DiscreteAction action = getUnvisitedAction(state, stateId);
		if (action == null) {
			action = getUctAction(state, stateId);
		}
		incrementVisits(stateId, action.getId());
		TransitionReward tr = model.simulateAction(new StateAction(state, action));
		//System.out.println(tr);
		
		DiscretizedStateAction stateIdActionId = new DiscretizedStateAction(stateId, action.getId());
		boolean firstVisit = visitedInRollout.add(stateIdActionId);

		// Total discounted return following visit to (s,a). Note: recursive.
		double ret = tr.getReward() + discountFactor*rolloutAndUpdateUctQValues(tr.getToState(), depth + 1, visitedInRollout);
		//System.out.println("Total reward " + ret);
		
		// Update UCT Q-value
		if (firstVisit) {
			double oldQ = getUctQValue(stateId, action.getId());
			int nsa = getNumberOfVisits(stateId, action.getId());
			double newQ = oldQ + 1.0/nsa * (ret - oldQ);
			uctQValues.setValue(stateIdActionId, newQ);
		}
		
		return ret;
	}
	
	/*
	private void performRollout() {	
		State state = startState;	
		List<Double> cumulativeRewards = new ArrayList<Double>(horizon);
		List<DiscretizedStateAction> trajectory = new ArrayList<>(horizon);
		
		for (int s = 0; s < horizon; s++) {	
			
			int stateId = stateDiscretizer.getId(state);
			incrementVisits(stateId);
			
			if (model.getAllowedActions(state).isEmpty()) {
				break; // we're at end state
			}
			
			// Always perform unexplored action first
			// Otherwise select action based on UCT exploration policy
			DiscreteAction chosenAction = getUnvisitedAction(state, stateId);
			if (chosenAction == null) {
				chosenAction = getUctAction(state, stateId);
			}
			trajectory.add(new DiscretizedStateAction(stateId, chosenAction.getId()));
			incrementVisits(stateId, chosenAction.getId());
			
			StateAction chosenStateAction = new StateAction(state, chosenAction);
			TransitionReward tr = model.simulateAction(chosenStateAction);
			cumulativeRewards.add(tr.getReward());

			// Next state in trajectory
			state = tr.getToState();
		}

		// Update Q-values
		Set<DiscretizedStateAction> visited = new HashSet<>();
		double totalReturn = cumulativeRewards.get(trajectory.size() - 1);
		for (int i = 0; i < trajectory.size(); i++) {
			DiscretizedStateAction stateAction = trajectory.get(i);
			if (!visited.contains(stateAction)) {
				int stateId = stateAction.getStateId();
				int actionId = stateAction.getActionId();
				
				double firstVisitReturn = totalReturn - (i == 0 ? 0 : cumulativeRewards.get(i-1));
				double oldQ = getUctQValue(stateId, actionId);
				int nsa = getNumberOfVisits(stateId, actionId);
				double newQ = oldQ + 1/nsa * (firstVisitReturn- oldQ);
				uctQValues.setValue(stateAction, newQ);
				
				visited.add(stateAction);
			}
		}
	}
	*/

	private DiscreteAction getUnvisitedAction(State state, int stateId) {
		List<DiscreteAction> unvisited = new ArrayList<DiscreteAction>();
		for (DiscreteAction action : model.getAllowedActions(state)) {
			if (getNumberOfVisits(stateId, action.getId()) == 0) { 
				unvisited.add(action);
			}
		}
		
		if (unvisited.isEmpty()) {
			return null;
		} else {
			return unvisited.get(random.nextInt(unvisited.size()));
		}
	}
	
	
	private DiscreteAction getUctAction(State state, int stateId) {

		double bestValue = Double.NEGATIVE_INFINITY;
		List<DiscreteAction> bestActions = new ArrayList<>();
		
		for (DiscreteAction action : model.getAllowedActions(state)) {
			double qUct = getUctQValue(stateId, action.getId());
			int ns = getNumberOfVisits(stateId);
			int nsa = getNumberOfVisits(stateId, action.getId());
			double qEx = qUct + uctConstant*sqrt(2*log(ns)/nsa); // Q-value + UCT exploration term
			
			if (qEx > bestValue) {
				bestActions.clear();
				bestActions.add(action);
				bestValue = qEx;
			} else if (qEx == bestValue) {
				bestActions.add(action);
			}
		}
		
		DiscreteAction bestAction = bestActions.get(random.nextInt(bestActions.size()));
		return bestAction;
	}
	
	public double getUctQValue(int stateId, int actionId) {
		DiscretizedStateAction stateAction = new DiscretizedStateAction(stateId, actionId);
		if (uctQValues.getValue(stateAction) == Double.NEGATIVE_INFINITY) {
			double q = longTermQValues.getValue(stateAction);
			uctQValues.setValue(stateAction, q);
			return q;
		} else {
			return uctQValues.getValue(stateAction);
		}
				
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
	
	private int getNumberOfVisits(int stateId) {
		if (stateVisits.containsKey(stateId)) {
			return stateVisits.get(stateId);
		} else {
			return 0;
		}
	}
	
	private void incrementVisits(int stateId) {
		int visits = getNumberOfVisits(stateId);
		stateVisits.put(stateId, visits + 1);
	}
	


	@Override
	public Integer getActionId(int stateId) {
		return uctQValues.getActionId(stateId);
	}
	
}
