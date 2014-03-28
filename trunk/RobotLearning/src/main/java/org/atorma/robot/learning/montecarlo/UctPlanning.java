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
public class UctPlanning implements DiscretePolicy {

	private ForwardModel model;
	private StateDiscretizer stateDiscretizer;
	private QTable longTermQValues;
	private QTable uctQValues;
	private int horizon;
	private double uctConstant;
	private double discountFactor;
	
	private Map<DiscretizedStateAction, Integer> visits;
	
	private State startState;

	private Random random = new Random();
	
	
	
	public UctPlanning(UctPlanningParameters parameters) {
		this.model = parameters.model;
		this.stateDiscretizer = parameters.stateDiscretizer;
		this.longTermQValues = parameters.qTable;
		this.horizon = parameters.horizon;
		this.uctConstant = parameters.uctConstant;
		this.discountFactor = parameters.discountFactor;
	}
	
	public void setRolloutStartState(State startState) {
		this.startState = startState;
		visits = new HashMap<>();
		uctQValues = new HashMapQTable(Double.NEGATIVE_INFINITY); // infinity marks (s,a) for which we don't yet have an UCT Q-value
	}
	
	public void performRollouts(int num) {
		for (int i = 0; i < num; i++) {
			performRollout();
		}
	}
	
	private void performRollout() {
		List<TransitionReward> trajectory = simulateTrajectory();
		Map<DiscretizedStateAction, Double> returns = computeReturns(trajectory);
		updateUctQValues(returns);
	}
	
	// Simulate a trajectory from the startState and updates the number of visits to each (s,a)
	private List<TransitionReward> simulateTrajectory() {
		List<TransitionReward> trajectory = new ArrayList<>(horizon);		
		State state = startState;	
		
		for (int s = 0; s < horizon; s++) {	
			
			if (model.getAllowedActions(state).isEmpty()) {
				break; // we're at end state
			}
			
			// Always perform unexplored action first
			// Otherwise select action based on UCT tree policy
			DiscreteAction chosenAction = getUnvisitedAction(state);
			if (chosenAction == null) {
				chosenAction = getBestAction(state);
			}
			
			StateAction chosenStateAction = new StateAction(state, chosenAction);
			TransitionReward tr = model.simulateAction(chosenStateAction);
			trajectory.add(tr);
			
			// Update visits to (s,a)
			DiscretizedStateAction stateIdActionId = discretize(chosenStateAction);
			int numVisits = getNumberOfVisits(stateIdActionId);
			visits.put(stateIdActionId, numVisits + 1);
			
			// Next state in trajectory
			state = tr.getToState();
		}
		
		return trajectory;
	}

	// Compute the total reward and returns following first visit to each (s, a) in the trajectory
	private Map<DiscretizedStateAction, Double> computeReturns(List<TransitionReward> trajectory) {
		Map<DiscretizedStateAction, Double> firstVisitRewards = new HashMap<>();
		double totalReward = 0;
		int step = trajectory.size();
		while(!trajectory.isEmpty()) {
			step--;
			int lastIndex = trajectory.size() - 1;
			TransitionReward tr = trajectory.get(lastIndex);
			trajectory.remove(lastIndex);
			
			totalReward += Math.pow(discountFactor, step) * tr.getReward();
			
			int fromStateId = stateDiscretizer.getId(tr.getFromState());
			int actionId = tr.getAction().getId();
			firstVisitRewards.put(new DiscretizedStateAction(fromStateId, actionId), totalReward);
		}
		return firstVisitRewards;
	}
	
	private void updateUctQValues(Map<DiscretizedStateAction, Double> returns) {
		for (Map.Entry<DiscretizedStateAction, Double> saReward : returns.entrySet()) {
			DiscretizedStateAction sa = saReward.getKey();
			double oldQ = getUctQValue(sa);
			int nsa = getNumberOfVisits(sa);
			double newQ = oldQ + 1/nsa * (saReward.getValue() - oldQ);
			uctQValues.setValue(sa, newQ);
		}
	}
	
	
	private DiscreteAction getUnvisitedAction(State state) {
		for (DiscreteAction action : model.getAllowedActions(state)) {
			if (getNumberOfVisits(new StateAction(state, action)) == 0) { // Always perform unexplored action first
				return action;
			}
		}
		return null;
	}
	
	
	private DiscreteAction getBestAction(State state) {
		int stateId = stateDiscretizer.getId(state);
		
		// Compute number of visits to state
		int ns = 0;
		for (DiscreteAction action : model.getAllowedActions(state)) {
			ns += getNumberOfVisits(new DiscretizedStateAction(stateId, action.getId()));
		}
		
		
		// Find best action, breaking ties at random
		
		double bestValue = Double.NEGATIVE_INFINITY;
		List<DiscreteAction> bestActions = new ArrayList<>();
		
		for (DiscreteAction action : model.getAllowedActions(state)) {
			DiscretizedStateAction stateIdActionId = new DiscretizedStateAction(stateId, action.getId());
			double q = getUctQValue(stateIdActionId);
			int nsa = getNumberOfVisits(stateIdActionId);
			double qUct = q + uctConstant*sqrt(log(ns)/nsa);
			
			if (qUct > bestValue) {
				bestActions.clear();
				bestActions.add(action);
				bestValue = qUct;
			} else if (qUct == bestValue) {
				bestActions.add(action);
			}
		}
		
		DiscreteAction bestAction = bestActions.get(random.nextInt(bestActions.size()));
		return bestAction;
	}
	
	private double getUctQValue(DiscretizedStateAction stateAction) {
		if (uctQValues.getValue(stateAction) == Double.NEGATIVE_INFINITY) {
			double q = longTermQValues.getValue(stateAction);
			uctQValues.setValue(stateAction, q);
			return q;
		} else {
			return uctQValues.getValue(stateAction);
		}
				
	}

	private int getNumberOfVisits(StateAction stateAction) {
		DiscretizedStateAction discretizedStateAction = discretize(stateAction);
		return getNumberOfVisits(discretizedStateAction);
	}
	
	private int getNumberOfVisits(DiscretizedStateAction stateAction) {
		if (visits.containsKey(stateAction)) {
			return visits.get(stateAction);
		} else {
			return 0;
		}
	}
	
	private DiscretizedStateAction discretize(StateAction stateAction) {
		int stateId = stateDiscretizer.getId(stateAction.getState());
		int actionId = stateAction.getAction().getId();
		DiscretizedStateAction discretizedStateAction = new DiscretizedStateAction(stateId, actionId);
		return discretizedStateAction;
	}
	

	@Override
	public Integer getActionId(int stateId) {
		return uctQValues.getActionId(stateId);
	}
	
}
