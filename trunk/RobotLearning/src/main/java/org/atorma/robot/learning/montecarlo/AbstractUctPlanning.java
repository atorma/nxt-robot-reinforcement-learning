package org.atorma.robot.learning.montecarlo;

import static java.lang.Math.log;
import static java.lang.Math.sqrt;

import java.util.*;

import org.atorma.robot.discretization.StateDiscretizer;
import org.atorma.robot.learning.HashMapQTable;
import org.atorma.robot.learning.QTable;
import org.atorma.robot.mdp.*;

public abstract class AbstractUctPlanning {

	protected ForwardModel model;
	protected StateDiscretizer stateDiscretizer;
	
	protected QTable longTermQValues;
	
	protected int horizon;
	protected double uctConstant;
	protected double discountFactor;
	protected QTable uctQValues = new HashMapQTable();
	
	protected Map<DiscretizedStateAction, Integer> stateActionVisits = new HashMap<>();
	protected Map<Integer, Integer> stateVisits = new HashMap<>();
	
	protected State startState;

	private Random random = new Random();
	
	
	public AbstractUctPlanning(UctPlanningParameters parameters) {
		this.model = parameters.model;
		this.stateDiscretizer = parameters.stateDiscretizer;
		this.longTermQValues = parameters.longTermQValues;
		this.horizon = parameters.planningHorizon;
		this.uctConstant = parameters.uctConstant;
	}
	
	public void setRolloutStartState(State state) {
		startState = state;
		stateVisits.clear();
		stateActionVisits.clear();
		uctQValues = new HashMapQTable(); 
	}
	
	public void performRollouts(int num) {
		if (startState == null) {
			return;
		}
		for (int i = 0; i < num; i++) {
			performRollout(startState);
		}
	}
	
	protected abstract void performRollout(State startState);
		
	
	/**
	 * Chooses a random untried action in given state. When no untried
	 * actions, chooses the best action according to UCT tree policy:
	 * the one that maximizes Q(s,a)_longTerm + Q(s,a)_planning + UCT_exploration_term.  
	 */
	protected DiscreteAction chooseAction(State state, int stateId) {
		// Always perform unexplored action first
		// Otherwise select action based on UCT exploration policy
		DiscreteAction action = getUnvisitedAction(state, stateId);
		if (action == null) {
			action = getUctAction(state, stateId, true); // with exploration
		} 
		return action;
	}

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

	private DiscreteAction getUctAction(State state, int stateId, boolean isExploration) {

		double bestValue = Double.NEGATIVE_INFINITY;
		List<DiscreteAction> bestActions = new ArrayList<>();
		
		for (DiscreteAction action : model.getAllowedActions(state)) {
			DiscretizedStateAction sa = new DiscretizedStateAction(stateId, action.getId());
			
			double qLt = longTermQValues != null ? 0.5 * longTermQValues.getValue(sa) : 0;
			double qUct =longTermQValues != null ? 0.5 * uctQValues.getValue(sa) : uctQValues.getValue(sa);
			
			int ns = getNumberOfVisits(stateId);
			int nsa = getNumberOfVisits(stateId, action.getId());
			double expl = isExploration ? uctConstant*sqrt(log(ns)/nsa) : 0;
			
//			System.out.println("qLt="+qLt+", qUct="+qUct+", expl="+expl);
			
			double q = qLt + qUct + expl; 
			
			if (q > bestValue) {
				bestActions.clear();
				bestActions.add(action);
				bestValue = q;
			} else if (q == bestValue) {
				bestActions.add(action);
			}
		}
		
		DiscreteAction bestAction = bestActions.get(random.nextInt(bestActions.size()));
		return bestAction;
	}
	
	protected int getNumberOfVisits(int stateId, int actionId) {
		DiscretizedStateAction stateAction = new DiscretizedStateAction(stateId, actionId);
		if (stateActionVisits.containsKey(stateAction)) {
			return stateActionVisits.get(stateAction);
		} else {
			return 0;
		}
	}
	
	protected void incrementVisits(int stateId, int actionId) {
		int visits = getNumberOfVisits(stateId, actionId);
		stateActionVisits.put(new DiscretizedStateAction(stateId, actionId), visits + 1);
	}
	
	protected int getNumberOfVisits(int stateId) {
		if (stateVisits.containsKey(stateId)) {
			return stateVisits.get(stateId);
		} else {
			return 0;
		}
	}
	
	protected void incrementVisits(int stateId) {
		int visits = getNumberOfVisits(stateId);
		stateVisits.put(stateId, visits + 1);
	}
	
	
	/**
	 * Returns the planned action for the given state after the rollouts performed
	 * so far. The input state is typically the rollout start state, but its
	 * close successors can also be visited enough times for reasonable value estimates
	 * for action selection.
	 */
	public DiscreteAction getPlannedAction(State state) {
		return getUctAction(state, stateDiscretizer.getId(state), false); // select best action without the exploration term
	}
}
