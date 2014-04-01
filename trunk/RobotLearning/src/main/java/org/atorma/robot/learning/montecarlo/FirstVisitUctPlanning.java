package org.atorma.robot.learning.montecarlo;

import static java.lang.Math.log;
import static java.lang.Math.sqrt;

import java.util.*;

import org.atorma.robot.discretization.StateDiscretizer;
import org.atorma.robot.learning.HashMapQTable;
import org.atorma.robot.learning.QTable;
import org.atorma.robot.mdp.*;

/**
 * UCT (upper confidence bounds applied to trees) action selection.
 * The best action is selected based on on long-term Q-values
 * learned by another algorithm and temporary planning Q-values that 
 * balance exploration and exploitation. The temporary Q-values
 * are computed by simulating rollouts from given state.
 * <p>
 * This variant computes planning Q-values as return following the
 * first visit to (state, action).
 */
public class FirstVisitUctPlanning {

	private ForwardModel model;
	private StateDiscretizer stateDiscretizer;
	private QTable longTermQValues;
	private HashMapQTable uctQValues;
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
		this.longTermQValues = parameters.longTermQValues;
		this.horizon = parameters.planningHorizon;
		this.uctConstant = parameters.uctConstant;
		this.discountFactor = parameters.discountFactor;
	}
	
	public void setRolloutStartState(State startState) {
		this.startState = startState;
		stateVisits = new HashMap<>();
		stateActionVisits = new HashMap<>();
		uctQValues = new HashMapQTable(); 
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
		incrementVisits(stateId);
		
		// Always perform unexplored action first
		// Otherwise select action based on UCT exploration policy
		DiscreteAction action = getUnvisitedAction(state, stateId);
		if (action == null) {
			action = getUctAction(state, stateId, true); // with exploration
		}
		incrementVisits(stateId, action.getId());
		TransitionReward tr = model.simulateAction(new StateAction(state, action));
		//System.out.println(tr);
		
		DiscretizedStateAction stateIdActionId = new DiscretizedStateAction(stateId, action.getId());
		boolean firstVisit = visitedInRollout.add(stateIdActionId);

		// Total discounted return following visit to (s,a). Note: recursive.
		double ret = tr.getReward() + discountFactor*performRollout(tr.getToState(), depth + 1, visitedInRollout);
		//System.out.println("Total reward " + ret);
		
		// Update UCT planning Q-values
		if (firstVisit) {
			double oldQ = uctQValues.getValue(stateIdActionId);
			int nsa = getNumberOfVisits(stateId, action.getId());
			double newQ = oldQ + 1.0/nsa * (ret - oldQ);
			uctQValues.setValue(stateIdActionId, newQ);
		}
		
		return ret;
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
			
			double qLt = longTermQValues != null ? longTermQValues.getValue(sa) : 0;
			double qUct = uctQValues.getValue(sa);
			
			int ns = getNumberOfVisits(stateId);
			int nsa = getNumberOfVisits(stateId, action.getId());
			double expl = isExploration ? uctConstant*sqrt(2*log(ns)/nsa) : 0;
			
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
