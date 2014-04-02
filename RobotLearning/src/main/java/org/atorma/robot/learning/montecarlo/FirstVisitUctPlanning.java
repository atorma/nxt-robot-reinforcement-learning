package org.atorma.robot.learning.montecarlo;

import java.util.HashSet;
import java.util.Set;

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
public class FirstVisitUctPlanning extends AbstractUctPlanning {

	private double discountFactor;
	
	public FirstVisitUctPlanning(FirstVisitUctPlanningParameters parameters) {
		super(parameters);
		this.discountFactor = parameters.discountFactor;
	}
	
	@Override
	protected void performRollout(State startState) {
		performRollout(startState, 0, new HashSet<DiscretizedStateAction>());
	}
	
	private double performRollout(State state, int depth, Set<DiscretizedStateAction> visitedInRollout) {
		if (model.getAllowedActions(state).isEmpty() || depth > horizon) {
			return 0; // terminal state, end recursion and return reward 0
		}
		
		int stateId = stateDiscretizer.getId(state);
		incrementVisits(stateId);
		
		DiscreteAction action = chooseAction(state, stateId);
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



	

	
}
