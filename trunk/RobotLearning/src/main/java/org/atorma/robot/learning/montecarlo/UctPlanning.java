package org.atorma.robot.learning.montecarlo;

import java.util.*;

import org.atorma.robot.discretization.StateDiscretizer;
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
	private int horizon;
	
	private Map<DiscretizedStateAction, Integer> visits;
	
	private State startState;

	private Random random = new Random();
	
	public UctPlanning(UctPlanningParameters parameters) {
		this.model = parameters.model;
		this.stateDiscretizer = parameters.stateDiscretizer;
		this.longTermQValues = parameters.qTable;
		this.horizon = parameters.horizon;
	}
	
	public void setRolloutStartState(State startState) {
		this.startState = startState;
		visits = new HashMap<>();
	}
	
	public void performRollouts(int num) {
		for (int i = 0; i < num; i++) {
			performRollout();
		}
	}
	
	private void performRollout() {
		Queue<State> stateQueue = new LinkedList<>();		
		stateQueue.add(startState);
		
		for (int s = 0; s < horizon; s++) {
			State state = stateQueue.poll();	
			State nextState = null;			
			for (DiscreteAction action : model.getAllowedActions(state)) {
				TransitionReward tr = model.simulateAction(new StateAction(startState, action));
				
				if (getNumberOfVisits(tr.getFromStateAction()) == 0) { // Always perform unexplored action
					stateQueue.add(tr.getToState());
				} else { // Otherwise select action based on UCT tree policy
					
				}
			}
		}
	}
	
	private int getNumberOfVisits(StateAction stateAction) {
		int stateId = stateDiscretizer.getId(stateAction.getState());
		int actionId = stateAction.getAction().getId();
		DiscretizedStateAction discretizedStateAction = new DiscretizedStateAction(stateId, actionId);
		if (visits.containsKey(discretizedStateAction)) {
			return visits.get(stateAction);
		} else {
			return 0;
		}
	}
	

	@Override
	public Integer getActionId(int stateId) {
		// TODO Auto-generated method stub
		return null;
	}
	
}
