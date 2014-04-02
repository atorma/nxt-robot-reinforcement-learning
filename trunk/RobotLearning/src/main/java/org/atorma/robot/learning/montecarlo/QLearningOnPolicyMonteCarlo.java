package org.atorma.robot.learning.montecarlo;

import org.atorma.robot.discretization.StateDiscretizer;
import org.atorma.robot.learning.*;
import org.atorma.robot.mdp.*;
import org.atorma.robot.policy.DiscretePolicy;

public class QLearningOnPolicyMonteCarlo implements DiscretePolicy {

	private ForwardModel model;
	private DiscreteAction[] allActions;
	private StateDiscretizer stateDiscretizer;
	private DiscretePolicy policy;
	private int horizon;
	
	private QLearning qLearning;
	private QTable planningQValues;
	private double defaultQValue;
	private EligibilityTraces traces;
	private double learningRate;

	private State startState;
	
	
	public QLearningOnPolicyMonteCarlo(QLearningOnPolicyMonteCarloParameters parameters) {
		
		this.model = parameters.model;
		this.allActions = parameters.allActions;
		this.stateDiscretizer = parameters.stateDiscretizer;
		this.policy = parameters.policy;
		this.horizon = parameters.horizon;
		this.learningRate = parameters.learningRate;
		this.traces = parameters.traces;
		this.defaultQValue = parameters.defaultQValue;
		
	}
	
	public void setRolloutStartState(State startState) {
		this.startState = startState;
		traces.clear();
		planningQValues = new HashMapQTable(defaultQValue, allActions);
		qLearning = new QLearning(learningRate, traces, planningQValues);
	}
	
	public void performRollouts(int num) {
		for (int i = 0; i < num; i++) {
			performRollout(startState);
		}
	}

	private void performRollout(State startState) {
		State state = startState;
		int step = 0;
		while (step < horizon & !model.getAllowedActions(state).isEmpty()) {

			int stateId = stateDiscretizer.getId(state);
			TransitionReward tr = simulatePolicy(state, stateId);

			int actionId = tr.getAction().getId();
	
			int toStateId = stateDiscretizer.getId(tr.getToState());
			DiscretizedTransitionReward discrTr = new DiscretizedTransitionReward(stateId, actionId, toStateId, tr.getReward());
			qLearning.update(discrTr);
			
			state = tr.getToState();
			step++;
		}
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
	
	
	@Override
	public Integer getActionId(int stateId) {
		return planningQValues.getActionId(stateId);
	}
	
	
}
