package org.atorma.robot.learning.montecarlo;

import org.atorma.robot.learning.*;
import org.atorma.robot.mdp.*;

public class QLearningUctPlanning extends AbstractUctPlanning {
	
	private double learningRate;
	private EligibilityTraces traces;
	private QLearning qLearning;
	private DiscreteAction[] actions;
	

	public QLearningUctPlanning(QLearningUctPlanningParameters parameters) {
		super(parameters);
		this.uctConstant = parameters.uctConstant;
		this.learningRate = parameters.learningRate;
		this.traces = parameters.eligibilityTraces;
		this.actions = parameters.allActions;
	}

	

	@Override
	public void setRolloutStartState(State state) {
		super.setRolloutStartState(state);
		traces.clear();
		uctQValues = new HashMapQTable(0, actions);
		qLearning = new QLearning(learningRate, traces, uctQValues);
	}



	@Override
	protected void performRollout(State startState) {
		State state = startState;
		int step = 0;
		while (step < horizon && !model.getAllowedActions(state).isEmpty()) {
			
			int stateId = stateDiscretizer.getId(state);
			
			DiscreteAction action = chooseAction(state, stateId);
			
			incrementVisits(stateId);
			incrementVisits(stateId, action.getId());
		
			TransitionReward tr = model.simulateAction(new StateAction(state, action));

			int toStateId = stateDiscretizer.getId(tr.getToState());
			DiscretizedTransitionReward discrTr = new DiscretizedTransitionReward(stateId, action.getId(), toStateId, tr.getReward());
			qLearning.update(discrTr);
			
			state = tr.getToState();
			step++;
		}
		
	}
	
	


	
	
}
