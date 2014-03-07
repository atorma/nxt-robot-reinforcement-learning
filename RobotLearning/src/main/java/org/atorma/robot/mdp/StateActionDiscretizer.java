package org.atorma.robot.mdp;

import org.atorma.robot.discretization.StateDiscretizer;

public class StateActionDiscretizer {

	private StateDiscretizer stateDiscretizer;
	private RewardFunction rewardFunction;
	
	public StateActionDiscretizer(StateDiscretizer stateDiscretizer, RewardFunction rewardFunction) {
		this.stateDiscretizer = stateDiscretizer;
		this.rewardFunction = rewardFunction;
	}
	
	public DiscretizedStateAction discretize(State state, DiscreteAction action) {
		int stateId = stateDiscretizer.getId(state);
		int actionId = action.getId();
		return new DiscretizedStateAction(stateId, actionId);
	}
	
	public DiscretizedStateAction discretize(StateAction stateAction) {
		return discretize(stateAction.getState(), stateAction.getAction());
	}
	
	public DiscretizedTransition discretize(Transition transition) {
		return discretize(transition.getFromState(), transition.getAction(), transition.getToState());
	}
	
	public DiscretizedTransition discretize(State fromState, DiscreteAction byAction, State toState) {
		int fromStateId = stateDiscretizer.getId(fromState);
		int byActionId = byAction.getId();
		int toStateId = stateDiscretizer.getId(toState);
		return new DiscretizedTransition(fromStateId, byActionId, toStateId);
	}
	
	public DiscretizedTransitionReward discretize(State fromState, DiscreteAction byAction, State toState, double reward) {
		return new DiscretizedTransitionReward(discretize(fromState, byAction, toState), reward);
	}
	
	public DiscretizedTransitionReward discretize(TransitionReward transition) {
		return new DiscretizedTransitionReward(discretize((Transition) transition), transition.getReward());
	}
	
	public DiscretizedTransitionReward discretizeAndComputeReward(State fromState, DiscreteAction byAction, State toState) {
		Transition transition = new Transition(fromState, byAction, toState);
		return discretizeAndComputeReward(transition);
	}
	
	public DiscretizedTransitionReward discretizeAndComputeReward(Transition transition) {
		double reward = rewardFunction.getReward(transition);
		DiscretizedTransition discretizedTransition = discretize(transition);
		return new DiscretizedTransitionReward(discretizedTransition, reward);
	}
}
