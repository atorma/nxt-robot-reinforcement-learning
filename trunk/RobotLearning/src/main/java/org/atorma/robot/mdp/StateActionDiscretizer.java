package org.atorma.robot.mdp;

import org.atorma.robot.discretization.VectorDiscretizer;

public class StateActionDiscretizer {

	private VectorDiscretizer stateDiscretizer;
	private RewardFunction rewardFunction;
	
	public StateActionDiscretizer(VectorDiscretizer stateDiscretizer, RewardFunction rewardFunction) {
		this.stateDiscretizer = stateDiscretizer;
		this.rewardFunction = rewardFunction;
	}
	
	public DiscretizedStateAction discretize(State state, DiscreteAction action) {
		int stateId = stateDiscretizer.getId(state.getValues());
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
		int fromStateId = stateDiscretizer.getId(fromState.getValues());
		int byActionId = byAction.getId();
		int toStateId = stateDiscretizer.getId(toState.getValues());
		return new DiscretizedTransition(fromStateId, byActionId, toStateId);
	}
	
	public DiscretizedTransitionWithReward discretize(State fromState, DiscreteAction byAction, State toState, double reward) {
		return new DiscretizedTransitionWithReward(discretize(fromState, byAction, toState), reward);
	}
	
	public DiscretizedTransitionWithReward discretize(TransitionWithReward transition) {
		return new DiscretizedTransitionWithReward(discretize((Transition) transition), transition.getReward());
	}
	
	public DiscretizedTransitionWithReward discretizeAndComputeReward(State fromState, DiscreteAction byAction, State toState) {
		Transition transition = new Transition(fromState, byAction, toState);
		return discretizeAndComputeReward(transition);
	}
	
	public DiscretizedTransitionWithReward discretizeAndComputeReward(Transition transition) {
		double reward = rewardFunction.getReward(transition);
		DiscretizedTransition discretizedTransition = discretize(transition);
		return new DiscretizedTransitionWithReward(discretizedTransition, reward);
	}
}
