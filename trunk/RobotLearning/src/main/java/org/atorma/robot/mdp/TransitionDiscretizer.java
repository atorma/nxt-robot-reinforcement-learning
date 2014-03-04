package org.atorma.robot.mdp;

import org.atorma.robot.discretization.VectorDiscretizer;

public class TransitionDiscretizer<S extends State, A extends DiscreteAction> {

	private VectorDiscretizer stateDiscretizer;
	private RewardFunction<S, A> rewardFunction;
	
	public TransitionDiscretizer(VectorDiscretizer stateDiscretizer, RewardFunction<S, A> rewardFunction) {
		this.stateDiscretizer = stateDiscretizer;
		this.rewardFunction = rewardFunction;
	}
	
	public DiscretizedTransition discretize(Transition<S, A> transition) {
		return discretize(transition.getFromState(), transition.getAction(), transition.getToState());
	}
	
	public DiscretizedTransition discretize(S fromState, A byAction, S toState) {
		int fromStateId = stateDiscretizer.getId(fromState.getValues());
		int byActionId = byAction.getId();
		int toStateId = stateDiscretizer.getId(toState.getValues());
		return new DiscretizedTransition(fromStateId, byActionId, toStateId);
	}
	
	public DiscretizedTransitionWithReward discretize(S fromState, A byAction, S toState, double reward) {
		return new DiscretizedTransitionWithReward(discretize(fromState, byAction, toState), reward);
	}
	
	public DiscretizedTransitionWithReward discretize(TransitionWithReward<S, A> transition) {
		return new DiscretizedTransitionWithReward(discretize((Transition<S, A>) transition), transition.getReward());
	}
	
	public DiscretizedTransitionWithReward discretizeAndComputeReward(S fromState, A byAction, S toState) {
		Transition<S, A> transition = new Transition<S, A>(fromState, byAction, toState);
		return discretizeAndComputeReward(transition);
	}
	
	public DiscretizedTransitionWithReward discretizeAndComputeReward(Transition<S, A> transition) {
		double reward = rewardFunction.getReward(transition);
		DiscretizedTransition discretizedTransition = discretize(transition);
		return new DiscretizedTransitionWithReward(discretizedTransition, reward);
	}
}
