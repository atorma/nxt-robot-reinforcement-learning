package org.atorma.robot.mdp;

public class DiscretizedTransitionWithReward extends DiscretizedTransition {

	private final double reward;
	
	public DiscretizedTransitionWithReward(int fromStateId, int byActionId, int toStateId, double reward) {
		super(fromStateId, byActionId, toStateId);
		this.reward = reward;
	}
	
	public DiscretizedTransitionWithReward(DiscretizedTransition transition, double reward) {
		super(transition.getFromStateId(), transition.getByActionId(), transition.getToStateId());
		this.reward = reward;
	}

	public double getReward() {
		return reward;
	}
	
	
}
