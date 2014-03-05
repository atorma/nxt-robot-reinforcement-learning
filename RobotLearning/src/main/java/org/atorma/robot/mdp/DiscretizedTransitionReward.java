package org.atorma.robot.mdp;

public class DiscretizedTransitionReward extends DiscretizedTransition {

	private final double reward;
	
	public DiscretizedTransitionReward(int fromStateId, int byActionId, int toStateId, double reward) {
		super(fromStateId, byActionId, toStateId);
		this.reward = reward;
	}
	
	public DiscretizedTransitionReward(DiscretizedTransition transition, double reward) {
		super(transition.getFromStateId(), transition.getByActionId(), transition.getToStateId());
		this.reward = reward;
	}

	public double getReward() {
		return reward;
	}
	
	
}
