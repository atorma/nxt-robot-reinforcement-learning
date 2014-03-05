package org.atorma.robot.learning.cliffworld;

import org.atorma.robot.mdp.RewardFunction;
import org.atorma.robot.mdp.Transition;

public class CliffWorldRewardFunction implements RewardFunction<CliffWorldState, CliffWorldAction> {

	@Override
	public double getReward(Transition<CliffWorldState, CliffWorldAction> transition) {
		CliffWorldState toState = transition.getToState();
		if (toState.isCliff()) {
			return -100;
		} else {
			return -1;
		}
	}

}
