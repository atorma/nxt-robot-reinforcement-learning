package org.atorma.robot.learning.cliffworld;

import org.atorma.robot.mdp.RewardFunction;
import org.atorma.robot.mdp.Transition;

public class CliffWorldRewardFunction implements RewardFunction {

	@Override
	public double getReward(Transition transition) {
		CliffWorldState toState = (CliffWorldState) transition.getToState();
		if (toState.isCliff()) {
			return -100;
		} else {
			return -1;
		}
	}

}
