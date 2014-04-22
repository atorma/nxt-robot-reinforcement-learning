package org.atorma.robot.learning.cliffworld;

import org.atorma.robot.mdp.Transition;

public class ModifiedCliffWorldRewardFunction extends CliffWorldRewardFunction {

	@Override
	public double getReward(Transition transition) {
		CliffWorldState toState = (CliffWorldState) transition.getToState();
		if (toState.isGoal()) {
			return 100.0;
		} else {
			return super.getReward(transition);
		}
	}
	
}