package org.atorma.robot.learning.cliffworld;

import org.atorma.robot.mdp.RewardFunction;
import org.atorma.robot.mdp.Transition;

/** 
 * Cliff world rewards scaled to interval [0,1]. 
 */
public class ScaledCliffWorldRewardFunction implements RewardFunction {

	@Override
	public double getReward(Transition transition) {
		CliffWorldState toState = (CliffWorldState) transition.getToState();
		if (toState.isCliff()) {
			return 0;
		} else if (toState.isGoal()){
			return 1.0;
		} else {
			return 0.5;
		}
	}

}
