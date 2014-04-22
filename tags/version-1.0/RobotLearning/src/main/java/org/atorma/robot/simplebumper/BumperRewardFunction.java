package org.atorma.robot.simplebumper;

import org.atorma.robot.mdp.RewardFunction;
import org.atorma.robot.mdp.Transition;

public class BumperRewardFunction implements RewardFunction {

	@Override
	public double getReward(Transition transition) {
		CollisionState toState = (CollisionState) transition.getToState();
		BumperAction action = (BumperAction) transition.getAction();
		
		if (toState.isCollided()) {
			return -10;
		} else if (action.equals(BumperAction.FORWARD)) {
			return 1;
		} else if (action.equals(BumperAction.BACKWARD)) {
			return -5;
		} else {
			return -1;
		}
	}
}
