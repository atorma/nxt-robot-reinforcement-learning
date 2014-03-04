package org.atorma.robot.simplebumper;

import org.atorma.robot.mdp.RewardFunction;
import org.atorma.robot.mdp.Transition;

public class BumperRewardFunction implements RewardFunction<BumperPercept, BumperAction> {

	@Override
	public double getReward(Transition<BumperPercept, BumperAction> transition) {
		BumperPercept toState = (BumperPercept) transition.getToState();
		BumperAction action = (BumperAction) transition.getAction();
		
		if (toState.isCollided()) {
			return -100;
		} else if (action.equals(BumperAction.FORWARD)) {
			return 1;
		} else if (action.equals(BumperAction.BACKWARD)) {
			return -5;
		} else {
			return -1;
		}
	}
}
