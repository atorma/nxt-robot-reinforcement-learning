package org.atorma.robot.objecttrackingbumper;

import org.atorma.robot.mdp.RewardFunction;
import org.atorma.robot.mdp.Transition;
import org.atorma.robot.simplebumper.BumperAction;

public class BumperRewardFunction implements RewardFunction<ModeledBumperState, BumperAction> {

	@Override
	public double getReward(Transition<ModeledBumperState, BumperAction> transition) {
		ModeledBumperState toState = transition.getToState();
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
