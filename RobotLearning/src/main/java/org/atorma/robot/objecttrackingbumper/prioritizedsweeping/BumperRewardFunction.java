package org.atorma.robot.objecttrackingbumper.prioritizedsweeping;

import org.atorma.robot.mdp.RewardFunction;
import org.atorma.robot.mdp.Transition;
import org.atorma.robot.simplebumper.BumperAction;

public class BumperRewardFunction implements RewardFunction {

	@Override
	public double getReward(Transition transition) {
		ModeledBumperState toState = (ModeledBumperState) transition.getToState();
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
