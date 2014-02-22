package org.atorma.robot.simplebumper;

import org.atorma.robot.RewardFunction;
import org.atorma.robot.learning.Transition;

public class BumperRewardFunction implements RewardFunction {

	@Override
	public double getReward(Transition transition) {
		BumperState toState = (BumperState) transition.getToState();
		BumperAction action = (BumperAction) transition.getAction();
		
		if (toState.getDistanceToObstacle() < BumperState.MIN_ULTRASONIC_DIST + 5) {
			return -100;
		} else if (action.equals(BumperAction.FORWARD)) {
			return +1;
		} else if (action.equals(BumperAction.LEFT) || action.equals(BumperAction.RIGHT)){
			return -5;
		} else {
			return -10;
		}
	}
}
