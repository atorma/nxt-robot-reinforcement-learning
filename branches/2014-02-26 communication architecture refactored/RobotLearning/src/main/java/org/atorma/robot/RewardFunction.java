package org.atorma.robot;

import org.atorma.robot.learning.Transition;

public interface RewardFunction {

	double getReward(Transition transition);
}
