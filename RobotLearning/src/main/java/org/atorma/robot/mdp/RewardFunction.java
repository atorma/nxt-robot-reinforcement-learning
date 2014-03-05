package org.atorma.robot.mdp;


public interface RewardFunction {

	double getReward(Transition transition);
}
