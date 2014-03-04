package org.atorma.robot.mdp;


public interface RewardFunction<S extends State, A extends DiscreteAction> {

	double getReward(Transition<S, A> transition);
}
