package org.atorma.robot.learning.cliffworld;

import java.util.Collections;
import java.util.Set;

import org.atorma.robot.learning.montecarlo.ForwardModel;
import org.atorma.robot.mdp.*;

import com.google.common.collect.Sets;

public class ExactCliffWorldForwardModel implements ForwardModel {
	
	private CliffWorldRewardFunction rewardFunction = new CliffWorldRewardFunction();

	@Override
	public Set<? extends DiscreteAction> getAllowedActions(State state) {
		CliffWorldState cliffWorldState = (CliffWorldState) state;
		if (cliffWorldState.isEnd()) {
			return Collections.emptySet();
		} else {
			return Sets.newHashSet(CliffWorldAction.values());
		}
	}

	@Override
	public TransitionReward simulateAction(StateAction fromStateAction) {
		CliffWorldState state = (CliffWorldState) fromStateAction.getState();
		CliffWorldAction action = (CliffWorldAction) fromStateAction.getAction();
		CliffWorldState nextState = state.getNextState(action);
		Transition tr = new Transition(fromStateAction, nextState);
		double reward = rewardFunction.getReward(tr);
		return new TransitionReward(tr, reward);
	}

}
