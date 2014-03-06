package org.atorma.robot.learning;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.List;

import org.atorma.robot.learning.cliffworld.*;
import org.atorma.robot.mdp.DiscretizedTransitionReward;
import org.atorma.robot.mdp.Transition;
import org.atorma.robot.policy.EpsilonGreedyPolicy;
import org.junit.Before;
import org.junit.Test;

public class CliffWorldQLearningTests {

	private QLearning qLearning;
	private double learningRate = 0.1;
	private double discountFactor = 1;
	
	private CliffWorldStateDiscretizer stateDiscretizer = new CliffWorldStateDiscretizer();
	private CliffWorldRewardFunction rewardFunction = new CliffWorldRewardFunction();
	private EpsilonGreedyPolicy policy;
	
	@Before
	public void setUp() {
		qLearning = new QLearning(learningRate, discountFactor);
		policy = new EpsilonGreedyPolicy(0.1, qLearning, CliffWorldAction.UP, CliffWorldAction.DOWN, CliffWorldAction.LEFT, CliffWorldAction.RIGHT);
	}
	
	@Test
	public void cliff_world_q_learning_with_epsilon_greedy_policy_finds_optimal_policy() {
		learnPolicy();
		List<CliffWorldAction> learnedPath = getLearnedPath();
		assertEquals(CliffWorldEnvironment.OPTIMAL_PATH, learnedPath);
	}

	private List<CliffWorldAction> getLearnedPath() {
		CliffWorldState state = CliffWorldState.START;
		List<CliffWorldAction> learnedActions = new ArrayList<>();
		while (!state.isGoal() && learnedActions.size() <= CliffWorldEnvironment.OPTIMAL_PATH.size()) {
			int stateId = stateDiscretizer.getId(state.getValues());
			int actionId = qLearning.getActionId(stateId);
			CliffWorldAction action = CliffWorldAction.getActionById(actionId);
			learnedActions.add(action);
			state = state.getNextState(action);
		}
		return learnedActions;
	}
	

	private void learnPolicy() {
		int numEpisodes = 500;
		
		for (int episode=0; episode<numEpisodes; episode++) {
			
			CliffWorldState fromState = CliffWorldState.START;
			CliffWorldState toState;
			
			do {
				int fromStateId = stateDiscretizer.getId(fromState.getValues());
				Integer byActionId = policy.getActionId(fromStateId);
				CliffWorldAction byAction = CliffWorldAction.getActionById(byActionId);
				toState = fromState.getNextState(byAction);
				int toStateId = stateDiscretizer.getId(toState.getValues());
				Transition transition = new Transition(fromState, byAction, toState);
				double reward = rewardFunction.getReward(transition);
				
				qLearning.update(new DiscretizedTransitionReward(fromStateId, byActionId, toStateId, reward));
				fromState = toState;

			} while (!toState.isGoal());

		}
	}

}
