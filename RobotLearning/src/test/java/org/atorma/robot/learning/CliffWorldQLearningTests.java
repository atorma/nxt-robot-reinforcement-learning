package org.atorma.robot.learning;

import static org.junit.Assert.assertEquals;

import org.atorma.robot.learning.cliffworld.*;
import org.atorma.robot.mdp.DiscretizedTransitionWithReward;
import org.atorma.robot.mdp.Transition;
import org.atorma.robot.policy.EpsilonGreedyPolicy;
import org.junit.Before;
import org.junit.Test;

public class CliffWorldQLearningTests {

	private QLearning qLearning;
	private CliffWorldStateDiscretizer stateDiscretizer = new CliffWorldStateDiscretizer();
	private CliffWorldRewardFunction rewardFunction = new CliffWorldRewardFunction();
	
	@Before
	public void setUp() {
		qLearning = new QLearning(0.1, 1);
	}
	
	@Test
	public void cliff_world_q_learning_with_epsilon_greedy_policy_finds_optimal_policy() {
		learnPolicy();

		CliffWorldState state = CliffWorldState.START;
		CliffWorldAction action;
		
		action = CliffWorldAction.UP;
		assertExpectedActionEqualsLearnedAction(action, state);
		
		for (int i=0; i<11; i++) {
			state = state.getNextState(action);
			action = CliffWorldAction.RIGHT;
			assertExpectedActionEqualsLearnedAction(action, state);
		} 
		
		state = state.getNextState(action);
		action = CliffWorldAction.DOWN;
		assertExpectedActionEqualsLearnedAction(action, state);
		
	}
	
	private void assertExpectedActionEqualsLearnedAction(CliffWorldAction expectedAction, CliffWorldState currentState) {
		int expectedActionId = expectedAction.getId();
		int currentStateId = stateDiscretizer.getId(currentState.getValues());
		int learnedActionId = qLearning.getActionId(currentStateId);
		assertEquals(expectedActionId, learnedActionId);
	}

	private void learnPolicy() {
		EpsilonGreedyPolicy policy = new EpsilonGreedyPolicy(0.1, qLearning,
				CliffWorldAction.UP, CliffWorldAction.DOWN, CliffWorldAction.LEFT, CliffWorldAction.RIGHT);
		
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
				Transition<CliffWorldState, CliffWorldAction> transition = new Transition<>(fromState, byAction, toState);
				double reward = rewardFunction.getReward(transition);
				
				qLearning.update(new DiscretizedTransitionWithReward(fromStateId, byActionId, toStateId, reward));
				fromState = toState;

			} while (!toState.isGoal());

		}
	}

}
