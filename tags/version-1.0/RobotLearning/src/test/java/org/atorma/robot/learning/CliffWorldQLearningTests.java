package org.atorma.robot.learning;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;

import org.atorma.robot.discretization.StateDiscretizer;
import org.atorma.robot.learning.cliffworld.*;
import org.atorma.robot.mdp.*;
import org.atorma.robot.policy.DiscretePolicy;
import org.atorma.robot.policy.EpsilonGreedyPolicy;
import org.junit.Before;
import org.junit.Test;

public class CliffWorldQLearningTests {

	private QLearning qLearning;
	private double learningRate = 0.2;
	private double discountFactor = 1;
	private double traceDecay = 0.8;
	private QTable qTable;
	
	private StateDiscretizer stateDiscretizer = new CliffWorldStateDiscretizer();
	private RewardFunction rewardFunction = new CliffWorldRewardFunction();
	private DiscretePolicy policy;
	private EligibilityTraces traces;
	
	@Before
	public void setUp() {
		qTable = new ArrayQTable(stateDiscretizer.getNumberOfStates(), CliffWorldAction.values().length, 0);
		policy = new EpsilonGreedyPolicy(0.1, qTable, CliffWorldAction.values());
	}
	
	// Both of the tests can sometimes fail, due to randomness
	
	@Test
	public void standard_q_learning_with_epsilon_greedy_exploration_learns_optimal_policy() {
		qLearning = new QLearning(learningRate, discountFactor, qTable);
		int totalIterations = learnPolicy(250);
		System.out.println("Cliff world standard Q-learning: " + totalIterations + " total iterations");
		List<CliffWorldAction> learnedPath = getLearnedPath();
		assertEquals(CliffWorldEnvironment.OPTIMAL_PATH, learnedPath);
	}
	
	@Test
	public void q_learning_with_traces_and_epsilon_greed_exploration_learns_good_policy() {
		traces = new ReplacingEligibilityTraces(discountFactor, traceDecay);
		qLearning = new QLearning(learningRate, traces, qTable);
		int totalIterations = learnPolicy(40);
		System.out.println("Cliff world Q-learning with eligibility traces: " + totalIterations + " total iterations");
		List<CliffWorldAction> learnedPath = getLearnedPath();
		System.out.println(learnedPath);
		assertTrue(learnedPath.size() <= 1.6 * CliffWorldEnvironment.OPTIMAL_PATH.size());
	}

	private List<CliffWorldAction> getLearnedPath() {
		CliffWorldState state = CliffWorldState.START;
		List<CliffWorldAction> learnedActions = new ArrayList<>();
		while (!state.isGoal() && learnedActions.size() <= 5*CliffWorldEnvironment.OPTIMAL_PATH.size()) {
			int stateId = stateDiscretizer.getId(state);
			int actionId = qLearning.getActionId(stateId);
			CliffWorldAction action = CliffWorldAction.getActionById(actionId);
			learnedActions.add(action);
			state = state.getNextState(action);
		}
		return learnedActions;
	}
	

	private int learnPolicy(int numEpisodes) {
		int totalIterations = 0;
		
		for (int episode=0; episode<numEpisodes; episode++) {
			
			CliffWorldState fromState = CliffWorldState.START;
			CliffWorldState toState;
			if (traces != null) {
				traces.clear();
			}
			
			do {
				int fromStateId = stateDiscretizer.getId(fromState);
				Integer byActionId = policy.getActionId(fromStateId);
				CliffWorldAction byAction = CliffWorldAction.getActionById(byActionId);
				toState = fromState.getNextState(byAction);
				int toStateId = stateDiscretizer.getId(toState);
				Transition transition = new Transition(fromState, byAction, toState);
				double reward = rewardFunction.getReward(transition);
				
				DiscretizedTransitionReward trReward = new DiscretizedTransitionReward(fromStateId, byActionId, toStateId, reward);
				qLearning.update(trReward);

				fromState = toState;
				
				totalIterations++;

			} while (!toState.isEnd());

		}
		
		return totalIterations;
	}

}
