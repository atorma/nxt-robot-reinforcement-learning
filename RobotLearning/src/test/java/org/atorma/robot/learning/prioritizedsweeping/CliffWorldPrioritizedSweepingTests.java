package org.atorma.robot.learning.prioritizedsweeping;

import static org.junit.Assert.assertEquals;

import java.util.*;

import org.atorma.robot.learning.HashMapQTable;
import org.atorma.robot.learning.cliffworld.*;
import org.atorma.robot.learning.prioritizedsweeping.PrioritizedSweeping;
import org.atorma.robot.mdp.*;
import org.atorma.robot.policy.EpsilonGreedyPolicy;
import org.junit.Before;
import org.junit.Test;

public class CliffWorldPrioritizedSweepingTests {

	private PrioritizedSweeping sweeping;
	private double discountFactor = 1;
	
	private CliffWorldStateDiscretizer stateDiscretizer = new CliffWorldStateDiscretizer();
	
	private LearningCliffWorldModel model;

	@Before
	public void setUp() {
		model = new LearningCliffWorldModel();
		
		HashMapQTable qTable = new HashMapQTable(0, CliffWorldAction.values());
		
		sweeping = new PrioritizedSweeping();
		sweeping.setDiscountFactor(discountFactor);
		sweeping.setStateDiscretizer(stateDiscretizer);
		sweeping.setModel(model);
		sweeping.setQTable(qTable);
	}
	
	@Test
	public void with_optimal_path_and_big_reward_in_the_goal_default_q_values_prioritized_sweeping_learns_model_in_one_episode() {
		
		CliffWorldRewardFunction rewardFunction = new ModifiedCliffWorldRewardFunction();
		
		int numIter = CliffWorldEnvironment.OPTIMAL_PATH.size();

		CliffWorldState state = CliffWorldState.START;
		for (int i = 0; i < CliffWorldEnvironment.OPTIMAL_PATH.size(); i++) {
			
			CliffWorldAction action = CliffWorldEnvironment.OPTIMAL_PATH.get(i);
			CliffWorldState nextState = state.getNextState(action);
			Transition transition = new Transition(state, action, nextState);
			TransitionReward transitionReward = new TransitionReward(transition, rewardFunction.getReward(transition));
			model.update(transitionReward);
			
			sweeping.setSweepStartStateAction(new StateAction(state, action));
			sweeping.performIterations(numIter);

			state = nextState;
		}
		
		List<CliffWorldAction> learnedPath = getLearnedPath();
		assertEquals(CliffWorldEnvironment.OPTIMAL_PATH, learnedPath);
	}
	
	@Test
	public void learns_optimal_path_in_few_episodes() {
		
		CliffWorldRewardFunction rewardFunction = new CliffWorldRewardFunction();
		EpsilonGreedyPolicy policy = new EpsilonGreedyPolicy(0.1, sweeping, CliffWorldAction.values());
		
		for (int episode = 0; episode < 20; episode++) { // Q-learning takes about 500 episodes to learn the optimal path with high probability 
			
			CliffWorldState fromState = CliffWorldState.START;
			CliffWorldState toState;
			
			do {
				int fromStateId = stateDiscretizer.getId(fromState);
				Integer byActionId = policy.getActionId(fromStateId);
				CliffWorldAction byAction = CliffWorldAction.getActionById(byActionId);
				toState = fromState.getNextState(byAction);
				Transition transition = new Transition(fromState, byAction, toState);
				double reward = rewardFunction.getReward(transition);
				
				model.update(new TransitionReward(transition, reward));
				sweeping.setSweepStartStateAction(transition.getFromStateAction());
				sweeping.performIterations(CliffWorldEnvironment.OPTIMAL_PATH.size()); // ... though of course we're a while in the sweeps now
				
				fromState = toState;

			} while (!toState.isEnd());

		}
		
		List<CliffWorldAction> learnedPath = getLearnedPath();
		assertEquals(CliffWorldEnvironment.OPTIMAL_PATH, learnedPath);
	}
	
	private List<CliffWorldAction> getLearnedPath() {
		CliffWorldState state = CliffWorldState.START;
		List<CliffWorldAction> learnedActions = new ArrayList<>();
		while (!state.isEnd() && learnedActions.size() <= 2*CliffWorldEnvironment.OPTIMAL_PATH.size()) {
			int stateId = stateDiscretizer.getId(state);
			int actionId = sweeping.getActionId(stateId);
			CliffWorldAction action = CliffWorldAction.getActionById(actionId);
			learnedActions.add(action);
			state = state.getNextState(action);
		}
		return learnedActions;
	}
}
