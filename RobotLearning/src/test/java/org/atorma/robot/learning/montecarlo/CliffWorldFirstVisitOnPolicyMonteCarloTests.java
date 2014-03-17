package org.atorma.robot.learning.montecarlo;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.List;

import org.atorma.robot.learning.*;
import org.atorma.robot.learning.cliffworld.*;
import org.atorma.robot.mdp.*;
import org.atorma.robot.policy.DiscretePolicy;
import org.atorma.robot.policy.EpsilonGreedyPolicy;
import org.junit.Before;
import org.junit.Test;

public class CliffWorldFirstVisitOnPolicyMonteCarloTests {
	
	private FirstVisitOnPolicyMonteCarlo monteCarlo;
	private double discountFactor = 1;
	private CliffWorldStateDiscretizer stateDiscretizer = new CliffWorldStateDiscretizer();
	private QTable qTable;
	private LearningCliffWorldModel model;
	private DiscretePolicy policy;
	private CliffWorldRewardFunction rewardFunction = new CliffWorldRewardFunction();

	@Before
	public void setUp() {
		model = new LearningCliffWorldModel();
		qTable = new ArrayQTable(stateDiscretizer.getNumberOfStates(), CliffWorldAction.values().length);
		policy = new EpsilonGreedyPolicy(0.1, qTable, CliffWorldAction.values());
		monteCarlo = new FirstVisitOnPolicyMonteCarlo(model, stateDiscretizer, policy, qTable, 10, discountFactor);
	}
	
	@Test
	public void learns_optimal_path_in_few_episodes() {

		for (int episode = 0; episode < 10; episode++) { // Q-learning takes about 500 episodes to learn the optimal path with high probability 
			
			CliffWorldState fromState = CliffWorldState.START;
			CliffWorldState toState;
			
			do {
				monteCarlo.setRolloutStartState(fromState);
				monteCarlo.performRollouts(100); // ... though of course we're a while in the sweeps now
				
				int fromStateId = stateDiscretizer.getId(fromState);
				Integer byActionId = policy.getActionId(fromStateId);
				CliffWorldAction byAction = CliffWorldAction.getActionById(byActionId);
				toState = fromState.getNextState(byAction);
				Transition transition = new Transition(fromState, byAction, toState);
				double reward = rewardFunction.getReward(transition);
				
				model.updateModel(new TransitionReward(transition, reward));

				fromState = toState;

			} while (!toState.isGoal());

		}
		
		List<CliffWorldAction> learnedPath = getLearnedPath();
		assertEquals(CliffWorldEnvironment.OPTIMAL_PATH, learnedPath);
	}
	
	private List<CliffWorldAction> getLearnedPath() {
		CliffWorldState state = CliffWorldState.START;
		List<CliffWorldAction> learnedActions = new ArrayList<>();
		while (!state.isGoal() && learnedActions.size() <= 2*CliffWorldEnvironment.OPTIMAL_PATH.size()) {
			int stateId = stateDiscretizer.getId(state);
			int actionId = qTable.getActionId(stateId);
			CliffWorldAction action = CliffWorldAction.getActionById(actionId);
			learnedActions.add(action);
			state = state.getNextState(action);
		}
		return learnedActions;
	}
	
	
	// Modified reward function to give reward upon reaching the goal. 
	// Makes it easier to test this case of prioritized sweeping where the model
	// does not know the transitions nor the reward function.
	private static class ModifiedCliffWorldRewardFunction extends CliffWorldRewardFunction {

		@Override
		public double getReward(Transition transition) {
			CliffWorldState toState = (CliffWorldState) transition.getToState();
			if (toState.isGoal()) {
				return 100.0;
			} else {
				return super.getReward(transition);
			}
		}
		
	}
}
