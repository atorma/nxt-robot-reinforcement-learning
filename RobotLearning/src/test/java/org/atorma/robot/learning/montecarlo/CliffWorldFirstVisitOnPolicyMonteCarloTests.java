package org.atorma.robot.learning.montecarlo;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.List;

import org.atorma.robot.learning.*;
import org.atorma.robot.learning.cliffworld.*;
import org.atorma.robot.mdp.*;
import org.atorma.robot.policy.*;
import org.junit.*;

public class CliffWorldFirstVisitOnPolicyMonteCarloTests {
	
	private FirstVisitOnPolicyMonteCarlo monteCarlo;
	private double discountFactor = 1;
	private int planningHorizon = 20;
	private CliffWorldStateDiscretizer stateDiscretizer = new CliffWorldStateDiscretizer();
	private QTable qTable;
	private ExactCliffWorldForwardModel model;
	private DiscretePolicy policy;
	

	@Before
	public void setUp() {
		model = new ExactCliffWorldForwardModel(new ScaledCliffWorldRewardFunction());
		qTable = new ArrayQTable(stateDiscretizer.getNumberOfStates(), CliffWorldAction.values().length);
		
		DirectedExploration directedExploration = new DirectedExploration(qTable, 0.02, 0.1, CliffWorldAction.values());
		EpsilonGreedyPolicy epsilonGreedyPolicy = new EpsilonGreedyPolicy(0.1, directedExploration, CliffWorldAction.values());
		policy = new DirectedExplorationPolicy(epsilonGreedyPolicy, directedExploration);
		
		monteCarlo = new FirstVisitOnPolicyMonteCarlo(model, stateDiscretizer, policy, qTable, planningHorizon, discountFactor);
	}
	
	// FirstVisitOnPolicyMonteCarlo sucks at learning the long-term Q-values
	// It will consider it better to jump down the cliff right away than to walk ahead
	// because it will think it'll fall down the cliff later anyway!
	@Test @Ignore
	public void learns_optimal_path() {

		for (int episode = 0; episode < 50; episode++) { // Q-learning takes about 500 episodes to learn the optimal path with high probability 
			
			CliffWorldState fromState = CliffWorldState.START;
			CliffWorldState toState;
			
			do {
				monteCarlo.setRolloutStartState(fromState);
				monteCarlo.performRollouts(50); // ... though of course we're a while in the sweeps now
				
				int fromStateId = stateDiscretizer.getId(fromState);
				Integer byActionId = policy.getActionId(fromStateId);
				CliffWorldAction byAction = CliffWorldAction.getActionById(byActionId);
				toState = fromState.getNextState(byAction);

				fromState = toState;

			} while (!toState.isEnd());

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
	

	private static class DirectedExplorationPolicy implements DiscretePolicy {
		
		private DiscretePolicy policy;
		private DirectedExploration directedExploration;

		private DirectedExplorationPolicy(DiscretePolicy policy, DirectedExploration directedExploration) {
			this.policy = policy;
			this.directedExploration = directedExploration;
		}

		@Override
		public Integer getActionId(int stateId) {
			Integer actionId = policy.getActionId(stateId);
			directedExploration.recordStateAction(new DiscretizedStateAction(stateId, actionId));
			return actionId;
		}
		
	}
}
