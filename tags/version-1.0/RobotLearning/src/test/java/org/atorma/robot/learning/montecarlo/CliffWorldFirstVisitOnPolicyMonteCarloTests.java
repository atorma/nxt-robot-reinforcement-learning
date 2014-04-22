package org.atorma.robot.learning.montecarlo;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.atorma.robot.learning.*;
import org.atorma.robot.learning.cliffworld.*;
import org.atorma.robot.mdp.*;
import org.atorma.robot.policy.*;
import org.junit.*;

public class CliffWorldFirstVisitOnPolicyMonteCarloTests {
	
	private QLearning qLearning;
	private double learningRate = 0.25;
	private double discountFactor = 0.9;
	private QTable qTable;
	
	private FirstVisitOnPolicyMonteCarlo monteCarlo;
	private int planningHorizon = 25;
	
	private CliffWorldStateDiscretizer stateDiscretizer = new CliffWorldStateDiscretizer();
	private ExactCliffWorldForwardModel model;
	private DiscretePolicy explorationPolicy;
	
	private RewardFunction rewardFunction = new ModifiedCliffWorldRewardFunction();
	

	@Before
	public void setUp() {
		model = new ExactCliffWorldForwardModel(rewardFunction);
		qTable = new ArrayQTable(stateDiscretizer.getNumberOfStates(), CliffWorldAction.values().length);
		qLearning = new QLearning(learningRate, discountFactor, qTable);
		
		//DirectedExploration directedExploration = new DirectedExploration(qTable, 0.02, 0.1, CliffWorldAction.values());
		//EpsilonGreedyPolicy epsilonGreedyPolicy = new EpsilonGreedyPolicy(0.1, directedExploration, CliffWorldAction.values());
		//policy = new DirectedExplorationPolicy(epsilonGreedyPolicy, directedExploration);
		explorationPolicy = new EpsilonGreedyPolicy(0.1, qLearning, CliffWorldAction.values());
		
		monteCarlo = new FirstVisitOnPolicyMonteCarlo(model, stateDiscretizer, explorationPolicy, planningHorizon, discountFactor);
	}
	
	@Test  
	public void next_to_goal_planned_action_is_to_go_goal() {
		
		CliffWorldState state = new CliffWorldState(11, 1);
		assertTrue(state.getNextState(CliffWorldAction.DOWN).isGoal());
		
		monteCarlo.setRolloutStartState(state);
		monteCarlo.performRollouts(10); 
		
		int stateId = stateDiscretizer.getId(state);
		DiscreteAction plannedAction = CliffWorldAction.getActionById(monteCarlo.getActionId(stateId));
		assertEquals(CliffWorldAction.DOWN, plannedAction);
	}
	
	@Test  
	public void next_to_cliff_planned_action_is_not_to_fall_off_the_cliff() {
		
		CliffWorldState state = CliffWorldState.START;
		assertTrue(state.getNextState(CliffWorldAction.RIGHT).isCliff());
		
		monteCarlo.setRolloutStartState(state);
		monteCarlo.performRollouts(100); 
		
		int stateId = stateDiscretizer.getId(state);
		DiscreteAction plannedAction = CliffWorldAction.getActionById(monteCarlo.getActionId(stateId));
		assertTrue(CliffWorldAction.RIGHT != plannedAction);
	}
	
	// Requires a function that rewards highly on getting to goal. 
	// Otherwise action planning is simply too pessimistic to ever take a step towards the goal (except by accident).
	@Test @Ignore // Fairly slow, but works
	public void learns_good_path() {

		for (int episode = 0; episode < 50; episode++) { 
			
			CliffWorldState fromState = CliffWorldState.START;
			CliffWorldState toState;
			
			do {
				monteCarlo.setRolloutStartState(fromState);
				monteCarlo.performRollouts(30); 
				
				int fromStateId = stateDiscretizer.getId(fromState);
				Integer byActionId = monteCarlo.getActionId(fromStateId);
				CliffWorldAction byAction = CliffWorldAction.getActionById(byActionId);
				TransitionReward transition = model.simulateAction(new StateAction(fromState, byAction)); // shortcut, this is a deterministic model!
				toState = (CliffWorldState) transition.getToState();

				int toStateId = stateDiscretizer.getId(toState);
				qLearning.update(new DiscretizedTransitionReward(fromStateId, byActionId, toStateId, transition.getReward()));
				System.out.println(transition);

				fromState = toState;

			} while (!toState.isEnd());

		}
		
		List<CliffWorldAction> learnedPath = getLearnedPath();
		System.out.println(learnedPath);
		assertTrue(learnedPath.size() < 1.5 * CliffWorldEnvironment.OPTIMAL_PATH.size());
	}
	
	private List<CliffWorldAction> getLearnedPath() {
		CliffWorldState state = CliffWorldState.START;
		List<CliffWorldAction> learnedActions = new ArrayList<>();
		while (!state.isGoal() && learnedActions.size() <= 2*CliffWorldEnvironment.OPTIMAL_PATH.size()) {
			int stateId = stateDiscretizer.getId(state);
			int actionId = qLearning.getActionId(stateId);
			CliffWorldAction action = CliffWorldAction.getActionById(actionId);
			learnedActions.add(action);
			state = state.getNextState(action);
			
			if (state.isCliff()) {
				throw new IllegalStateException("Learns to throw itself off the cliff"); 
			}
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
