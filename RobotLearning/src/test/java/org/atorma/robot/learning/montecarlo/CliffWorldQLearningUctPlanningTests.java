package org.atorma.robot.learning.montecarlo;

import static org.junit.Assert.*;

import java.util.*;

import org.atorma.robot.learning.*;
import org.atorma.robot.learning.cliffworld.*;
import org.atorma.robot.mdp.*;
import org.junit.*;

public class CliffWorldQLearningUctPlanningTests {
	
	private QLearning qLearning;
	private double discountFactor = 1;
	private double learningRate = 0.4;
	private EligibilityTraces longTermTraces;
	private QTable qTable;
	
	private QLearningUctPlanning uctPlanning;
	private int planningHorizon = 25;
	
	private ExactCliffWorldForwardModel model;
	private CliffWorldStateDiscretizer stateDiscretizer = new CliffWorldStateDiscretizer();
	
	// Monte-Carlo planning methods seem to require a function that gives a substantial reward on 
	// reaching the goal. Otherwise they will plan it safe avoid going anywhere near the cliff,
	// which means they'll never find the proper path.
	private RewardFunction rewardFunction = new ModifiedCliffWorldRewardFunction(); 
	

	@Before
	public void setUp() {
		model = new ExactCliffWorldForwardModel(rewardFunction);
		
		qTable = new ArrayQTable(stateDiscretizer.getNumberOfStates(), CliffWorldAction.values().length, 0);
		longTermTraces = new ReplacingEligibilityTraces(discountFactor, 0.8);
		qLearning = new QLearning(learningRate, longTermTraces, qTable);
		
		QLearningUctPlanningParameters uctParams = new QLearningUctPlanningParameters();
		uctParams.model = model;
		uctParams.allActions = CliffWorldAction.values();
		uctParams.stateDiscretizer = stateDiscretizer;
		uctParams.longTermQValues = qTable; // this really does speed up learning
		uctParams.learningRate = learningRate;
		uctParams.eligibilityTraces = new ReplacingEligibilityTraces(discountFactor, 0.8);
		uctParams.planningHorizon = planningHorizon;
		uctParams.uctConstant = 3.0;
		uctPlanning = new QLearningUctPlanning(uctParams);
	}
	
	@Test  
	public void next_to_goal_planned_action_is_to_go_to_goal() {
		
		CliffWorldState state = new CliffWorldState(11, 1);
		assertTrue(state.getNextState(CliffWorldAction.DOWN).isGoal());
		
		uctPlanning.setRolloutStartState(state);
		uctPlanning.performRollouts(4); 
		
		DiscreteAction plannedAction = uctPlanning.getPlannedAction(state);
		assertEquals(CliffWorldAction.DOWN, plannedAction);
		
	}
	
	@Test  
	public void next_to_cliff_planned_action_is_not_to_fall_off_the_cliff() {
		
		CliffWorldState state = CliffWorldState.START;
		assertTrue(state.getNextState(CliffWorldAction.RIGHT).isCliff());
		
		uctPlanning.setRolloutStartState(state);
		uctPlanning.performRollouts(20); 
		
		DiscreteAction plannedAction = uctPlanning.getPlannedAction(state);
		assertTrue(CliffWorldAction.RIGHT != plannedAction);
	}

	// Typically learns a path [UP, UP, RIGHT, UP, RIGHT, RIGHT, RIGHT, RIGHT, RIGHT, RIGHT, RIGHT, RIGHT, RIGHT, RIGHT, DOWN, DOWN, DOWN].
	// This is the consequence of "naive" Q(lambda). It does not find the optimal path but rather a safer one (a bit like Sarsa).
	@Test 
	public void learns_near_optimal_path() {

		final int numRollOuts = 20;
		int totalIterations = 0; 
		
		for (int episode = 0; episode < 5; episode++) { 
			
			CliffWorldState fromState = CliffWorldState.START;
			CliffWorldState toState;
			longTermTraces.clear();
			
			do {
				System.out.println(fromState);
				uctPlanning.setRolloutStartState(fromState);
				uctPlanning.performRollouts(numRollOuts); 
				
				totalIterations += numRollOuts*planningHorizon; // not exact because rollout can end in goal sooner than planning horizon
				
				CliffWorldAction byAction = (CliffWorldAction) uctPlanning.getPlannedAction(fromState);
				TransitionReward transition = model.simulateAction(new StateAction(fromState, byAction)); // shortcut, this is a deterministic model!
				toState = (CliffWorldState) transition.getToState();
				
				int fromStateId = stateDiscretizer.getId(fromState);
				int byActionId = byAction.getId();
				int toStateId = stateDiscretizer.getId(transition.getToState());
				qLearning.update(new DiscretizedTransitionReward(fromStateId, byActionId, toStateId, transition.getReward()));

				fromState = toState;
				
				totalIterations++;

			} while (!toState.isEnd());

		}
		
		System.out.println("Iterations: " + totalIterations);
		
		List<CliffWorldAction> learnedPath = getLearnedPath();
		System.out.println(learnedPath);
		assertTrue(learnedPath.size() < 1.5 * CliffWorldEnvironment.OPTIMAL_PATH.size());
	}
	
	private List<CliffWorldAction> getLearnedPath() {
		CliffWorldState state = CliffWorldState.START;
		List<CliffWorldAction> learnedActions = new ArrayList<>();
		while (!state.isGoal() && learnedActions.size() <= 10*CliffWorldEnvironment.OPTIMAL_PATH.size()) {
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
	
}
