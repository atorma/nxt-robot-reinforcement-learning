package org.atorma.robot.learning.montecarlo;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;

import org.atorma.robot.learning.*;
import org.atorma.robot.learning.cliffworld.*;
import org.atorma.robot.mdp.*;
import org.junit.*;

public class CliffWorldUctPlanningWithQLearning {
	
	private FirstVisitUctPlanning uctPlanning;
	private double discountFactor = 1;
	
	private QLearning qLearning;
	private double learningRate = 0.2;
	
	private int planningHorizon = 25;
	private CliffWorldStateDiscretizer stateDiscretizer = new CliffWorldStateDiscretizer();
	private QTable qTable;
	private ExactCliffWorldForwardModel model;
	
	//private ScaledCliffWorldRewardFunction rewardFunction = new ScaledCliffWorldRewardFunction();
	private CliffWorldRewardFunction rewardFunction = new CliffWorldRewardFunction();
	

	@Before
	public void setUp() {
		model = new ExactCliffWorldForwardModel(rewardFunction);
		
		qTable = new ArrayQTable(stateDiscretizer.getNumberOfStates(), CliffWorldAction.values().length, 1);
		qLearning = new QLearning(learningRate, discountFactor, qTable);
		
		UctPlanningParameters uctParams = new UctPlanningParameters();
		uctParams.discountFactor = discountFactor;
		uctParams.planningHorizon = planningHorizon;
		uctParams.model = model;
		uctParams.longTermQValues = qTable;
		uctParams.stateDiscretizer = stateDiscretizer;
		uctParams.uctConstant = 15.0;
		uctPlanning = new FirstVisitUctPlanning(uctParams);
	}
	
	@Test  
	public void next_to_goal_planned_action_is_to_go_goal() {
		
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
		uctPlanning.performRollouts(100); 
		
		DiscreteAction plannedAction = uctPlanning.getPlannedAction(state);
		assertTrue(CliffWorldAction.RIGHT != plannedAction);
	}

	// Slow in learning because of the reward setting. The agent gets negative rewards when
	// moving and a big negative reward if falling of the cliff. In action 
	// planning falling off the cliff is much more likely than ending up in the goal state,
	// so the planner finds it best to jump off the cliff to minimize costs rather than
	// wander around and then fall off!
	@Test @Ignore
	public void learns_optimal_path() {

		for (int episode = 0; episode < 50; episode++) { 
			
			CliffWorldState fromState = CliffWorldState.START;
			CliffWorldState toState;
			
			do {
				System.out.println(fromState);
				uctPlanning.setRolloutStartState(fromState);
				uctPlanning.performRollouts(50); 
				
				CliffWorldAction byAction = (CliffWorldAction) uctPlanning.getPlannedAction(fromState);
				TransitionReward transition = model.simulateAction(new StateAction(fromState, byAction)); // shortcut, this is a deterministic model!
				toState = (CliffWorldState) transition.getToState();
				
				int fromStateId = stateDiscretizer.getId(fromState);
				int byActionId = byAction.getId();
				int toStateId = stateDiscretizer.getId(transition.getToState());
				qLearning.update(new DiscretizedTransitionReward(fromStateId, byActionId, toStateId, transition.getReward()));

				fromState = toState;

			} while (!toState.isEnd());

		}
		
		List<CliffWorldAction> learnedPath = getLearnedPath();
		assertEquals(CliffWorldEnvironment.OPTIMAL_PATH, learnedPath);
	}
	
	private List<CliffWorldAction> getLearnedPath() {
		CliffWorldState state = CliffWorldState.START;
		List<CliffWorldAction> learnedActions = new ArrayList<>();
		while (!state.isEnd() && learnedActions.size() <= 10*CliffWorldEnvironment.OPTIMAL_PATH.size()) {
			int stateId = stateDiscretizer.getId(state);
			int actionId = qLearning.getActionId(stateId);
			CliffWorldAction action = CliffWorldAction.getActionById(actionId);
			learnedActions.add(action);
			state = state.getNextState(action);
		}
		return learnedActions;
	}
	
}
