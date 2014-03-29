package org.atorma.robot.learning.montecarlo;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;

import org.atorma.robot.learning.*;
import org.atorma.robot.learning.cliffworld.*;
import org.atorma.robot.mdp.DiscretizedTransitionReward;
import org.atorma.robot.mdp.Transition;
import org.junit.*;

public class CliffWorldUctPlanningWithQLearning {
	
	private FirstVisitUctPlanning uctPlanning;
	private double discountFactor = 1;
	
	private QLearning qLearning;
	private double learningRate = 0.1;
	
	private int planningHorizon = 50;
	private CliffWorldStateDiscretizer stateDiscretizer = new CliffWorldStateDiscretizer();
	private QTable qTable;
	private ExactCliffWorldForwardModel model;
	
	private ScaledCliffWorldRewardFunction rewardFunction = new ScaledCliffWorldRewardFunction();
	

	@Before
	public void setUp() {
		model = new ExactCliffWorldForwardModel(rewardFunction);
		
		qTable = new ArrayQTable(stateDiscretizer.getNumberOfStates(), CliffWorldAction.values().length, 1);
		qLearning = new QLearning(learningRate, discountFactor, qTable);
		
		UctPlanningParameters uctParams = new UctPlanningParameters();
		uctParams.discountFactor = discountFactor;
		uctParams.horizon = planningHorizon;
		uctParams.model = model;
		uctParams.qTable = qTable;
		uctParams.stateDiscretizer = stateDiscretizer;
		uctParams.uctConstant = 1.0;
		uctPlanning = new FirstVisitUctPlanning(uctParams);
	}
	
	// Problem with reward setting: with ScaledCliffWorldRewardFunction the agent gets positive reward for moving anywhere (except cliff).
	// The total return for wandering around is thus bigger than just heading straight for the goal in one step!
	@Test @Ignore 
	public void next_to_goal_planned_action_is_to_go_goal() {
		
		CliffWorldState state = new CliffWorldState(11, 1);
		assertTrue(state.getNextState(CliffWorldAction.DOWN).isGoal());
		
		uctPlanning.setRolloutStartState(state);
		uctPlanning.performRollouts(10); 
		
		int stateId = stateDiscretizer.getId(state);
		Integer plannedActionId = uctPlanning.getActionId(stateId);
		assertEquals(CliffWorldAction.DOWN.getId(), plannedActionId.intValue());
	}

	@Test @Ignore // same problem as with first visit monte carlo
	public void learns_optimal_path() {

		for (int episode = 0; episode < 50; episode++) { 
			
			CliffWorldState fromState = CliffWorldState.START;
			CliffWorldState toState;
			
			do {
				if (fromState.getX() == 11) {
					System.out.println("Almost there!");
				}
				System.out.println(fromState);
				uctPlanning.setRolloutStartState(fromState);
				uctPlanning.performRollouts(1000); 
				
				int fromStateId = stateDiscretizer.getId(fromState);
				Integer byActionId = uctPlanning.getActionId(fromStateId);
				CliffWorldAction byAction = CliffWorldAction.getActionById(byActionId);
				toState = fromState.getNextState(byAction);
				int toStateId = stateDiscretizer.getId(toState);
				
				Transition transition = new Transition(fromState, byAction, toState);
				double reward = rewardFunction.getReward(transition);
				qLearning.update(new DiscretizedTransitionReward(fromStateId, byActionId, toStateId, reward));

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
	
}
