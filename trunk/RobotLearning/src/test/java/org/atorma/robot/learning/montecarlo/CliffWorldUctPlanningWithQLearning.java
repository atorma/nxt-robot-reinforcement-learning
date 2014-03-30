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
	private double learningRate = 1;
	
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
		uctParams.horizon = planningHorizon;
		uctParams.model = model;
		uctParams.qTable = qTable;
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
		
		int stateId = stateDiscretizer.getId(state);
		System.out.println("UCT Q-value for DOWN: " + uctPlanning.getUctQValue(stateId, CliffWorldAction.DOWN.getId()));
		Integer plannedActionId = uctPlanning.getActionId(stateId);
		System.out.println("Planned action " + CliffWorldAction.getActionById(plannedActionId) + " UCT Q-value: " + uctPlanning.getUctQValue(stateId, plannedActionId) );
		System.out.println("UCT Q-value for DOWN: " + uctPlanning.getUctQValue(stateId, CliffWorldAction.DOWN.getId()));
		assertEquals(CliffWorldAction.DOWN.getId(), plannedActionId.intValue());
		
	}

	@Test 
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
				uctPlanning.performRollouts(50); 
				
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
		while (!state.isEnd() && learnedActions.size() <= 2*CliffWorldEnvironment.OPTIMAL_PATH.size()) {
			int stateId = stateDiscretizer.getId(state);
			int actionId = qTable.getActionId(stateId);
			CliffWorldAction action = CliffWorldAction.getActionById(actionId);
			learnedActions.add(action);
			state = state.getNextState(action);
		}
		return learnedActions;
	}
	
}