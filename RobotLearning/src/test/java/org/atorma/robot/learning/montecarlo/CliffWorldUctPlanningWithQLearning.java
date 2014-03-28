package org.atorma.robot.learning.montecarlo;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.List;

import org.atorma.robot.learning.*;
import org.atorma.robot.learning.cliffworld.*;
import org.atorma.robot.mdp.DiscretizedTransitionReward;
import org.atorma.robot.mdp.Transition;
import org.junit.*;

public class CliffWorldUctPlanningWithQLearning {
	
	private UctPlanning uctPlanning;
	private double discountFactor = 1;
	
	private QLearning qLearning;
	private double learningRate = 0.1;
	
	private int planningHorizon = 20;
	private CliffWorldStateDiscretizer stateDiscretizer = new CliffWorldStateDiscretizer();
	private QTable qTable;
	private ExactCliffWorldForwardModel model;
	
	private CliffWorldRewardFunction rewardFunction = new CliffWorldRewardFunction();
	

	@Before
	public void setUp() {
		model = new ExactCliffWorldForwardModel();
		
		qTable = new ArrayQTable(stateDiscretizer.getNumberOfStates(), CliffWorldAction.values().length);
		qLearning = new QLearning(learningRate, discountFactor, qTable);
		
		UctPlanningParameters uctParams = new UctPlanningParameters();
		uctParams.discountFactor = discountFactor;
		uctParams.horizon = planningHorizon;
		uctParams.model = model;
		uctParams.qTable = qTable;
		uctParams.stateDiscretizer = stateDiscretizer;
		uctParams.uctConstant = 1.0;
		uctPlanning = new UctPlanning(uctParams);
	}
	

	@Test @Ignore // same problem as with first visit monte carlo
	public void learns_optimal_path() {

		for (int episode = 0; episode < 50; episode++) { // Q-learning takes about 500 episodes to learn the optimal path with high probability 
			
			CliffWorldState fromState = CliffWorldState.START;
			CliffWorldState toState;
			
			do {
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
