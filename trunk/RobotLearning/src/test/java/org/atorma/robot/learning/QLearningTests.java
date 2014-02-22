package org.atorma.robot.learning;

import static org.junit.Assert.assertEquals;

import java.util.Arrays;

import org.atorma.robot.Action;
import org.atorma.robot.DiscretePolicy;
import org.atorma.robot.EpsilonGreedyPolicy;
import org.atorma.robot.RewardFunction;
import org.atorma.robot.State;
import org.atorma.robot.discretization.IdFunction;
import org.junit.Before;
import org.junit.Test;

public class QLearningTests {

	private QLearning qLearning;
	private DiscretePolicy learnedPolicy;
	private ArrayHashCode idFunction = new ArrayHashCode();
	private CliffWorldRewardFunction rewardFunction = new CliffWorldRewardFunction();
	
	private int episode;
	private double episodeReward;
	
	@Before
	public void setUp() {
		qLearning = new QLearning(idFunction, idFunction, rewardFunction, 0.1, 1);
	}
	
	@Test
	public void cliff_world_q_learning_with_epsilon_greedy_policy_finds_optimal_policy() {
		learnPolicy();

		CliffWorldState state = CliffWorldState.START;
		CliffWorldAction action;
		
		action = CliffWorldAction.UP;
		assertExpectedActionEqualsLearnedAction(action, state);
		
		for (int i=0; i<11; i++) {
			state = state.getNextState(action);
			action = CliffWorldAction.RIGHT;
			assertExpectedActionEqualsLearnedAction(action, state);
		} 
		
		state = state.getNextState(action);
		action = CliffWorldAction.DOWN;
		assertExpectedActionEqualsLearnedAction(action, state);
		
	}
	
	private void assertExpectedActionEqualsLearnedAction(CliffWorldAction expectedAction, CliffWorldState currentState) {
		int expectedActionId = idFunction.getId(expectedAction.getValues());
		int currentStateId = idFunction.getId(currentState.getValues());
		int learnedActionId = learnedPolicy.getActionId(currentStateId);
		assertEquals(expectedActionId, learnedActionId);
	}

	private void learnPolicy() {
		EpsilonGreedyPolicy agent = new EpsilonGreedyPolicy(new Action[] {CliffWorldAction.UP, CliffWorldAction.DOWN, CliffWorldAction.LEFT, CliffWorldAction.RIGHT}, idFunction, 0.1);
		
		int numEpisodes = 500;
		
		for (episode=0; episode<numEpisodes; episode++) {
			episodeReward = 0;
			
			CliffWorldState fromState = CliffWorldState.START;
			CliffWorldState toState;
			
			do {
				int fromStateId = idFunction.getId(fromState.getValues());
				Integer byActionId = agent.getActionId(fromStateId);
				CliffWorldAction byAction = CliffWorldAction.getActionById(byActionId);
				toState = fromState.getNextState(byAction);
				Transition transition = new Transition(fromState, byAction, toState);
				
				episodeReward += rewardFunction.getReward(transition);
				qLearning.update(transition);
				agent.setDeterministicPolicy(qLearning.getLearnedPolicy());
				fromState = toState;

			} while (!toState.isGoal());
			
			System.out.println("episode " + episode +", reward " + episodeReward);
		}
		
		learnedPolicy = qLearning.getLearnedPolicy();
	}
	
	
	public static class CliffWorldState implements State {

		public static final int X_MIN = 0;
		public static final int X_MAX = 11;
		public static final int Y_MIN = 0;
		public static final int Y_MAX = 3;

		public static final CliffWorldState START = new CliffWorldState(X_MIN, Y_MIN);
		
		private final double x;
		private final double y;
		
		public CliffWorldState(int x, int y) {
			this.x = x;
			this.y = y;
		}
		
		@Override
		public double[] getValues() {
			return new double[] {x, y};
		}
		
		// Cliff world environment
		
		public boolean isOutOfBounds() {
			return x < X_MIN || x > X_MAX || y < Y_MIN || y > X_MAX;
		}
		
		public boolean isCliff() {
			return x > X_MIN && x < X_MAX && y == Y_MIN; 
		}
		
		public boolean isGoal() {
			return x == X_MAX && y == Y_MIN;
		}
		
		public CliffWorldState getNextState(CliffWorldAction action) {
			CliffWorldState nextState = new CliffWorldState((int) (x + action.getValues()[0]), (int) (y + action.getValues()[1]));
			if (nextState.isOutOfBounds() || isGoal()) {
				return this;
			} else if (nextState.isCliff()) {
				return START;
			} else {
				return nextState;
			}
		}
	}
	
	public static class CliffWorldAction implements Action {
		
		public static final CliffWorldAction UP = new CliffWorldAction(0, 1);
		public static final CliffWorldAction DOWN = new CliffWorldAction(0, -1);
		public static final CliffWorldAction LEFT = new CliffWorldAction(-1, 0);
		public static final CliffWorldAction RIGHT = new CliffWorldAction(1, 0);
		
		private final double[] values;
		
		public CliffWorldAction(int dx, int dy) {
			this.values = new double[] {dx, dy};
		}

		@Override
		public double[] getValues() {
			return values;
		}
		
		public static CliffWorldAction getActionById(int id) {
			IdFunction idMap = new ArrayHashCode();
			if (id == idMap.getId(UP.getValues())) {
				return UP;
			} else if (id == idMap.getId(DOWN.getValues())) {
				return DOWN;
			} else if (id == idMap.getId(LEFT.getValues())) {
				return LEFT;
			} else if (id == idMap.getId(RIGHT.getValues())) {
				return RIGHT;
			} else {
				throw new IllegalArgumentException();
			}
		}
		
	}
	
	public static class CliffWorldRewardFunction implements RewardFunction {

		@Override
		public double getReward(Transition transition) {
			CliffWorldState toState = (CliffWorldState) transition.getToState();
			if (toState.isCliff()) {
				return -100;
			} else {
				return -1;
			}
		}
		
	}
	
	public static class ArrayHashCode implements IdFunction {

		@Override
		public int getId(double[] value) {
			return Arrays.hashCode(value);
		}
		
	}
		
}
