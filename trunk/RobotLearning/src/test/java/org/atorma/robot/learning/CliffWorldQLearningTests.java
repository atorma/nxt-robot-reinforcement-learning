package org.atorma.robot.learning;

import static org.junit.Assert.assertEquals;

import java.util.Arrays;

import org.atorma.robot.discretization.VectorDiscretizer;
import org.atorma.robot.mdp.*;
import org.atorma.robot.policy.EpsilonGreedyPolicy;
import org.junit.Before;
import org.junit.Test;

public class CliffWorldQLearningTests {

	private QLearning<CliffWorldState, CliffWorldAction> qLearning;
	private ArrayHashCode stateDiscretizer = new ArrayHashCode();
	private CliffWorldRewardFunction rewardFunction = new CliffWorldRewardFunction();
	
	
	@Before
	public void setUp() {
		qLearning = new QLearning<>(stateDiscretizer, rewardFunction, 0.1, 1);
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
		int expectedActionId = expectedAction.getId();
		int currentStateId = stateDiscretizer.getId(currentState.getValues());
		int learnedActionId = qLearning.getActionId(currentStateId);
		assertEquals(expectedActionId, learnedActionId);
	}

	private void learnPolicy() {
		EpsilonGreedyPolicy policy = new EpsilonGreedyPolicy(0.1, 
				new DiscreteAction[] {CliffWorldAction.UP, CliffWorldAction.DOWN, CliffWorldAction.LEFT, CliffWorldAction.RIGHT},
				qLearning);
		
		int numEpisodes = 500;
		
		for (int episode=0; episode<numEpisodes; episode++) {
			
			CliffWorldState fromState = CliffWorldState.START;
			CliffWorldState toState;
			
			do {
				int fromStateId = stateDiscretizer.getId(fromState.getValues());
				Integer byActionId = policy.getActionId(fromStateId);
				CliffWorldAction byAction = CliffWorldAction.getActionById(byActionId);
				toState = fromState.getNextState(byAction);
				Transition<CliffWorldState, CliffWorldAction> transition = new Transition<>(fromState, byAction, toState);

				qLearning.update(transition);
				fromState = toState;

			} while (!toState.isGoal());

		}
	}
	
	
	public static class CliffWorldState implements State {

		public static final int X_MIN = 0;
		public static final int X_MAX = 11;
		public static final int Y_MIN = 0;
		public static final int Y_MAX = 3;

		public static final CliffWorldState START = new CliffWorldState(X_MIN, Y_MIN);
		
		private final int x;
		private final int y;
		
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
			CliffWorldState nextState = new CliffWorldState(x + action.dx, y + action.dy);
			if (nextState.isOutOfBounds() || isGoal()) {
				return this;
			} else if (nextState.isCliff()) {
				return START;
			} else {
				return nextState;
			}
		}
	}
	
	public static class CliffWorldAction implements DiscreteAction {
		
		public static final CliffWorldAction UP = new CliffWorldAction(0, 1, 0);
		public static final CliffWorldAction DOWN = new CliffWorldAction(0, -1, 1);
		public static final CliffWorldAction LEFT = new CliffWorldAction(-1, 0, 2);
		public static final CliffWorldAction RIGHT = new CliffWorldAction(1, 0, 3);
		
		public final int dx;
		public final int dy;
		public final int id; 
		
		public CliffWorldAction(int dx, int dy, int id) {
			this.dx = dx;
			this.dy = dy;
			this.id = id;
		}

		@Override
		public int getId() {
			return id;
		}
		
		public static CliffWorldAction getActionById(int id) {
			if (id == UP.id) {
				return UP;
			} else if (id == DOWN.id) {
				return DOWN;
			} else if (id == LEFT.id) {
				return LEFT;
			} else if (id == RIGHT.id) {
				return RIGHT;
			} else {
				throw new IllegalArgumentException();
			}
		}
		
	}
	
	public static class CliffWorldRewardFunction implements RewardFunction<CliffWorldState, CliffWorldAction> {

		@Override
		public double getReward(Transition<CliffWorldState, CliffWorldAction> transition) {
			CliffWorldState toState = transition.getToState();
			if (toState.isCliff()) {
				return -100;
			} else {
				return -1;
			}
		}
		
	}
	
	public static class ArrayHashCode implements VectorDiscretizer {

		@Override
		public int getId(double[] value) {
			return Arrays.hashCode(value);
		}
		
	}
		
}
