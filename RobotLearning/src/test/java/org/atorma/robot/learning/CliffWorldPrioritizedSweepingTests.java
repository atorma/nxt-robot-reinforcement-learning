package org.atorma.robot.learning;

import static org.junit.Assert.assertEquals;

import java.util.*;

import org.atorma.robot.learning.cliffworld.*;
import org.atorma.robot.learning.prioritizedsweeping.PrioritizedSweeping;
import org.atorma.robot.learning.prioritizedsweeping.PrioritizedSweepingModel;
import org.atorma.robot.mdp.*;
import org.atorma.robot.policy.EpsilonGreedyPolicy;
import org.junit.Before;
import org.junit.Test;

import com.google.common.collect.Sets;

public class CliffWorldPrioritizedSweepingTests {

	private PrioritizedSweeping sweeping;
	private double discountFactor = 1;
	
	private CliffWorldStateDiscretizer stateDiscretizer = new CliffWorldStateDiscretizer();
	
	private CliffWorldModel model;

	@Before
	public void setUp() {
		model = new CliffWorldModel();
		
		sweeping = new PrioritizedSweeping();
		sweeping.setDiscountFactor(discountFactor);
		sweeping.setStateDiscretizer(stateDiscretizer);
		sweeping.setModel(model);
	}
	
	@Test
	public void with_optimal_path_and_big_reward_in_the_goal_default_q_values_prioritized_sweeping_learns_model_in_one_episode() {
		
		CliffWorldRewardFunction rewardFunction = new ModifiedCliffWorldRewardFunction();
		
		int numIter = CliffWorldEnvironment.OPTIMAL_PATH.size();

		CliffWorldState state = CliffWorldState.START;
		for (int i = 0; i < CliffWorldEnvironment.OPTIMAL_PATH.size(); i++) {
			
			CliffWorldAction action = CliffWorldEnvironment.OPTIMAL_PATH.get(i);
			CliffWorldState nextState = state.getNextState(action);
			Transition transition = new Transition(state, action, nextState);
			TransitionReward transitionReward = new TransitionReward(transition, rewardFunction.getReward(transition));
			sweeping.updateModel(transitionReward);
			
			sweeping.setSweepStartStateAction(new StateAction(state, action));
			sweeping.performIterations(numIter);

			state = nextState;
		}
		
		List<CliffWorldAction> learnedPath = getLearnedPath();
		assertEquals(CliffWorldEnvironment.OPTIMAL_PATH, learnedPath);
	}
	
	@Test
	public void learns_optimal_path_in_few_episodes() {
		
		CliffWorldRewardFunction rewardFunction = new CliffWorldRewardFunction();
		EpsilonGreedyPolicy policy = new EpsilonGreedyPolicy(0.1, sweeping, CliffWorldAction.values());
		
		for (int episode = 0; episode < 10; episode++) { // Q-learning takes about 500 episodes to learn the optimal path with high probability 
			
			CliffWorldState fromState = CliffWorldState.START;
			CliffWorldState toState;
			
			do {
				int fromStateId = stateDiscretizer.getId(fromState);
				Integer byActionId = policy.getActionId(fromStateId);
				CliffWorldAction byAction = CliffWorldAction.getActionById(byActionId);
				toState = fromState.getNextState(byAction);
				Transition transition = new Transition(fromState, byAction, toState);
				double reward = rewardFunction.getReward(transition);
				
				sweeping.updateModel(new TransitionReward(transition, reward));
				sweeping.setSweepStartStateAction(transition.getFromStateAction());
				sweeping.performIterations(50);
				
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
			int actionId = sweeping.getActionId(stateId);
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
	
	// Cliff world model where we know the world is deterministic but 
	// we don't know the transitions nor the reward function up front.
	private static class CliffWorldModel implements PrioritizedSweepingModel {
		
		private static final double TRANSITION_PROBABILITY = 1.0;
		
		private Map<StateAction, StochasticTransitionReward> outgoingTransitions = new HashMap<>();
		private Map<State, Set<StochasticTransitionReward>> incomingTransitions = new HashMap<>();
		
		@Override
		public Set<CliffWorldAction> getAllActions() {
			return Sets.newHashSet(CliffWorldAction.values());
		}
		
		@Override
		public Set<StochasticTransitionReward> getOutgoingTransitions(StateAction stateAction) {
			if (outgoingTransitions.get(stateAction) != null) {
				// There's only one possible transition since the world is deterministic
				return Sets.newHashSet(outgoingTransitions.get(stateAction));
			}
			return Collections.emptySet();
		}
		
		@Override
		public Set<StochasticTransitionReward> getIncomingTransitions(State state) {
			Set<StochasticTransitionReward> incoming = this.incomingTransitions.get(state);
			if (incoming == null) {
				incoming = new HashSet<>();
				this.incomingTransitions.put(state, incoming);
			}
			return incoming;
		}

		@Override
		public void updateModel(TransitionReward observation) {
			this.outgoingTransitions.put(observation.getFromStateAction(), new StochasticTransitionReward(observation, TRANSITION_PROBABILITY));
			
			Set<StochasticTransitionReward> incoming = getIncomingTransitions(observation.getToState());
			incoming.add(new StochasticTransitionReward(observation, TRANSITION_PROBABILITY));
		}

		
	}
}
