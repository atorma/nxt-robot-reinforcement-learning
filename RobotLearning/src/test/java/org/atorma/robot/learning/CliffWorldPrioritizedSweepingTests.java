package org.atorma.robot.learning;

import static org.junit.Assert.assertEquals;

import java.util.*;

import org.atorma.robot.learning.cliffworld.*;
import org.atorma.robot.mdp.*;
import org.junit.Before;
import org.junit.Test;

import com.google.common.collect.Sets;

public class CliffWorldPrioritizedSweepingTests {

	private PrioritizedSweeping sweeping;
	private double discountFactor = 1;
	
	private CliffWorldStateDiscretizer stateDiscretizer = new CliffWorldStateDiscretizer();
	private CliffWorldRewardFunction rewardFunction = new CliffWorldRewardFunction();
	
	private HashMapQTable qTable;
	private double defaultQValue = -5;
	private CliffWorldModel model;

	@Before
	public void setUp() {
		qTable = new HashMapQTable(defaultQValue); // alternative to discouraging default q-values would be big reward at goal
		for (CliffWorldAction action : CliffWorldAction.values()) {
			qTable.addActionId(action.getId());
		}
		model = new CliffWorldModel();
		
		sweeping = new PrioritizedSweeping();
		sweeping.setDiscountFactor(discountFactor);
		sweeping.setStateDiscretizer(stateDiscretizer);
		sweeping.setQTable(qTable);
		sweeping.setModel(model);
	}
	
	@Test
	public void with_optimal_path_and_discouraging_default_q_values_prioritized_sweeping_learns_model() {

		CliffWorldState state = CliffWorldState.START;
		for (int i = 0; i < CliffWorldEnvironment.OPTIMAL_PATH.size(); i++) {
			
			CliffWorldAction action = CliffWorldEnvironment.OPTIMAL_PATH.get(i);
			CliffWorldState nextState = state.getNextState(action);
			Transition transition = new Transition(state, action, nextState);
			TransitionReward transitionReward = new TransitionReward(transition, rewardFunction.getReward(transition));
			sweeping.updateModel(transitionReward);
			
			sweeping.setCurrentStateAction(new StateAction(state, action));
			sweeping.performIterations(CliffWorldEnvironment.OPTIMAL_PATH.size());

			state = nextState;
		}
		
		List<CliffWorldAction> learnedPath = getLearnedPath();
		assertEquals(CliffWorldEnvironment.OPTIMAL_PATH, learnedPath);
	}
	
	private List<CliffWorldAction> getLearnedPath() {
		CliffWorldState state = CliffWorldState.START;
		List<CliffWorldAction> learnedActions = new ArrayList<>();
		while (!state.isGoal()) {
			int stateId = stateDiscretizer.getId(state.getValues());
			int actionId = qTable.getActionId(stateId);
			CliffWorldAction action = CliffWorldAction.getActionById(actionId);
			learnedActions.add(action);
			state = state.getNextState(action);
		}
		return learnedActions;
	}
	
	
	// Cliff world model where we know the world is deterministic but 
	// we don't know the transitions nor the reward function up front.
	private static class CliffWorldModel implements MarkovModel {
		
		private static final double TRANSITION_PROBABILITY = 1.0;
		
		private Map<StateAction, StochasticTransitionReward> outgoingTransitions = new HashMap<>();
		private Map<State, Set<StochasticTransitionReward>> incomingTransitions = new HashMap<>();
		
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
		public Set<StateAction> getPredecessors(State state) {
			Set<StateAction> predecessors = new HashSet<>();
			for (StochasticTransitionReward tr : this.incomingTransitions.get(state)) {
				predecessors.add(tr.getFromStateAction());
			}
			return predecessors;
		}

		@Override
		public double getTransitionProbability(Transition transition) {
			return TRANSITION_PROBABILITY; // Deterministic world
		}

		@Override
		public void updateModel(TransitionReward observation) {
			this.outgoingTransitions.put(observation.getFromStateAction(), new StochasticTransitionReward(observation, TRANSITION_PROBABILITY));
			
			Set<StochasticTransitionReward> incoming = getIncomingTransitions(observation.getToState());
			incoming.add(new StochasticTransitionReward(observation, TRANSITION_PROBABILITY));
		}

		
		
	}
}
