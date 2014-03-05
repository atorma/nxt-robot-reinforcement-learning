package org.atorma.robot.learning;

import java.util.*;

import org.atorma.robot.discretization.VectorDiscretizer;
import org.atorma.robot.learning.cliffworld.CliffWorldRewardFunction;
import org.atorma.robot.learning.cliffworld.CliffWorldStateDiscretizer;
import org.atorma.robot.mdp.*;
import org.junit.Before;

import com.google.common.collect.Sets;

public class CliffWorldPrioritizedSweepingTests {

	private PrioritizedSweeping sweeping;
	private double discountFactor = 1;
	
	private CliffWorldStateDiscretizer stateDiscretizer = new CliffWorldStateDiscretizer();
	private CliffWorldRewardFunction rewardFunction = new CliffWorldRewardFunction();
	
	private DiscreteQFunction qFunction = new HashMapQTable();

	@Before
	public void setUp() {
		sweeping = new PrioritizedSweeping();
		sweeping.setDiscountFactor(discountFactor);
		sweeping.setStateDiscretizer(stateDiscretizer);
	}
	
	// Cliff world model where we know the world is deterministic but 
	// we don't know the transitions nor the reward function up front.
	private static class CliffWorldModel implements MarkovModel {
		
		private Map<StateAction, TransitionReward> stateActionToReward = new HashMap<>();
		private Map<State, Set<StateAction>> stateToPredecessors = new HashMap<>();
		
		@Override
		public Set<StochasticTransitionReward> getTransitions(StateAction stateAction) {
			Set<StochasticTransitionReward> transitions = new HashSet<>();
			if (stateActionToReward.get(stateAction) != null) {
				return Sets.newHashSet(new StochasticTransitionReward(stateActionToReward.get(stateAction), 1.0));
			}
			return Collections.emptySet();
		}

		@Override
		public Set<StateAction> getPredecessors(State state) {
			Set<StateAction> predecessors = this.stateToPredecessors.get(state);
			if (predecessors == null) {
				predecessors = new HashSet<StateAction>();
				this.stateToPredecessors.put(state, predecessors);
			}
			return stateToPredecessors.get(state);
		}

		@Override
		public double getTransitionProbability(Transition transition) {
			return 1;
		}

		@Override
		public void updateModel(TransitionReward observation) {
			this.stateActionToReward.put(observation.getFromStateAction(), observation);
			
			Set<StateAction> predecessors = getPredecessors(observation.getToState());
			predecessors.add(observation.getFromStateAction());
		}
		
	}
}
