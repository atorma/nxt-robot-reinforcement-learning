package org.atorma.robot.learning.cliffworld;

import java.util.*;

import org.atorma.robot.learning.prioritizedsweeping.PrioritizedSweepingModel;
import org.atorma.robot.mdp.*;

import com.google.common.collect.Sets;

/**
 * A cliff world model where we know the world is deterministic but 
 * we don't know the transitions nor the reward function up front.
 */
public class LearningCliffWorldModel implements PrioritizedSweepingModel {
	
	private static final double TRANSITION_PROBABILITY = 1.0;
	
	private Map<StateAction, StochasticTransitionReward> outgoingTransitions = new HashMap<>();
	private Map<State, Set<StochasticTransitionReward>> incomingTransitions = new HashMap<>();
	
	@Override
	public Set<CliffWorldAction> getAllowedActions(State state) {
		CliffWorldState cliffWorldState = (CliffWorldState) state;
		if (cliffWorldState.isCliff() || cliffWorldState.isGoal()) {
			return Collections.emptySet();
		}
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

	public void update(TransitionReward observation) {
		this.outgoingTransitions.put(observation.getFromStateAction(), new StochasticTransitionReward(observation, TRANSITION_PROBABILITY));
		
		Set<StochasticTransitionReward> incoming = getIncomingTransitions(observation.getToState());
		incoming.add(new StochasticTransitionReward(observation, TRANSITION_PROBABILITY));
	}

}