package org.atorma.robot.objecttrackingbumper.prioritizedsweeping;

import java.util.*;

import org.apache.commons.math3.distribution.EnumeratedDistribution;
import org.apache.commons.math3.stat.Frequency;
import org.apache.commons.math3.util.Pair;
import org.atorma.robot.discretization.StateDiscretizer;
import org.atorma.robot.learning.montecarlo.ForwardModel;
import org.atorma.robot.learning.prioritizedsweeping.PrioritizedSweepingModel;
import org.atorma.robot.mdp.*;
import org.atorma.robot.objecttrackingbumper.ModeledBumperState;
import org.atorma.robot.simplebumper.BumperAction;

import com.google.common.collect.Sets;

/**
 * A stochastic Bumper world model. Effect of agent's moves on relative obstacle locations
 * are modeled deterministically, but collision probabilities given discrete (state, action)
 * pairs are learned from the data.
 * <p>
 * The initial performance of the agent depends strongly on the state discretization. Either
 * keep the number of states low or initialize prior collision probabilities "well". 
 * It's not required that the state discretization used for action-value reinforcement learning
 * is the same as the one used for predicting collisions.
 * <p>
 * To keep the number of states low, a good discretization could be one where 
 * all states that have an obstacle at some distance interval in the front sector 
 * (e.g. -45..45 degree sector) are projected onto a single state id.
 * <p>
 * You can set prior collision probabilities by creating artificial transition samples 
 * and inputing them to {@link #updateModel(TransitionReward)}. This way, with appropriate 
 * obstacle location, action and result combinations, it's possible to prime the model 
 * even when modeling collision probabilities using more than one sector and combinatorics
 * of obstacle locations come into play.
 */
public class BumperModel implements PrioritizedSweepingModel, ForwardModel {
	
	// Parameters of Beta distribution prior P(collision_prob | front_obstacle_distance, action)
	// for Beta posterior P(collision_prob | data, front_obstacle_distance, action) 
	public static final double BETA_PRIOR_COLLISION = 2;
	public static final double BETA_PRIOR_NO_COLLISION = 10;
	
	private RewardFunction rewardFunction;
	private StateDiscretizer stateDiscretizer;
	private org.apache.commons.math3.stat.Frequency collStats = new Frequency(); // Observed data N(collision | state, action)

	/**
	 * Creates bumper world model where collision probabilities are learned for each (state id, action id)
	 * tuple and the state ids are determined by the given <tt>collisionStateDiscretizer</tt>.
	 * The prior collision probability is 0.1 for each (state id, action id).  
	 * 
	 * @param rewardFunction
	 * 	reward function
	 * @param collisionStateDiscretizer
	 * 	discretizer that projects different {@link ModeledBumperState}s to different ids if the 
	 * states are different enough to require distinct collision probabilities
	 * 	 
	 */
	public BumperModel(RewardFunction rewardFunction, StateDiscretizer collisionStateDiscretizer) {
		this.rewardFunction = rewardFunction;
		this.stateDiscretizer = collisionStateDiscretizer;
	}
		
	
	@Override
	public Set<? extends DiscreteAction> getAllowedActions(State state) {
		return Sets.newHashSet(BumperAction.values());
	}

	@Override
	public Set<StochasticTransitionReward> getOutgoingTransitions(StateAction stateAction) {
		Set<StochasticTransitionReward> transitions = new LinkedHashSet<>();
		
		ModeledBumperState fromState = (ModeledBumperState) stateAction.getState();
		BumperAction action = (BumperAction) stateAction.getAction();
		double collisionProbability = getCollisionProbability(fromState, action);
		
		// In case of collision being the result, if the agent wasn't already collided
		// we model that the action was carried out entirely (e.g. moved the full distance).
		// But if the agent was collided to begin with, then we model no movement.
		ModeledBumperState toStateWhenCollided;
		if (fromState.isCollided()) {
			toStateWhenCollided = (ModeledBumperState) fromState.copy();
		} else {
			toStateWhenCollided = (ModeledBumperState) fromState.afterAction(action);
		}
		toStateWhenCollided.setCollided(true);
		double reward = rewardFunction.getReward(new Transition(stateAction, toStateWhenCollided));
		StochasticTransitionReward transitionRewardWhenCollided = new StochasticTransitionReward(fromState, action, toStateWhenCollided, reward, collisionProbability);
		transitions.add(transitionRewardWhenCollided);
		
		// In case of not colliding, the action changes the state
		ModeledBumperState toStateWhenNotCollided = (ModeledBumperState) fromState.afterAction(action);
		toStateWhenNotCollided.setCollided(false);
		reward = rewardFunction.getReward(new Transition(stateAction, toStateWhenNotCollided));
		StochasticTransitionReward transitionRewardWhenNotCollided = new StochasticTransitionReward(fromState, action, toStateWhenNotCollided, reward, 1 - collisionProbability);
		transitions.add(transitionRewardWhenNotCollided);
		
		return transitions;
	}


	@Override
	public Set<StochasticTransitionReward> getIncomingTransitions(State toState) {
		Set<StochasticTransitionReward> transitions = new LinkedHashSet<>();
		ModeledBumperState toBumperState = (ModeledBumperState) toState;
		
		if (toBumperState.isCollided()) {
			
			// In case of collision, toState is the same as fromState for all actions
			for (BumperAction action : BumperAction.values()) {
				for (Boolean wasCollided : Arrays.asList(true, false)) {
					ModeledBumperState fromState = (ModeledBumperState) toBumperState.copy();
					fromState.setCollided(wasCollided);
					Transition tr = new Transition(fromState, action, toState);
					double reward = rewardFunction.getReward(tr);
					double collisionProbability = getCollisionProbability(fromState, action);
					transitions.add(new StochasticTransitionReward(tr, reward, collisionProbability));
				}
			}
			
		} else {
			
			// Otherwise toState may be a result of any action. For each action, there's a reverse action, so
			// for example for transition "fromState, FORWARD -> toState" we get fromState by taking action
			// BACKWARD from toState. For each action, there's two possible fromStates, one where the agent
			// wasn't collided and one where it was.
			for (BumperAction action : BumperAction.values()) {
				for (Boolean wasCollided : Arrays.asList(true, false)) {
					ModeledBumperState fromState = toBumperState.afterAction(getReverse(action));
					fromState.setCollided(wasCollided);
					Transition tr = new Transition(fromState, action, toState);
					double reward = rewardFunction.getReward(tr);
					double noCollisionProbability = 1 - getCollisionProbability(fromState, action);
					transitions.add(new StochasticTransitionReward(tr, reward, noCollisionProbability));
				}
			}
		}
		
		return transitions;
	}
	
	private BumperAction getReverse(BumperAction action) {
		switch(action) {
		case BACKWARD: return BumperAction.FORWARD;
		case FORWARD: return BumperAction.BACKWARD;
		case LEFT: return BumperAction.RIGHT;
		case RIGHT: return BumperAction.LEFT;
		default: throw new IllegalArgumentException();
		}
	}

	@Override
	public void updateModel(TransitionReward transition) {
		int fromStateId = stateDiscretizer.getId(transition.getFromState());
		ModeledBumperState toState = (ModeledBumperState) transition.getToState();
		
		CollisionObservation collisionObservation = new CollisionObservation(fromStateId, transition.getAction().getId(), toState.isCollided());
		collStats.addValue(collisionObservation);
	}

	
	public double getCollisionProbability(ModeledBumperState state, BumperAction action) {
		int stateId = stateDiscretizer.getId(state);
		return getCollisionProbability(stateId, action.getId());
	}
	
	public double getCollisionProbability(int stateId, int actionId) {
		CollisionObservation collided = new CollisionObservation(stateId, actionId, true);
		CollisionObservation notCollided = new CollisionObservation(stateId, actionId, false);
	
		double probability = (collStats.getCount(collided) + BETA_PRIOR_COLLISION - 1) / 
				             (collStats.getCount(collided) + BETA_PRIOR_COLLISION + collStats.getCount(notCollided) + BETA_PRIOR_NO_COLLISION - 2);
		return probability;
	}
	
	public void printCollisionProbabilities() {
		for (int stateId = 0; stateId < stateDiscretizer.getNumberOfStates(); stateId++) {
			for (BumperAction action : BumperAction.values()) {
				double probability = getCollisionProbability(stateId, action.getId());
				System.out.println("P(c = true | s = " + stateId + ", a = " + action + ") = " + probability);
			}
		}
		
	}
	
	
	private static class CollisionObservation implements Comparable<CollisionObservation> {
		
		private final int stateId;
		private final int actionId;
		private final boolean collision;

		public CollisionObservation(int stateId, int actionId, boolean isCollided) {
			this.stateId = stateId;
			this.actionId = actionId;
			this.collision = isCollided;
		}

		@Override
		public int compareTo(CollisionObservation o) {
			if (this.stateId != o.stateId) {
				return this.stateId - o.stateId;
			}
			if (this.actionId != o.actionId) {
				return this.actionId - o.actionId;
			}
			return Boolean.valueOf(this.collision).compareTo(Boolean.valueOf(o.collision));
		}

		@Override
		public int hashCode() {
			final int prime = 31;
			int result = 1;
			result = prime * result + actionId;
			result = prime * result + (collision ? 1231 : 1237);
			result = prime * result + stateId;
			return result;
		}

		@Override
		public boolean equals(Object obj) {
			if (this == obj)
				return true;
			if (obj == null)
				return false;
			if (getClass() != obj.getClass())
				return false;
			CollisionObservation other = (CollisionObservation) obj;
			if (actionId != other.actionId)
				return false;
			if (collision != other.collision)
				return false;
			if (stateId != other.stateId)
				return false;
			return true;
		}

		
	}


	@Override
	public TransitionReward simulateAction(StateAction fromStateAction) {
		Set<StochasticTransitionReward> outgoing = getOutgoingTransitions(fromStateAction);

		List<Pair<StochasticTransitionReward, Double>> pmf = new ArrayList<>(outgoing.size());
		for (StochasticTransitionReward tr : outgoing) {
			pmf.add(new Pair<>(tr, tr.getProbability()));
		}
		
		return new EnumeratedDistribution<>(pmf).sample();
	}
}
