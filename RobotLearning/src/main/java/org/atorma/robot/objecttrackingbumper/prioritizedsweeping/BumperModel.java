package org.atorma.robot.objecttrackingbumper.prioritizedsweeping;

import java.util.*;

import org.apache.commons.math.stat.Frequency;
import org.atorma.robot.discretization.Discretizer;
import org.atorma.robot.learning.prioritizedsweeping.PrioritizedSweepingModel;
import org.atorma.robot.mdp.*;
import org.atorma.robot.objecttracking.TrackedObject;
import org.atorma.robot.objecttrackingbumper.ModeledBumperState;
import org.atorma.robot.simplebumper.BumperAction;

import com.google.common.collect.Sets;

public class BumperModel implements PrioritizedSweepingModel {
	
	private RewardFunction rewardFunction;
	private Discretizer obstacleDistanceDiscretizer;
	// Observed data N(collision_event | front_obstacle_distance, action)
	private Frequency collFreq = new Frequency(); 
	// Parameters of Beta distribution prior P(collision_prob | front_obstacle_distance, action)
	// for Beta posterior P(collision_prob | data, front_obstacle_distance, action) 
	private Map<CollisionObservation, Double> priorParams = new HashMap<>(); 
		

	public BumperModel(RewardFunction rewardFunction, Discretizer obstacleDistanceDiscretizer) {
		this.rewardFunction = rewardFunction;
		this.obstacleDistanceDiscretizer = obstacleDistanceDiscretizer;
		
		// Set up the prior so that when obstacle is close in front, driving forward results in collision
		// with high probability. For other obstacle states, when the robot is collided, 
		// another collision is more likely than when not. 
		int distanceId = 0;
		priorParams.put(new CollisionObservation(distanceId, true, BumperAction.FORWARD, true), 9.0);
		priorParams.put(new CollisionObservation(distanceId, true, BumperAction.FORWARD, false), 3.0);
		priorParams.put(new CollisionObservation(distanceId, false, BumperAction.FORWARD, true), 9.0);
		priorParams.put(new CollisionObservation(distanceId, false, BumperAction.FORWARD, false), 3.0);
		for (BumperAction action : Arrays.asList(BumperAction.BACKWARD, BumperAction.LEFT, BumperAction.RIGHT)) {
			priorParams.put(new CollisionObservation(distanceId, true, action, true), 2.0);
			priorParams.put(new CollisionObservation(distanceId, true, action, false), 20.0);
			priorParams.put(new CollisionObservation(distanceId, false, action, true), 2.0);
			priorParams.put(new CollisionObservation(distanceId, false, action, false), 20.0);
		}
		for (distanceId = 1; distanceId < obstacleDistanceDiscretizer.getNumberOfBins(); distanceId++) {
			for (BumperAction action : BumperAction.values()) {
				priorParams.put(new CollisionObservation(distanceId, true, action, true), 2.0);
				priorParams.put(new CollisionObservation(distanceId, true, action, false), 20.0);
				priorParams.put(new CollisionObservation(distanceId, false, action, true), 2.0);
				priorParams.put(new CollisionObservation(distanceId, false, action, false), 20.0);
			}
		}

	}
	
	@Override
	public Set<? extends DiscreteAction> getAllActions() {
		return Sets.newHashSet(BumperAction.values());
	}

	@Override
	public Set<StochasticTransitionReward> getOutgoingTransitions(StateAction stateAction) {
		Set<StochasticTransitionReward> transitions = new LinkedHashSet<>();
		
		ModeledBumperState fromState = (ModeledBumperState) stateAction.getState();
		BumperAction action = (BumperAction) stateAction.getAction();
		double collisionProbability = getCollisionProbability(fromState, action);
		
		// In case of collision, state is unchanged
		ModeledBumperState toStateWhenCollided = (ModeledBumperState) fromState.copy();
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
		ModeledBumperState fromState = (ModeledBumperState) transition.getFromState();
		BumperAction action = (BumperAction) transition.getAction();
		ModeledBumperState toState = (ModeledBumperState) transition.getToState();
		TrackedObject obstacleInFront = fromState.getObjectInSectorDegree(0);
		double obstacleDistanceFront = obstacleInFront != null ? obstacleInFront.getDistance() : Double.MAX_VALUE;
		int discretizedDistance = obstacleDistanceDiscretizer.discretize(obstacleDistanceFront);
		CollisionObservation collisionObservation = new CollisionObservation(discretizedDistance, fromState.isCollided(), action, toState.isCollided());
		collFreq.addValue(collisionObservation);
	}

	
	public double getCollisionProbability(ModeledBumperState state, BumperAction action) {
		TrackedObject obstacleInFront = state.getObjectInSectorDegree(0);
		int discretizedDistance = obstacleDistanceDiscretizer.discretize(obstacleInFront != null ? obstacleInFront.getDistance() : Double.MAX_VALUE); 
		CollisionObservation collided = new CollisionObservation(discretizedDistance, state.isCollided(), action, true);
		CollisionObservation notCollided = new CollisionObservation(discretizedDistance, state.isCollided(), action, false);
		
		double probability = (collFreq.getCount(collided) + priorParams.get(collided) - 1) / 
				             (collFreq.getCount(collided) + priorParams.get(collided) + collFreq.getCount(notCollided) + priorParams.get(notCollided) - 2);
		
		//System.out.println("P(coll = 1 | d = " + discretizedDistance + ", a = " + action + ") = " + probability);
		return probability;
	}
	
	
	private static class CollisionObservation implements Comparable<CollisionObservation> {
		
		private final Integer discretizedDistance;
		private final Boolean wasCollided;
		private final BumperAction action;
		private final Boolean isCollided;

		public CollisionObservation(int discretizedDistance, boolean wasCollided, BumperAction action, boolean isCollided) {
			this.discretizedDistance = discretizedDistance;
			this.wasCollided = wasCollided;
			this.action = action;
			this.isCollided = isCollided;
		}

		@Override
		public int compareTo(CollisionObservation o) {
			if (this.discretizedDistance != o.discretizedDistance) {
				return this.discretizedDistance.compareTo(o.discretizedDistance);
			}
			if (this.wasCollided != o.wasCollided) {
				return this.wasCollided.compareTo(o.wasCollided);
			}
			if (this.action != o.action) {
				return this.action.compareTo(o.action);
			}
			return this.isCollided.compareTo(o.isCollided);
		}

		@Override
		public int hashCode() {
			final int prime = 31;
			int result = 1;
			result = prime * result
					+ ((action == null) ? 0 : action.hashCode());
			result = prime
					* result
					+ ((discretizedDistance == null) ? 0 : discretizedDistance
							.hashCode());
			result = prime * result
					+ ((isCollided == null) ? 0 : isCollided.hashCode());
			result = prime * result
					+ ((wasCollided == null) ? 0 : wasCollided.hashCode());
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
			if (action != other.action)
				return false;
			if (discretizedDistance == null) {
				if (other.discretizedDistance != null)
					return false;
			} else if (!discretizedDistance.equals(other.discretizedDistance))
				return false;
			if (isCollided == null) {
				if (other.isCollided != null)
					return false;
			} else if (!isCollided.equals(other.isCollided))
				return false;
			if (wasCollided == null) {
				if (other.wasCollided != null)
					return false;
			} else if (!wasCollided.equals(other.wasCollided))
				return false;
			return true;
		}

		
		
		
	}
}
