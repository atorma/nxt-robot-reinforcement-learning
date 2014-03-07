package org.atorma.robot.objecttrackingbumper.prioritizedsweeping;

import java.util.*;

import org.apache.commons.math.stat.Frequency;
import org.atorma.robot.discretization.Discretizer;
import org.atorma.robot.learning.prioritizedsweeping.PrioritizedSweepingModel;
import org.atorma.robot.mdp.*;
import org.atorma.robot.simplebumper.BumperAction;

import com.google.common.collect.Sets;

public class BumperModel implements PrioritizedSweepingModel {
	
	private RewardFunction rewardFunction;
	private Frequency conditionalCollisionFrequency = new Frequency();
	private Map<ObstacleDistanceActionCollision, Double> betaPriorParameters = new HashMap<>();
	private Discretizer obstacleDistanceDiscretizer;	
	
	
	public BumperModel(RewardFunction rewardFunction, Discretizer obstacleDistanceDiscretizer) {
		this.rewardFunction = rewardFunction;
		this.obstacleDistanceDiscretizer = obstacleDistanceDiscretizer;
		
		// Parameters of Beta prior distribution P(collision_prob | front_obstacle_distance, action)
		// for P(collision_prob | collision_event, front_obstacle_distance, action)
		// for each front_obstacle_distance and action.
		// Since we're only interested in MAP estimates, prior observations are sufficient
		int distanceId = 0;
		betaPriorParameters.put(new ObstacleDistanceActionCollision(distanceId, BumperAction.FORWARD, true), 9.0);
		betaPriorParameters.put(new ObstacleDistanceActionCollision(distanceId, BumperAction.FORWARD, false), 3.0);
		for (BumperAction action : Arrays.asList(BumperAction.BACKWARD, BumperAction.LEFT, BumperAction.RIGHT)) {
			betaPriorParameters.put(new ObstacleDistanceActionCollision(distanceId, action, true), 2.0);
			betaPriorParameters.put(new ObstacleDistanceActionCollision(distanceId, action, false), 10.0);
		}
		
		for (distanceId = 1; distanceId < obstacleDistanceDiscretizer.getNumberOfBins(); distanceId++) {
			for (BumperAction action : BumperAction.values()) {
				betaPriorParameters.put(new ObstacleDistanceActionCollision(distanceId, action, true), 2.0);
				betaPriorParameters.put(new ObstacleDistanceActionCollision(distanceId, action, false), 10.0);
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
		
		// In case of not colliding, state is changes
		ModeledBumperState toStateWhenNotCollided = (ModeledBumperState) fromState.copy(); // TODO update, not copy!
		toStateWhenNotCollided.setCollided(false);
		reward = rewardFunction.getReward(new Transition(stateAction, toStateWhenNotCollided));
		StochasticTransitionReward transitionRewardWhenNotCollided = new StochasticTransitionReward(fromState, action, toStateWhenNotCollided, reward, 1 - collisionProbability);
		transitions.add(transitionRewardWhenNotCollided);
		
		return transitions;
	}

	@Override
	public Set<StochasticTransitionReward> getIncomingTransitions(State state) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void updateModel(TransitionReward observation) {
		ModeledBumperState fromState = (ModeledBumperState) observation.getFromState();
		BumperAction action = (BumperAction) observation.getAction();
		ModeledBumperState toState = (ModeledBumperState) observation.getToState();
		ObstacleDistanceActionCollision collisionObservation = new ObstacleDistanceActionCollision(
				obstacleDistanceDiscretizer.discretize(fromState.getObjectInSectorDegree(0).getDistance()),
				action,
				toState.isCollided());
		conditionalCollisionFrequency.addValue(collisionObservation);
	}

	
	private double getCollisionProbability(ModeledBumperState state, BumperAction action) {
		ObstacleDistanceActionCollision queryCollided = new ObstacleDistanceActionCollision(
				obstacleDistanceDiscretizer.discretize(state.getObjectInSectorDegree(0).getDistance()),
				action,
				true);
		ObstacleDistanceActionCollision queryNotCollided = new ObstacleDistanceActionCollision(
				obstacleDistanceDiscretizer.discretize(state.getObjectInSectorDegree(0).getDistance()),
				action,
				false);
		return 
				(conditionalCollisionFrequency.getCount(queryCollided) + betaPriorParameters.get(queryCollided) - 1) 
				/ 
				(conditionalCollisionFrequency.getCount(queryCollided) + betaPriorParameters.get(queryCollided) 
						+ conditionalCollisionFrequency.getCount(queryNotCollided) + betaPriorParameters.get(queryNotCollided) - 2);
	}
	
	
	private static class ObstacleDistanceActionCollision implements Comparable<ObstacleDistanceActionCollision> {
		
		private final Integer discretizedDistance;
		private final BumperAction action;
		private final Boolean isCollided;

		public ObstacleDistanceActionCollision(int discretizedDistance, BumperAction action, boolean isCollided) {
			this.discretizedDistance = discretizedDistance;
			this.action = action;
			this.isCollided = isCollided;
		}

		@Override
		public int compareTo(ObstacleDistanceActionCollision o) {
			if (this.discretizedDistance != o.discretizedDistance) {
				return this.discretizedDistance.compareTo(o.discretizedDistance);
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
			result = prime * result + discretizedDistance;
			result = prime * result + (isCollided ? 1231 : 1237);
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
			ObstacleDistanceActionCollision other = (ObstacleDistanceActionCollision) obj;
			if (action != other.action)
				return false;
			if (discretizedDistance != other.discretizedDistance)
				return false;
			if (isCollided != other.isCollided)
				return false;
			return true;
		}
		
		
	}
}
