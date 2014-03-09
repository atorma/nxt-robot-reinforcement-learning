package org.atorma.robot.objecttrackingbumper.prioritizedsweeping;

import static org.junit.Assert.*;

import java.util.Set;

import org.atorma.robot.mdp.*;
import org.atorma.robot.objecttracking.TrackedObject;
import org.atorma.robot.objecttrackingbumper.*;
import org.atorma.robot.simplebumper.BumperAction;
import org.junit.Before;
import org.junit.Test;

public class BumperModelTests {

	private static final double PRIOR_COLLISION_PROB_WHEN_DRIVING_TOWARDS_NEAR_OBSTACLE = 0.8;
	private static final double PRIOR_COLLISION_PROBABILITY_OTHERWISE = 0.05;

	private BumperModel model;
	
	private ObstacleDistanceDiscretizer obstacleDistanceDiscretizer = new ObstacleDistanceDiscretizer();
	private BumperRewardFunction rewardFunction = new BumperRewardFunction();
	
	
	@Before
	public void setUp() {
		model = new BumperModel(rewardFunction, obstacleDistanceDiscretizer); 
	}
	
	@Test
	public void test_bayesian_collision_probability_prior() {
		
		for (int distanceBin = 0; distanceBin < obstacleDistanceDiscretizer.getNumberOfBins(); distanceBin++) {
			
			double distance = (distanceBin + 0.5) * obstacleDistanceDiscretizer.getBinWidth() + obstacleDistanceDiscretizer.getMin();
			
			for (BumperAction action : BumperAction.values()) {	
				
				ModeledBumperState state = getState(distance);
				
				if (distanceBin == 0 && action == BumperAction.FORWARD) {
					// Prior probability is big when obstacle is close in front and action is to drive forward.
					assertEquals(PRIOR_COLLISION_PROB_WHEN_DRIVING_TOWARDS_NEAR_OBSTACLE, model.getCollisionProbability(state, action), 0.0001);
				} else {
					// In all other cases prior collision probability is small.
					assertEquals(PRIOR_COLLISION_PROBABILITY_OTHERWISE, model.getCollisionProbability(state, action), 0.0001);
				}
			}
		}
		
		// When no obstacle in front
		ModeledBumperState state = new ModeledBumperState();
		for (BumperAction action : BumperAction.values()) {	
			assertEquals(PRIOR_COLLISION_PROBABILITY_OTHERWISE, model.getCollisionProbability(state, action), 0.0001);
		}
	}

	@Test
	public void collision_probability_is_updated_after_observations() {
		
		// Agent bumps 10 times after turning right when obstacle is close in front and the agent is already collided
		ModeledBumperState fromState = getState(obstacleDistanceDiscretizer.getMin() + 0.5 * obstacleDistanceDiscretizer.getBinWidth());
		fromState.setCollided(true);
		BumperAction action = BumperAction.RIGHT;
		ModeledBumperState toState = fromState.afterAction(action);
		toState.setCollided(true);
		for (int i = 0; i < 10; i++) {
			TransitionReward transition = new TransitionReward(fromState, action, toState, -100);
			model.updateModel(transition);
		}
		// here we assume prior parameters alpha = 2 (collision), beta = 10 (no collision) 
		assertEquals(0.367, model.getCollisionProbability(fromState, action), 0.001); 
		
		// Collision probability when turning right when obstacle is close but the agent is NOT already collided is NOT updated -
		// these are separate probabilities.
		fromState = getState(obstacleDistanceDiscretizer.getMin() + 0.5 * obstacleDistanceDiscretizer.getBinWidth());
		assertEquals(PRIOR_COLLISION_PROBABILITY_OTHERWISE, model.getCollisionProbability(fromState, action), 0.001); 
		
		// Agent bumps 10 times after turning left when no obstacle in front.
		// This should update the probability given the farthest obstacle distance.
		fromState = new ModeledBumperState();
		fromState.setCollided(false);
		action = BumperAction.LEFT;
		toState = fromState.afterAction(action);
		toState.setCollided(true);
		for (int i = 0; i < 10; i++) {
			TransitionReward transition = new TransitionReward(fromState, action, toState, -100);
			model.updateModel(transition);
		}
		// again we assume prior parameters alpha = 2 (collision), beta = 20 (no collision) 
		assertEquals(0.367, model.getCollisionProbability(fromState, action), 0.001); 
	}
	
	@Test
	public void get_transitions_from_state_and_action() {
		// Obstacle is close in front, agent is not collided yet and risks forward movement (no previous observations)...
		double obstacleDistanceBefore = obstacleDistanceDiscretizer.getMin() + 0.5*obstacleDistanceDiscretizer.getBinWidth();
		ModeledBumperState fromState = getState(obstacleDistanceBefore);
		fromState.setCollided(false);
		BumperAction action = BumperAction.FORWARD;
		
		// There are two outcomes: either the agent collides and remains in place, or it moves the specified distance forward.
		// That when the outcome is a collision the agent remains in place is of course a simplification. An alternative model is
		// that agent remains in place only if it was already collided and the outcome is that it's still collided. But if it was
		// not collided before and then collides, how much does it move/turn?
		Set<StochasticTransitionReward> transitions = model.getOutgoingTransitions(new StateAction(fromState, action));
		assertEquals(2, transitions.size());
		for (StochasticTransitionReward tr : transitions) {
			assertEquals(fromState, tr.getFromState());
			assertEquals(action, tr.getAction());
			assertEquals(rewardFunction.getReward(tr), tr.getReward(), 0);
			
			ModeledBumperState toState = (ModeledBumperState) tr.getToState();
			if (toState.isCollided()) {
				assertEquals(PRIOR_COLLISION_PROB_WHEN_DRIVING_TOWARDS_NEAR_OBSTACLE, tr.getProbability(), 0.0001);
				assertEquals(obstacleDistanceBefore, toState.getValues()[0], 0);
			} else {
				assertEquals(1 - PRIOR_COLLISION_PROB_WHEN_DRIVING_TOWARDS_NEAR_OBSTACLE, tr.getProbability(), 0.0001);
				assertEquals(obstacleDistanceBefore - BumperAction.DRIVE_DISTANCE_CM, toState.getValues()[0], 0);
			}
		}
	}
	
	@Test
	public void get_transitions_to_non_collision_state() {
		
		// After some action and state, an obstacle is this close in front and the agent isn't collided. What are the 
		// possible states and actions that can lead to this state?
		double obstacleDistanceAfter = obstacleDistanceDiscretizer.getMin() + 0.5*obstacleDistanceDiscretizer.getBinWidth();
		ModeledBumperState toState = getState(obstacleDistanceAfter);
		toState.setCollided(false);
		
		// All actions can lead to toState in different ways from different fromStates. There's two transitions for each
		// state, one from a non-collided state, one from a collided state (agent gets free). In both cases the obstacle
		// has moved deterministically relative to the agent according to the action.
		Set<StochasticTransitionReward> transitions = model.getIncomingTransitions(toState);
		assertEquals(8, transitions.size());
		
		StochasticTransitionReward forwardFromNonCollidedState = null;
		StochasticTransitionReward forwardFromCollidedState = null;
		StochasticTransitionReward backwardFromNonCollidedState = null;
		StochasticTransitionReward backwardFromCollidedState = null;
		StochasticTransitionReward rightFromNonCollidedState = null;
		StochasticTransitionReward rightFromCollidedState = null;
		StochasticTransitionReward leftFromNonCollidedState = null;
		StochasticTransitionReward leftFromCollidedState = null;
		
		for (StochasticTransitionReward tr : transitions) {
			assertEquals(toState, tr.getToState());
			assertEquals(rewardFunction.getReward(tr), tr.getReward(), 0);
			
			ModeledBumperState fromState = (ModeledBumperState) tr.getFromState();
			BumperAction action = (BumperAction) tr.getAction();
			
			if (action == BumperAction.FORWARD && !fromState.isCollided()) {
				
				forwardFromNonCollidedState = tr;
				double expectedObstacleDistanceBefore = BumperAction.DRIVE_DISTANCE_CM + obstacleDistanceAfter;
				assertEquals(expectedObstacleDistanceBefore, fromState.getObjectInSectorDegree(0).getDistance(), 0.0001);
				assertTrue(expectedObstacleDistanceBefore < obstacleDistanceDiscretizer.getMin() + obstacleDistanceDiscretizer.getBinWidth());
				assertEquals(1 - PRIOR_COLLISION_PROB_WHEN_DRIVING_TOWARDS_NEAR_OBSTACLE, tr.getProbability(), 0.0001); // Because distance before was also close to obstacle 
				
			} else if (action == BumperAction.FORWARD && fromState.isCollided()) {
				
				forwardFromCollidedState = tr;
				// Same as when not collided since our _prior_ probability does not distinguish between starting from non-collided and collided states
				double expectedObstacleDistanceBefore = BumperAction.DRIVE_DISTANCE_CM + obstacleDistanceAfter;
				assertEquals(expectedObstacleDistanceBefore, fromState.getObjectInSectorDegree(0).getDistance(), 0.0001);
				assertEquals(1 - PRIOR_COLLISION_PROB_WHEN_DRIVING_TOWARDS_NEAR_OBSTACLE, tr.getProbability(), 0.0001); 
				
			} else if (action == BumperAction.BACKWARD && !fromState.isCollided()) {
				
				backwardFromNonCollidedState = tr;
				double expectedObstacleDistanceBefore = obstacleDistanceAfter - BumperAction.DRIVE_DISTANCE_CM;
				assertEquals(expectedObstacleDistanceBefore, fromState.getObjectInSectorDegree(0).getDistance(), 0.0001);
				assertEquals(1 - PRIOR_COLLISION_PROBABILITY_OTHERWISE, tr.getProbability(), 0.0001); 
				
			} else if (action == BumperAction.BACKWARD && fromState.isCollided()) {
				
				backwardFromCollidedState = tr;
				double expectedObstacleDistanceBefore = obstacleDistanceAfter - BumperAction.DRIVE_DISTANCE_CM;
				assertEquals(expectedObstacleDistanceBefore, fromState.getObjectInSectorDegree(0).getDistance(), 0.0001);
				assertEquals(1 - PRIOR_COLLISION_PROBABILITY_OTHERWISE, tr.getProbability(), 0.0001); 
				
			} else if (action == BumperAction.RIGHT && !fromState.isCollided()) {
				
				rightFromNonCollidedState = tr;
				double expectedObstacleDistanceBefore = obstacleDistanceAfter;
				// Floating point inaccuracy in to rad -> degree conversion and the fact that this turn degrees moves the object on the border of 
				// sectors causes it to be in the wrong sector. Foesn't really matter in this case.
				assertEquals(expectedObstacleDistanceBefore, fromState.getObjectInSectorDegree(BumperAction.TURN_DEGREES - 0.00001).getDistance(), 0.0001);
				assertEquals(1 - PRIOR_COLLISION_PROBABILITY_OTHERWISE, tr.getProbability(), 0.0001); 
				
			} else if (action == BumperAction.RIGHT && fromState.isCollided()) {
				
				rightFromCollidedState = tr;
				double expectedObstacleDistanceBefore = obstacleDistanceAfter;
				// Floating point inaccuracy in to rad -> degree conversion and the fact that this turn degrees moves the object on the border of 
				// sectors causes it to be in the wrong sector. Foesn't really matter in this case.
				assertEquals(expectedObstacleDistanceBefore, fromState.getObjectInSectorDegree(BumperAction.TURN_DEGREES - 0.00001).getDistance(), 0.0001);
				assertEquals(1 - PRIOR_COLLISION_PROBABILITY_OTHERWISE, tr.getProbability(), 0.0001); 
				
			} else if (action == BumperAction.LEFT && !fromState.isCollided()) {
				
				leftFromNonCollidedState = tr;
				double expectedObstacleDistanceBefore = obstacleDistanceAfter;
				assertEquals(expectedObstacleDistanceBefore, fromState.getObjectInSectorDegree(-BumperAction.TURN_DEGREES).getDistance(), 0.0001);
				assertEquals(1 - PRIOR_COLLISION_PROBABILITY_OTHERWISE, tr.getProbability(), 0.0001); 
				
			} else if (action == BumperAction.LEFT && fromState.isCollided()) {
				
				leftFromCollidedState = tr;
				double expectedObstacleDistanceBefore = obstacleDistanceAfter;
				assertEquals(expectedObstacleDistanceBefore, fromState.getObjectInSectorDegree(-BumperAction.TURN_DEGREES).getDistance(), 0.0001);
				assertEquals(1 - PRIOR_COLLISION_PROBABILITY_OTHERWISE, tr.getProbability(), 0.0001); 
				
			} 
		}

		assertNotNull(forwardFromNonCollidedState);
		assertNotNull(forwardFromCollidedState);
		assertNotNull(backwardFromNonCollidedState);
		assertNotNull(backwardFromCollidedState);
		assertNotNull(rightFromNonCollidedState);
		assertNotNull(rightFromCollidedState);
		assertNotNull(leftFromNonCollidedState);
		assertNotNull(leftFromCollidedState);
	}
	
	@Test
	public void get_transitions_to_collision_state() {
		
		// After some action and state, an obstacle is this close in front and the agent is collided.
		double obstacleDistanceAfter = obstacleDistanceDiscretizer.getMin() + 0.5*obstacleDistanceDiscretizer.getBinWidth();
		ModeledBumperState toState = getState(obstacleDistanceAfter);
		toState.setCollided(true);
		
		// All actions can lead to toState in different ways from different fromStates. 
		// As with transition to non-collision state, there's two transitions for each
		// state, one from a non-collided state, one from a collided state. 
		// The difference is, now that a collision had happened, we model that fromState = toState, 
		// i.e. agent hasn't moved!
		Set<StochasticTransitionReward> transitions = model.getIncomingTransitions(toState);
		assertEquals(8, transitions.size());
		
		StochasticTransitionReward forwardFromNonCollidedState = null;
		StochasticTransitionReward forwardFromCollidedState = null;
		StochasticTransitionReward backwardFromNonCollidedState = null;
		StochasticTransitionReward backwardFromCollidedState = null;
		StochasticTransitionReward rightFromNonCollidedState = null;
		StochasticTransitionReward rightFromCollidedState = null;
		StochasticTransitionReward leftFromNonCollidedState = null;
		StochasticTransitionReward leftFromCollidedState = null;
		
		for (StochasticTransitionReward tr : transitions) {
			assertEquals(toState, tr.getToState());
			assertEquals(rewardFunction.getReward(tr), tr.getReward(), 0);
			
			ModeledBumperState fromState = (ModeledBumperState) tr.getFromState();
			assertEquals(obstacleDistanceAfter, fromState.getObjectInSectorDegree(0).getDistance(), 0); // agent didn't move
			BumperAction action = (BumperAction) tr.getAction();
			
			if (action == BumperAction.FORWARD && !fromState.isCollided()) {
				
				forwardFromNonCollidedState = tr;
				assertEquals(PRIOR_COLLISION_PROB_WHEN_DRIVING_TOWARDS_NEAR_OBSTACLE, tr.getProbability(), 0.0001);
				
			} else if (action == BumperAction.FORWARD && fromState.isCollided()) {
				
				forwardFromCollidedState = tr;
				assertEquals(PRIOR_COLLISION_PROB_WHEN_DRIVING_TOWARDS_NEAR_OBSTACLE, tr.getProbability(), 0.0001); 
				
			} else if (action == BumperAction.BACKWARD && !fromState.isCollided()) {
				
				backwardFromNonCollidedState = tr;
				assertEquals(PRIOR_COLLISION_PROBABILITY_OTHERWISE, tr.getProbability(), 0.0001); 
				
			} else if (action == BumperAction.BACKWARD && fromState.isCollided()) {
				
				backwardFromCollidedState = tr;
				assertEquals(PRIOR_COLLISION_PROBABILITY_OTHERWISE, tr.getProbability(), 0.0001); 
				
			} else if (action == BumperAction.RIGHT && !fromState.isCollided()) {
				
				rightFromNonCollidedState = tr;
				assertEquals(PRIOR_COLLISION_PROBABILITY_OTHERWISE, tr.getProbability(), 0.0001); 
				
			} else if (action == BumperAction.RIGHT && fromState.isCollided()) {
				
				rightFromCollidedState = tr;
				assertEquals(PRIOR_COLLISION_PROBABILITY_OTHERWISE, tr.getProbability(), 0.0001); 
				
			} else if (action == BumperAction.LEFT && !fromState.isCollided()) {
				
				leftFromNonCollidedState = tr;
				assertEquals(PRIOR_COLLISION_PROBABILITY_OTHERWISE, tr.getProbability(), 0.0001); 
				
			} else if (action == BumperAction.LEFT && fromState.isCollided()) {
				
				leftFromCollidedState = tr;
				assertEquals(PRIOR_COLLISION_PROBABILITY_OTHERWISE, tr.getProbability(), 0.0001); 
				
			} 
		}

		assertNotNull(forwardFromNonCollidedState);
		assertNotNull(forwardFromCollidedState);
		assertNotNull(backwardFromNonCollidedState);
		assertNotNull(backwardFromCollidedState);
		assertNotNull(rightFromNonCollidedState);
		assertNotNull(rightFromCollidedState);
		assertNotNull(leftFromNonCollidedState);
		assertNotNull(leftFromCollidedState);
	}
	
	private ModeledBumperState getState(double obstacleDistanceForward) {
		ModeledBumperState state = new ModeledBumperState();
		state.addObservation(TrackedObject.inPolarDegreeCoordinates(obstacleDistanceForward, 0));
		return state;
	}
}
