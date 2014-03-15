package org.atorma.robot.objecttrackingbumper.prioritizedsweeping;

import static org.junit.Assert.*;

import java.util.Arrays;
import java.util.Set;

import org.atorma.robot.mdp.*;
import org.atorma.robot.objecttracking.TrackedObject;
import org.atorma.robot.objecttrackingbumper.*;
import org.atorma.robot.simplebumper.BumperAction;
import org.junit.*;
import org.paukov.combinatorics.*;

public class BumperModelTests {

	private static final int PRIOR_SAMPLES_WHEN_NOT_YET_COLLIDED = 35;
	private static final int PRIOR_SAMPLES_WHEN_ALREADY_COLLIDED = 890;
	
	// Expected prior collision probabilities given our prior samples
	// Computed using binomial collision model with beta prior, where the prior
	// parameters are 2 for collision and 10 for no collision.
	private static final double PRIOR_COLLISION_PROB_WHEN_MOVING_TOWARD_NEAR_OBSTACLE = 0.80;
	private static final double PRIOR_COLLISION_PROB_WHEN_MOVING_TOWARD_OBSTACLE_AFTER_ALREADY_COLLIDED = 0.99;
	private static final double PRIOR_COLLISION_PROBABILITY_OTHERWISE = 0.1;

	private BumperModel model;
	
	// Here we use the same state discretization as for action value learning. It considers
	// obstacles in 6 sectors with 4 distance intervals each and whether the agent is already
	// collided or not.
	private BumperStateDiscretizer discretizer = new BumperStateDiscretizer();
	private BumperRewardFunction rewardFunction = new BumperRewardFunction();
	
	
	@Before
	public void setUp() {
		model = new BumperModel(rewardFunction, discretizer); 
		addSimplePriorCollisionSamples();
	}

	// Here we train the collision model just for those states where there's a single obstacle 
	// and it's located directly in front of the agent and is near. This allows tests to run fast!
	private void addSimplePriorCollisionSamples() {
		ModeledBumperState fromState, toState;
		BumperAction action;
		
		// Add collisions when an obstacle is close in front, not collision yet, and the agent drives forward
		fromState = new ModeledBumperState();
		fromState.addObservation(TrackedObject.inPolarDegreeCoordinates(BumperAction.DRIVE_DISTANCE_CM, 0));
		fromState.setCollided(false);
		action = BumperAction.FORWARD;
		toState = fromState.afterAction(action);
		toState.setCollided(true);
		for (int i = 0; i < PRIOR_SAMPLES_WHEN_NOT_YET_COLLIDED; i++) {
			TransitionReward transition = new TransitionReward(fromState, action, toState, -100); // the reward doesn't matter in this implementation
			model.updateModel(transition);
		}

		// Same when already collided before starting the action
		fromState.setCollided(true);
		toState = fromState;
		for (int i = 0; i < PRIOR_SAMPLES_WHEN_ALREADY_COLLIDED; i++) {
			TransitionReward transition = new TransitionReward(fromState, action, toState, -100); 
			model.updateModel(transition);
		}
	}
	
	// Here we train the collision model so that for all obstacle combinations where there's an
	// obstacle close to in front and the agent drives forward, or close behind and the agent drives
	// backwards, there's a high collision probability. Due to combinatorics, this is fairly slow for
	// continuous testing.
	private void addFullPriorCollisionSamples() {
		// Generator for possible distance permutations in 5 sectors
		Double[] distances = new Double[discretizer.getNumberOfDistanceBins()];
		for (int i = 0; i < discretizer.getNumberOfDistanceBins(); i++) {
			distances[i] = discretizer.getMinDistance() + (i + 0.5)*discretizer.getDistanceBinWidth();
		}
		Generator<Double> permutationGen = Factory.createPermutationWithRepetitionGenerator(Factory.createVector(distances), discretizer.getNumberOfSectors() - 1);
		
		// Add collisions when an obstacle is close in front and the agent drives forward.
		// Do this for all distance permutations in the other sectors. Use more samples to
		// express more confidence in collision when the agent was already collided.
		for (ICombinatoricsVector<Double> perm : permutationGen) {
			for (boolean alreadyCollided : Arrays.asList(false, true)) {
				ModeledBumperState fromState = new ModeledBumperState();
				fromState.addObservation(TrackedObject.inPolarDegreeCoordinates(BumperAction.DRIVE_DISTANCE_CM, 0));
				fromState.setCollided(alreadyCollided);
				
				int i = 0;
				for (double sectorDegree = 0; sectorDegree < 360; sectorDegree += discretizer.getSectorWidthDegrees() ) {
					if (sectorDegree != 0) {
						fromState.addObservation(TrackedObject.inPolarDegreeCoordinates(perm.getValue(i), sectorDegree));
						i++;
					}
				}
				
				BumperAction action = BumperAction.FORWARD;
				ModeledBumperState toState = fromState.afterAction(action);
				toState.setCollided(true);
				
				int priorSamples = alreadyCollided ? PRIOR_SAMPLES_WHEN_ALREADY_COLLIDED : PRIOR_SAMPLES_WHEN_NOT_YET_COLLIDED;
				for (int s = 0; s < priorSamples; s++) {
					TransitionReward transition = new TransitionReward(fromState, action, toState, -100); // the reward doesn't matter in this implementation
					model.updateModel(transition);
				}
			}
		}
		
		// Same when moving backwards toward an obstacle behind
		for (ICombinatoricsVector<Double> perm : permutationGen) {
			for (boolean alreadyCollided : Arrays.asList(false, true)) {
				ModeledBumperState fromState = new ModeledBumperState();
				fromState.addObservation(TrackedObject.inPolarDegreeCoordinates(BumperAction.DRIVE_DISTANCE_CM, 180));
				fromState.setCollided(alreadyCollided);
				
				int i = 0;
				for (double sectorDegree = 0; sectorDegree < 360; sectorDegree += discretizer.getSectorWidthDegrees() ) {
					if (sectorDegree != 180) {
						fromState.addObservation(TrackedObject.inPolarDegreeCoordinates(perm.getValue(i), sectorDegree));
						i++;
					}
				}
				
				BumperAction action = BumperAction.BACKWARD;
				ModeledBumperState toState = fromState.afterAction(action);
				toState.setCollided(true);
				
				int priorSamples = alreadyCollided ? PRIOR_SAMPLES_WHEN_ALREADY_COLLIDED : PRIOR_SAMPLES_WHEN_NOT_YET_COLLIDED;
				for (int s = 0; s < priorSamples; s++) {
					TransitionReward transition = new TransitionReward(fromState, action, toState, -100); // the reward doesn't matter in this implementation
					model.updateModel(transition);
				}
			}
		}
	}
	
	@Test
	public void test_prior_collision_probability_when_no_knowledge_of_collisions() {
		model = new BumperModel(rewardFunction, discretizer); 
		
		for (int stateId = 0; stateId < discretizer.getNumberOfStates(); stateId++) {
			for (BumperAction action : BumperAction.values()) {	
				assertEquals(PRIOR_COLLISION_PROBABILITY_OTHERWISE, model.getCollisionProbability(stateId, action.getId()), 0.0001);
			}
		}
	}

	@Test
	public void collision_probability_is_updated_after_prior_samples() {
		ModeledBumperState fromState = getState(discretizer.getMinDistance() + 0.5 * discretizer.getDistanceBinWidth());
		fromState.setCollided(false);
		BumperAction action = BumperAction.FORWARD;

		assertEquals(0.8, model.getCollisionProbability(fromState, action), 0.1); 
		assertEquals(PRIOR_COLLISION_PROB_WHEN_MOVING_TOWARD_NEAR_OBSTACLE, model.getCollisionProbability(fromState, action), 0.0001);
		
		fromState.setCollided(true);
		assertEquals(0.95, model.getCollisionProbability(fromState, action), 0.1); 
		assertEquals(PRIOR_COLLISION_PROB_WHEN_MOVING_TOWARD_OBSTACLE_AFTER_ALREADY_COLLIDED, model.getCollisionProbability(fromState, action), 0.0001);
	}
	
	@Test @Ignore // Slow test
	public void test_collision_probabilities_with_full_prior_samples() {
		ModeledBumperState fromState = getState(BumperAction.DRIVE_DISTANCE_CM);
		fromState.setCollided(false);
		BumperAction action = BumperAction.FORWARD;
		
		// This is why either a good discretization, i.e. state projection, or good prior probability setting 
		// is needed. The state with an obstacle near in front AND near to the left is totally different from 
		// the state with just the obstacle near in front. 
		fromState.addObservation(TrackedObject.inPolarDegreeCoordinates(discretizer.getMinDistance()*2, -90));
		assertEquals(PRIOR_COLLISION_PROBABILITY_OTHERWISE, model.getCollisionProbability(fromState, action), 0.0001); // Not what common sense would say!
		
		// Now with full model training
		model = new BumperModel(rewardFunction, discretizer); 
		addFullPriorCollisionSamples();
		assertEquals(PRIOR_COLLISION_PROB_WHEN_MOVING_TOWARD_NEAR_OBSTACLE, model.getCollisionProbability(fromState, action), 0.0001); 
		
		// driving backwards toward obstacle
		fromState.addObservation(TrackedObject.inPolarDegreeCoordinates(BumperAction.DRIVE_DISTANCE_CM, 180));
		action = BumperAction.BACKWARD;
		assertEquals(PRIOR_COLLISION_PROB_WHEN_MOVING_TOWARD_NEAR_OBSTACLE, model.getCollisionProbability(fromState, action), 0.0001);
	}
	
	@Test
	public void get_transitions_from_state_and_action_when_agent_is_collided() {
		// Obstacle is close in front, agent is already collided, and yet risks forward movement 
		double obstacleDistanceBefore = discretizer.getMinDistance() + 0.5*discretizer.getDistanceBinWidth();
		ModeledBumperState fromState = getState(obstacleDistanceBefore);
		fromState.setCollided(true);
		BumperAction action = BumperAction.FORWARD;
		
		// There are two outcomes: either the agent collides again and thus remains in place, or it moves the specified distance forward.
		Set<StochasticTransitionReward> transitions = model.getOutgoingTransitions(new StateAction(fromState, action));
		assertEquals(2, transitions.size());
		for (StochasticTransitionReward tr : transitions) {
			assertEquals(fromState, tr.getFromState());
			assertEquals(action, tr.getAction());
			assertEquals(rewardFunction.getReward(tr), tr.getReward(), 0);
			
			ModeledBumperState toState = (ModeledBumperState) tr.getToState();
			if (toState.isCollided()) {
				assertEquals(PRIOR_COLLISION_PROB_WHEN_MOVING_TOWARD_OBSTACLE_AFTER_ALREADY_COLLIDED, tr.getProbability(), 0.0001);
				assertEquals(obstacleDistanceBefore, toState.getValues()[0], 0);
			} else {
				assertEquals(1 - PRIOR_COLLISION_PROB_WHEN_MOVING_TOWARD_OBSTACLE_AFTER_ALREADY_COLLIDED, tr.getProbability(), 0.0001);
				assertEquals(obstacleDistanceBefore - BumperAction.DRIVE_DISTANCE_CM, toState.getValues()[0], 0);
			}
		}
	}
	
	@Test
	public void get_transitions_from_state_and_action_when_agent_not_collided() {
		// Obstacle is close in front, agent is NOT collided yet and risks forward movement (no previous observations)...
		double obstacleDistanceBefore = discretizer.getMinDistance() + 0.5*discretizer.getDistanceBinWidth();
		ModeledBumperState fromState = getState(obstacleDistanceBefore);
		fromState.setCollided(false);
		BumperAction action = BumperAction.FORWARD;
		
		// There are two outcomes: either the agent collides or not. In both cases we model that the action happened in
		// its entirety, which is of course a simplification (we don't know how much it actually moved).
		Set<StochasticTransitionReward> transitions = model.getOutgoingTransitions(new StateAction(fromState, action));
		assertEquals(2, transitions.size());
		for (StochasticTransitionReward tr : transitions) {
			assertEquals(fromState, tr.getFromState());
			assertEquals(action, tr.getAction());
			assertEquals(rewardFunction.getReward(tr), tr.getReward(), 0);
			
			ModeledBumperState toState = (ModeledBumperState) tr.getToState();
			assertEquals(obstacleDistanceBefore - BumperAction.DRIVE_DISTANCE_CM, toState.getValues()[0], 0);
			if (toState.isCollided()) {
				assertEquals(PRIOR_COLLISION_PROB_WHEN_MOVING_TOWARD_NEAR_OBSTACLE, tr.getProbability(), 0.0001);
			} else {
				assertEquals(1 - PRIOR_COLLISION_PROB_WHEN_MOVING_TOWARD_NEAR_OBSTACLE, tr.getProbability(), 0.0001);
			}
		}
	}
	
	@Test
	public void get_transitions_to_non_collision_state() {
		
		// After some action and state, an obstacle is this close in front and the agent isn't collided. What are the 
		// possible states and actions that can lead to this state?
		double obstacleDistanceAfter = discretizer.getMinDistance() + 0.5*discretizer.getDistanceBinWidth();
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
				assertEquals(expectedObstacleDistanceBefore, fromState.getObjectInDirectionDegrees(0).getDistance(), 0.0001);
				assertTrue(expectedObstacleDistanceBefore < discretizer.getMinDistance() + discretizer.getDistanceBinWidth());
				assertEquals(1 - PRIOR_COLLISION_PROB_WHEN_MOVING_TOWARD_NEAR_OBSTACLE, tr.getProbability(), 0.0001); // Because distance before was also close to obstacle 
				
			} else if (action == BumperAction.FORWARD && fromState.isCollided()) {
				
				forwardFromCollidedState = tr;
				// Same as when not collided since our _prior_ probability does not distinguish between starting from non-collided and collided states
				double expectedObstacleDistanceBefore = BumperAction.DRIVE_DISTANCE_CM + obstacleDistanceAfter;
				assertEquals(expectedObstacleDistanceBefore, fromState.getObjectInDirectionDegrees(0).getDistance(), 0.0001);
				assertEquals(1 - PRIOR_COLLISION_PROB_WHEN_MOVING_TOWARD_OBSTACLE_AFTER_ALREADY_COLLIDED, tr.getProbability(), 0.0001); 
				
			} else if (action == BumperAction.BACKWARD && !fromState.isCollided()) {
				
				backwardFromNonCollidedState = tr;
				double expectedObstacleDistanceBefore = obstacleDistanceAfter - BumperAction.DRIVE_DISTANCE_CM;
				assertEquals(expectedObstacleDistanceBefore, fromState.getObjectInDirectionDegrees(0).getDistance(), 0.0001);
				assertEquals(1 - PRIOR_COLLISION_PROBABILITY_OTHERWISE, tr.getProbability(), 0.0001); 
				
			} else if (action == BumperAction.BACKWARD && fromState.isCollided()) {
				
				backwardFromCollidedState = tr;
				double expectedObstacleDistanceBefore = obstacleDistanceAfter - BumperAction.DRIVE_DISTANCE_CM;
				assertEquals(expectedObstacleDistanceBefore, fromState.getObjectInDirectionDegrees(0).getDistance(), 0.0001);
				assertEquals(1 - PRIOR_COLLISION_PROBABILITY_OTHERWISE, tr.getProbability(), 0.0001); 
				
			} else if (action == BumperAction.RIGHT && !fromState.isCollided()) {
				
				rightFromNonCollidedState = tr;
				double expectedObstacleDistanceBefore = obstacleDistanceAfter;
				// Floating point inaccuracy in to rad -> degree conversion and the fact that this turn degrees moves the object on the border of 
				// sectors causes it to be in the wrong sector. Foesn't really matter in this case.
				assertEquals(expectedObstacleDistanceBefore, fromState.getObjectInDirectionDegrees(BumperAction.TURN_DEGREES - 0.00001).getDistance(), 0.0001);
				assertEquals(1 - PRIOR_COLLISION_PROBABILITY_OTHERWISE, tr.getProbability(), 0.0001); 
				
			} else if (action == BumperAction.RIGHT && fromState.isCollided()) {
				
				rightFromCollidedState = tr;
				double expectedObstacleDistanceBefore = obstacleDistanceAfter;
				// Floating point inaccuracy in to rad -> degree conversion and the fact that this turn degrees moves the object on the border of 
				// sectors causes it to be in the wrong sector. Foesn't really matter in this case.
				assertEquals(expectedObstacleDistanceBefore, fromState.getObjectInDirectionDegrees(BumperAction.TURN_DEGREES - 0.00001).getDistance(), 0.0001);
				assertEquals(1 - PRIOR_COLLISION_PROBABILITY_OTHERWISE, tr.getProbability(), 0.0001); 
				
			} else if (action == BumperAction.LEFT && !fromState.isCollided()) {
				
				leftFromNonCollidedState = tr;
				double expectedObstacleDistanceBefore = obstacleDistanceAfter;
				assertEquals(expectedObstacleDistanceBefore, fromState.getObjectInDirectionDegrees(-BumperAction.TURN_DEGREES).getDistance(), 0.0001);
				assertEquals(1 - PRIOR_COLLISION_PROBABILITY_OTHERWISE, tr.getProbability(), 0.0001); 
				
			} else if (action == BumperAction.LEFT && fromState.isCollided()) {
				
				leftFromCollidedState = tr;
				double expectedObstacleDistanceBefore = obstacleDistanceAfter;
				assertEquals(expectedObstacleDistanceBefore, fromState.getObjectInDirectionDegrees(-BumperAction.TURN_DEGREES).getDistance(), 0.0001);
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
		double obstacleDistanceAfter = discretizer.getMinDistance() + 0.5*discretizer.getDistanceBinWidth();
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
			assertEquals(obstacleDistanceAfter, fromState.getObjectInDirectionDegrees(0).getDistance(), 0); // agent didn't move
			BumperAction action = (BumperAction) tr.getAction();
			
			if (action == BumperAction.FORWARD && !fromState.isCollided()) {
				
				forwardFromNonCollidedState = tr;
				assertEquals(PRIOR_COLLISION_PROB_WHEN_MOVING_TOWARD_NEAR_OBSTACLE, tr.getProbability(), 0.0001);
				
			} else if (action == BumperAction.FORWARD && fromState.isCollided()) {
				
				forwardFromCollidedState = tr;
				assertEquals(PRIOR_COLLISION_PROB_WHEN_MOVING_TOWARD_OBSTACLE_AFTER_ALREADY_COLLIDED, tr.getProbability(), 0.0001); 
				
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
