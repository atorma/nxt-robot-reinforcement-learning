package org.atorma.robot.objecttrackingbumper.prioritizedsweeping;

import static org.junit.Assert.assertEquals;

import java.util.Set;

import org.atorma.robot.mdp.*;
import org.atorma.robot.objecttracking.TrackedObject;
import org.atorma.robot.objecttrackingbumper.ObstacleDistanceDiscretizer;
import org.atorma.robot.simplebumper.BumperAction;
import org.junit.Before;
import org.junit.Test;

public class BumperModelTests {

	private static final double PRIOR_COLLISION_PROB_WHEN_DRIVING_TOWARDS_NEAR_OBSTACLE = 0.8;
	private static final double PRIOR_COLLISION_PROBABILITY_OTHERWISE = 0.1;

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
		
	}
	
	@Test
	public void collision_probability_is_updated_after_observations() {
		// Agent bumps 10 times after turning right when obstacle is close in front
		ModeledBumperState fromState = getState(obstacleDistanceDiscretizer.getMin() + 0.5 * obstacleDistanceDiscretizer.getBinWidth());
		BumperAction action = BumperAction.RIGHT;
		ModeledBumperState toState = fromState.afterAction(action);
		toState.setCollided(true);
		for (int i = 0; i < 10; i++) {
			TransitionReward transition = new TransitionReward(fromState, action, toState, -100);
			model.updateModel(transition);
		}
		// here we assume prior parameters alpha = 2 (collision), beta = 10 (no collision) 
		assertEquals(0.55, model.getCollisionProbability(fromState, action), 0.0001); 
	}
	
	@Test
	public void get_transitions_from_state_and_action() {
		// Obstacle is close in front and the agent risks forward movement (no previous observations)...
		double obstacleDistanceBefore = obstacleDistanceDiscretizer.getMin() + 0.5 * obstacleDistanceDiscretizer.getBinWidth();
		ModeledBumperState state = getState(obstacleDistanceBefore);
		BumperAction action = BumperAction.FORWARD;
		
		// There are two outcomes: either the agent collides and remains in place, or it moves the specified distance forward 
		Set<StochasticTransitionReward> transitions = model.getOutgoingTransitions(new StateAction(state, action));
		assertEquals(2, transitions.size());
		for (StochasticTransitionReward tr : transitions) {
			assertEquals(state, tr.getFromState());
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
	
	private ModeledBumperState getState(double obstacleDistanceForward) {
		ModeledBumperState state = new ModeledBumperState();
		state.addObservation(TrackedObject.inPolarDegreeCoordinates(obstacleDistanceForward, 0));
		return state;
	}
}
