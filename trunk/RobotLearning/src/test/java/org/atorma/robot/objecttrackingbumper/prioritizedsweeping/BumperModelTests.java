package org.atorma.robot.objecttrackingbumper.prioritizedsweeping;

import static org.junit.Assert.assertEquals;

import org.atorma.robot.mdp.StateAction;
import org.atorma.robot.mdp.StochasticTransitionReward;
import org.atorma.robot.objecttracking.TrackedObject;
import org.atorma.robot.objecttrackingbumper.ObstacleDistanceDiscretizer;
import org.atorma.robot.simplebumper.BumperAction;
import org.junit.Before;
import org.junit.Test;

public class BumperModelTests {

	private BumperModel model;
	private ObstacleDistanceDiscretizer obstacleDistanceDiscretizer = new ObstacleDistanceDiscretizer();
	
	@Before
	public void setUp() {
		model = new BumperModel(new BumperRewardFunction(), obstacleDistanceDiscretizer); 
	}
	
	@Test
	public void test_bayesian_collision_probability_prior() {
		
		for (int distanceBin = 0; distanceBin < obstacleDistanceDiscretizer.getNumberOfBins(); distanceBin++) {
			double distance = (distanceBin + 0.5) * obstacleDistanceDiscretizer.getBinWidth() + obstacleDistanceDiscretizer.getMin();
			for (BumperAction action : BumperAction.values()) {
				System.out.println(distance + " " + action);			
				for (StochasticTransitionReward tr : model.getOutgoingTransitions(getStateAction(distance, action))) {
					ModeledBumperState toState = (ModeledBumperState) tr.getToState();
					
					
					if (distanceBin == 0 && action == BumperAction.FORWARD) {
						// Nearest distance has big collision probability (0.8) for forward action.
						if (toState.isCollided()) {
							assertEquals(0.8, tr.getProbability(), 0.0001);
						} else {
							assertEquals(0.2, tr.getProbability(), 0.0001);
						}
						
					} else {
						// In all other cases the collision probability is small (0.1).
						if (toState.isCollided()) {
							assertEquals(0.1, tr.getProbability(), 0.0001);
						} else {
							assertEquals(0.9, tr.getProbability(), 0.0001);
						}
						
					}
				}
			}
		}
		
	}
	
	private StateAction getStateAction(double obstacleDistanceForward, BumperAction action) {
		ModeledBumperState state = new ModeledBumperState();
		state.addObservation(TrackedObject.inPolarDegreeCoordinates(obstacleDistanceForward, 0));
		return new StateAction(state, action);
	}
}
