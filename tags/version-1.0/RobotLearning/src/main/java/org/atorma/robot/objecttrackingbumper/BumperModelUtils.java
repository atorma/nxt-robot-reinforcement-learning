package org.atorma.robot.objecttrackingbumper;

import java.util.Arrays;

import org.atorma.robot.mdp.DiscretizedStateAction;
import org.atorma.robot.objecttracking.CircleSector;
import org.atorma.robot.objecttracking.TrackedObject;
import org.atorma.robot.simplebumper.BumperAction;
import org.paukov.combinatorics.*;

public class BumperModelUtils {

	/**
	 * Sets up prior collision probabilities for all states where there's an obstacle close in front or close behind
	 * and the agent moves towards it.
	 * 
	 * @param model
	 * 	the model to set up
	 * @param stateDiscretizer
	 * 	discretizer of states for collision probabilities
	 * @param collisionProbDrivingTowardObstacle
	 * 	collision probability when the agent starts from non-collision state
	 * @param collisionProbWhenAlreadyBumped
	 * 	collision probability when the agent is already collided and still moves toward the obstacle in front/behind
	 */
	public static void setPriorCollisionProbabilities(BumperModel model, BumperStateDiscretizer stateDiscretizer, double collisionProbDrivingTowardObstacle, double collisionProbWhenAlreadyBumped) {
		
		double priorParamNotYetCollided = getCollisionPriorParam(model, collisionProbDrivingTowardObstacle);
		double priorParamTwoCollisions = getCollisionPriorParam(model, collisionProbWhenAlreadyBumped);
		
		// Generator for distance permutations in the other sectors than the one with the obstacle that the robot is driving towards
		Double[] distances = new Double[stateDiscretizer.getNumberOfDistanceBins()];
		for (int i = 0; i < stateDiscretizer.getNumberOfDistanceBins(); i++) {
			distances[i] = stateDiscretizer.getMinDistance() + (i + 0.5)*stateDiscretizer.getDistanceBinWidth();
		}
		Generator<Double> permutationGen = Factory.createPermutationWithRepetitionGenerator(Factory.createVector(distances), stateDiscretizer.getNumberOfSectors() - 1);
		
		// Set prior parameters of collision probability when an obstacle is close in front and the agent drives forward.
		// Do this for all distance permutations in the other sectors. Use more samples to
		// express more confidence in collision when the agent was already collided.
		// Repeat for reversing towards obstacle behind.	
		for (BumperAction action : Arrays.asList(BumperAction.FORWARD, BumperAction.BACKWARD)) {
			for (boolean alreadyCollided : Arrays.asList(false, true)) {
			
				double obstacleDirectionDegrees = -1;
				if (action == BumperAction.FORWARD) {
					obstacleDirectionDegrees = 0;
				} else if (action == BumperAction.BACKWARD) {
					obstacleDirectionDegrees = 180;
				}
				
				// Make sure the obstacle is within a collision tracking sector.
				// Otherwise we'll teach that collision is likely even if no obstacle in sight!
				boolean found = false;
				for (CircleSector sector : stateDiscretizer.getSectors()) {
					if (sector.contains(obstacleDirectionDegrees)) {
						found = true;
					}
				}
				if (!found) {
					continue;
				}

				// For each state permutation, train that it's bad to move towards the near obstacle
				for (ICombinatoricsVector<Double> perm : permutationGen) {
					
					// Add the obstacle in the sector the robot is moving towards. This is fixed in all state permutations. 
					ModeledBumperState fromState = new ModeledBumperState();
					fromState.setCollided(alreadyCollided);
					fromState.addObservation(TrackedObject.inPolarDegreeCoordinates(BumperAction.DRIVE_DISTANCE_CM, obstacleDirectionDegrees));
					
					// Set distances in other sectors according to current permutation
					int i = 0;
					for (CircleSector sector : stateDiscretizer.getSectors()) {
						if (!sector.contains(obstacleDirectionDegrees)) { // this sector was fixed above
							fromState.addObservation(TrackedObject.inPolarDegreeCoordinates(perm.getValue(i), sector.getMidAngleDeg()));
							i++;
						}
					}
					
					DiscretizedStateAction sa = new DiscretizedStateAction(stateDiscretizer.getId(fromState), action.getId());
					if (alreadyCollided) {
						model.setCollisionProbabilityPrior(sa, priorParamTwoCollisions, model.getDefaultPriorParamNoCollision());
					} else {
						model.setCollisionProbabilityPrior(sa, priorParamNotYetCollided, model.getDefaultPriorParamNoCollision());
					}
					
					/*
					ModeledBumperState toState = fromState.afterAction(action);
					toState.setCollided(true);
					
					int priorSamples = alreadyCollided ? priorParamTwoCollisions : priorParamNotYetCollided;
					for (int s = 0; s < priorSamples; s++) {
						TransitionReward transition = new TransitionReward(fromState, action, toState, -100); // the reward doesn't matter in this implementation
						model.update(transition);
					}
					*/
					
				}
				
			}
		}
	}
	
	private static double getCollisionPriorParam(BumperModel model, double collisionProb) {
		double b = model.getDefaultPriorParamNoCollision();
		double p = collisionProb;
		double a = ( p*(b - 2) + 1 )/( 1 - p );
		return a;
	}
}
