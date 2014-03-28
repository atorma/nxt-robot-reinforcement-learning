package org.atorma.robot.objecttrackingbumper.montecarlo;

import java.io.File;
import java.util.*;

import org.atorma.robot.DiscreteRobotController;
import org.atorma.robot.learning.ArrayQTable;
import org.atorma.robot.learning.QTable;
import org.atorma.robot.learning.montecarlo.FirstVisitOnPolicyMonteCarlo;
import org.atorma.robot.logging.CsvLogWriter;
import org.atorma.robot.mdp.Transition;
import org.atorma.robot.mdp.TransitionReward;
import org.atorma.robot.objecttracking.TrackedObject;
import org.atorma.robot.objecttrackingbumper.*;
import org.atorma.robot.objecttrackingbumper.prioritizedsweeping.BumperModel;
import org.atorma.robot.policy.EpsilonGreedyPolicy;
import org.atorma.robot.simplebumper.BumperAction;
import org.atorma.robot.simplebumper.BumperPercept;
import org.paukov.combinatorics.*;

public class ObjectTrackingMonteCarloBumper implements DiscreteRobotController {
	
	private QTable qTable;
	private BumperModel model;
	private EpsilonGreedyPolicy epsilonGreedyPolicy;
	private BumperRewardFunction rewardFunction = new BumperRewardFunction();
	
	private FirstVisitOnPolicyMonteCarlo monteCarlo;
	private double discountFactor = 0.9;
	private BumperStateDiscretizer stateDiscretizer = new BumperStateDiscretizer();

	private ModeledBumperState previousState;
	private BumperAction previousAction;
	private Queue<TransitionReward> observedTransitions = new LinkedList<>();
	
	private double accumulatedReward = 0;
	private int accumulatedCollisions = 0;
	
	private CsvLogWriter logWriter;
	
	private volatile boolean actionRequested;
	
	
	public ObjectTrackingMonteCarloBumper(String logFile) {
		this();
		logWriter = new CsvLogWriter(new File(logFile), "Accumulated reward", "Accumulated collisions"); 
	}
	
	public ObjectTrackingMonteCarloBumper() {
		qTable = new ArrayQTable(stateDiscretizer.getNumberOfStates(), BumperAction.values().length);
		model = new BumperModel(rewardFunction, stateDiscretizer);
		//setPriorCollisionProbabilities(0.8, 0.99);
		epsilonGreedyPolicy = new EpsilonGreedyPolicy(0.1, qTable, BumperAction.values());
		monteCarlo = new FirstVisitOnPolicyMonteCarlo(model, stateDiscretizer, epsilonGreedyPolicy, qTable, 10, discountFactor);
		
		Thread sweeperThread = new Thread(new Sweeper());
		sweeperThread.start();
	}
	
	/**
	 * Sets up prior collision probabilities for all states where there's an obstacle close in front or close behind
	 * and the agent moves towards it.
	 * 
	 * @param collisionProbDrivingTowardObstacle
	 * 	collision probability when the agent starts from non-collision state
	 * @param collisionProbWhenAlreadyBumped
	 * 	collision probability when the agent is already collided and still moves toward the obstacle in front/behind
	 */
	private void setPriorCollisionProbabilities(double collisionProbDrivingTowardObstacle, double collisionProbWhenAlreadyBumped) {
		int priorSamplesNotYetCollided = (int) Math.ceil((10 * collisionProbDrivingTowardObstacle - 1)/(1 - collisionProbDrivingTowardObstacle));
		int priorSamplesTwoCollisions = (int) Math.ceil((10 * collisionProbWhenAlreadyBumped - 1)/(1 - collisionProbWhenAlreadyBumped));
		
		// Generator for possible distance permutations in 5 sectors
		Double[] distances = new Double[stateDiscretizer.getNumberOfDistanceBins()];
		for (int i = 0; i < stateDiscretizer.getNumberOfDistanceBins(); i++) {
			distances[i] = stateDiscretizer.getMinDistance() + (i + 0.5)*stateDiscretizer.getDistanceBinWidth();
		}
		Generator<Double> permutationGen = Factory.createPermutationWithRepetitionGenerator(Factory.createVector(distances), stateDiscretizer.getNumberOfSectors() - 1);
		
		// Add collisions when an obstacle is close in front and the agent drives forward.
		// Do this for all distance permutations in the other sectors. Use more samples to
		// express more confidence in collision when the agent was already collided.
		// Repeat for reversing towards obstacle behind.
		for (ICombinatoricsVector<Double> perm : permutationGen) {
			for (boolean alreadyCollided : Arrays.asList(false, true)) {
				for (BumperAction action : Arrays.asList(BumperAction.FORWARD, BumperAction.BACKWARD)) {
					
					double obstacleDirectionDegrees = -1;
					if (action == BumperAction.FORWARD) {
						obstacleDirectionDegrees = 0;
					} else if (action == BumperAction.BACKWARD) {
						obstacleDirectionDegrees = 180;
					}
					
					ModeledBumperState fromState = new ModeledBumperState();
					fromState.setCollided(alreadyCollided);
					fromState.addObservation(TrackedObject.inPolarDegreeCoordinates(BumperAction.DRIVE_DISTANCE_CM, obstacleDirectionDegrees));
					
					int i = 0;
					for (double sectorDegree = 0; sectorDegree < 360; sectorDegree += stateDiscretizer.getSectorWidthDegrees() ) {
						if (sectorDegree != obstacleDirectionDegrees) {
							fromState.addObservation(TrackedObject.inPolarDegreeCoordinates(perm.getValue(i), sectorDegree));
							i++;
						}
					}
					
					ModeledBumperState toState = fromState.afterAction(action);
					toState.setCollided(true);
					
					int priorSamples = alreadyCollided ? priorSamplesTwoCollisions : priorSamplesNotYetCollided;
					for (int s = 0; s < priorSamples; s++) {
						TransitionReward transition = new TransitionReward(fromState, action, toState, -100); // the reward doesn't matter in this implementation
						model.update(transition);
					}
				}
			}
		}
	}
	
	
	@Override
	public int getActionId(double[] currentPerceptValues) {
		actionRequested = true;
		
		BumperPercept currentPercept = new BumperPercept(currentPerceptValues);
		if (currentPercept.isCollided()) {
			accumulatedCollisions++;
			//model.printCollisionProbabilities();
		}
		
		ModeledBumperState currentState;
		if (previousAction != null) {
			currentState = previousState.afterActionAndObservation(previousAction, currentPercept);
		} else {
			currentState = ModeledBumperState.initialize(currentPercept);
		}
		int currentStateId = stateDiscretizer.getId(currentState);
		
		TransitionReward transitionReward = null;
		if (previousAction != null) {
			Transition transition = new Transition(previousState, previousAction, currentState);
			double reward = rewardFunction.getReward(transition);
			accumulatedReward += reward;
			transitionReward = new TransitionReward(transition, reward);
		}
		
		BumperAction action;
		synchronized (monteCarlo) { // synchronize on observedTransitions, qTable, monteCarlo
			
			if (transitionReward != null) {
				observedTransitions.add(transitionReward);
			}

			action = BumperAction.getAction(epsilonGreedyPolicy.getActionId(currentStateId));
			actionRequested = false;

			monteCarlo.setRolloutStartState(currentState);
			monteCarlo.notify();
		}
		
		previousState = currentState;
		previousAction = action;
		
		if (logWriter != null) {
			logWriter.addRow(accumulatedReward, accumulatedCollisions);
		}
		
		return action.getId();
	}
	
	private class Sweeper implements Runnable {

		@Override
		public void run() {
			int sweepsBetweenObservations = 0;
			
			while (true) {
				synchronized (monteCarlo) {
					
					while (actionRequested) {
						try {
							monteCarlo.wait();
						} catch (InterruptedException e) {}
					}
					
					TransitionReward transitionReward = observedTransitions.poll();
					if (transitionReward != null) {
						System.out.println("sweeps between observations " + sweepsBetweenObservations);
						sweepsBetweenObservations = 0;
						model.update(transitionReward);
					}
					monteCarlo.performRollouts(1);
					sweepsBetweenObservations += 1;
					
				}
			}
			
		}
		
	}

}
