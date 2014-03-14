package org.atorma.robot.objecttrackingbumper.prioritizedsweeping;

import java.io.File;
import java.util.LinkedList;
import java.util.Queue;

import org.atorma.robot.DiscreteRobotController;
import org.atorma.robot.discretization.StateDiscretizer;
import org.atorma.robot.learning.ArrayQTable;
import org.atorma.robot.learning.QTable;
import org.atorma.robot.learning.prioritizedsweeping.PrioritizedSweeping;
import org.atorma.robot.logging.CsvLogWriter;
import org.atorma.robot.mdp.*;
import org.atorma.robot.objecttracking.TrackedObject;
import org.atorma.robot.objecttrackingbumper.*;
import org.atorma.robot.policy.*;
import org.atorma.robot.simplebumper.BumperAction;
import org.atorma.robot.simplebumper.BumperPercept;

public class ObjectTrackingPrioritizedSweepingBumper implements DiscreteRobotController {
	
	private StateDiscretizer stateDiscretizer = new BumperStateDiscretizer();
	private BumperRewardFunction rewardFunction = new BumperRewardFunction();
	
	private QTable qTable;
	
	private BumperModel model;
	private StateDiscretizer collisionStateDiscretizer = new SingleSectorCollisionStateDiscretizer(90);
	
	private double discountFactor = 0.9;
	private PrioritizedSweeping prioritizedSweeping;
	
	//private DirectedExploration directedExploration;
	
	private double epsilon = 0.1;
	private EpsilonGreedyPolicy epsilonGreedyPolicy;
	
	//private BoltzmannActionSelection boltzmannPolicy;
	//private final double temperatureDiscountFactor = 0.999; 
	
	private ModeledBumperState previousState;
	private BumperAction previousAction;
	private Queue<TransitionReward> observedTransitions = new LinkedList<>();
	
	private double accumulatedReward = 0;
	private int accumulatedCollisions = 0;
	
	private CsvLogWriter logWriter;
	
	private volatile boolean actionRequested;
	
	
	public ObjectTrackingPrioritizedSweepingBumper(String logFile) {
		this();
		logWriter = new CsvLogWriter(new File(logFile), "Accumulated reward", "Accumulated collisions"); 
	}
	
	public ObjectTrackingPrioritizedSweepingBumper() {
		qTable = new ArrayQTable(stateDiscretizer.getNumberOfStates(), BumperAction.values().length);
		
		model = new BumperModel(rewardFunction, collisionStateDiscretizer);
		//model = new BumperModel(rewardFunction, stateDiscretizer);
		setPriorCollisionProbabilities();
		
		prioritizedSweeping = new PrioritizedSweeping();
		prioritizedSweeping.setDiscountFactor(discountFactor);
		prioritizedSweeping.setStateDiscretizer(stateDiscretizer);
		prioritizedSweeping.setModel(model);
		prioritizedSweeping.setQValueChangeThreshold(1E-4);
		prioritizedSweeping.setQTable(qTable);
		
		//directedExploration = new DirectedExploration(qTable, 0.005, 0.1, BumperAction.values());
		epsilonGreedyPolicy = new EpsilonGreedyPolicy(epsilon, qTable, BumperAction.values());
		//boltzmannPolicy = new BoltzmannActionSelection(directedExploration, 10, BumperAction.values());
		
		Thread sweeperThread = new Thread(new Sweeper());
		sweeperThread.start();
	}
	
	private void setPriorCollisionProbabilities() {
		ModeledBumperState fromState, toState;
		BumperAction action;
		
		// Add collisions when an obstacle is close in front, not collision yet, and the agent drives forward
		fromState = new ModeledBumperState();
		fromState.addObservation(TrackedObject.inPolarDegreeCoordinates(BumperAction.DRIVE_DISTANCE_CM, 0));
		fromState.setCollided(false);
		action = BumperAction.FORWARD;
		toState = fromState.afterAction(action);
		toState.setCollided(true);
		for (int i = 0; i < 80; i++) {
			TransitionReward transition = new TransitionReward(fromState, action, toState, -100); // reward here doesn't matter in this implementation
			model.updateModel(transition);
		}

		// Same when already collided before starting the action. Now we're more certain the agent collides
		fromState.setCollided(true);
		toState = fromState;
		for (int i = 0; i < 180; i++) {
			TransitionReward transition = new TransitionReward(fromState, action, toState, -100); 
			model.updateModel(transition);
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
		synchronized (prioritizedSweeping) { // synchronize on observedTransitions, qTable, prioritizedSweeping
			
			if (transitionReward != null) {
				observedTransitions.add(transitionReward);
			}

			//BumperAction action = BumperAction.getAction(boltzmannPolicy.getActionId(currentStateId));
			action = BumperAction.getAction(epsilonGreedyPolicy.getActionId(currentStateId));
			actionRequested = false;
			
			prioritizedSweeping.setSweepStartStateAction(new StateAction(currentState, action));
			//directedExploration.recordStateAction(new DiscretizedStateAction(currentStateId, action.getId()));
			//boltzmannPolicy.setTemperature(boltzmannPolicy.getTemperature() * temperatureDiscountFactor);

			prioritizedSweeping.notify();
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
				synchronized (prioritizedSweeping) {
					
					while (actionRequested) {
						try {
							prioritizedSweeping.wait();
						} catch (InterruptedException e) {}
					}
					
					TransitionReward transitionReward = observedTransitions.poll();
					if (transitionReward != null) {
						//System.out.println("sweeps between observations " + sweepsBetweenObservations);
						sweepsBetweenObservations = 0;
						prioritizedSweeping.updateModel(transitionReward);
					}
					sweepsBetweenObservations += prioritizedSweeping.performIterations(1);
					
				}
			}
			
		}
		
	}

}
