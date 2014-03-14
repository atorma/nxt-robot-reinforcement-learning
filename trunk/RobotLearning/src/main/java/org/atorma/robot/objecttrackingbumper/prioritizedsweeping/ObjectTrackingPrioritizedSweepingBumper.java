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
		sweeperThread.setPriority(Thread.NORM_PRIORITY - 1);
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
		synchronized (prioritizedSweeping) {
			
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
			

			if (previousAction != null) {
				Transition transition = new Transition(previousState, previousAction, currentState);
				double reward = rewardFunction.getReward(transition);
				accumulatedReward += reward;
				TransitionReward transitionReward = new TransitionReward(transition, reward);
				observedTransitions.add(transitionReward);
				//prioritizedSweeping.updateModel(transitionReward);
			}
			
			if (logWriter != null) {
				logWriter.addRow(accumulatedReward, accumulatedCollisions);
			}
			
			int currentStateId = stateDiscretizer.getId(currentState);
			//BumperAction action = BumperAction.getAction(boltzmannPolicy.getActionId(currentStateId));
			BumperAction action = BumperAction.getAction(epsilonGreedyPolicy.getActionId(currentStateId));
			
			//directedExploration.recordStateAction(new DiscretizedStateAction(currentStateId, action.getId()));
			//boltzmannPolicy.setTemperature(boltzmannPolicy.getTemperature() * temperatureDiscountFactor);
			
			// Debug
			if (currentState.isCollided()) {
				System.out.println("Collided!");
				System.out.println("Was already collided = " + previousState.isCollided());
				System.out.println("Previous action: " + previousAction);
				System.out.println("Next action: " + action);
				System.out.println("Previous state id " + stateDiscretizer.getId(previousState));
				System.out.println("Current state id " + stateDiscretizer.getId(currentState));
				System.out.println("Previous state " + previousState);
				System.out.println("Current state " + currentState);
				System.out.println("Current action values:");
				for (BumperAction a : BumperAction.values()) {
					System.out.println(a + " value " + prioritizedSweeping.getQTable().getValue(new DiscretizedStateAction(currentStateId, a.getId())));
				}
			}
			if (!currentState.isCollided() && previousState != null && previousState.isCollided()) {
				System.out.println("Got free!");
				System.out.println("Previous state " + previousState);
				System.out.println("Previous action " + previousAction);
			}
			
			previousState = currentState;
			previousAction = action;

			prioritizedSweeping.setSweepStartStateAction(new StateAction(currentState, action));
			//prioritizedSweeping.performIterations(1000);
			
			return action.getId();
		}
	}
	
	private class Sweeper implements Runnable {

		@Override
		public void run() {
			int sweepsBetweenObservations = 0;
			while (true) {
				synchronized (prioritizedSweeping) {
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
