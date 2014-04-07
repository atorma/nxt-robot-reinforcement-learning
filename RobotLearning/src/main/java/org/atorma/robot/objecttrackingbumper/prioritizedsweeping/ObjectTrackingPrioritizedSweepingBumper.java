package org.atorma.robot.objecttrackingbumper.prioritizedsweeping;

import java.io.File;
import java.util.*;

import org.atorma.robot.DiscreteRobotController;
import org.atorma.robot.learning.ArrayQTable;
import org.atorma.robot.learning.QTable;
import org.atorma.robot.learning.prioritizedsweeping.PrioritizedSweeping;
import org.atorma.robot.logging.CsvLogWriter;
import org.atorma.robot.mdp.*;
import org.atorma.robot.objecttracking.CircleSector;
import org.atorma.robot.objecttrackingbumper.*;
import org.atorma.robot.policy.EpsilonGreedyPolicy;
import org.atorma.robot.simplebumper.BumperAction;
import org.atorma.robot.simplebumper.BumperPercept;

public class ObjectTrackingPrioritizedSweepingBumper implements DiscreteRobotController {
	
	private BumperStateDiscretizer stateDiscretizer;
	private BumperRewardFunction rewardFunction = new BumperRewardFunction();
	private BumperModel model;
	
	private QTable qTable;
	private double discountFactor = 0.3;
	private PrioritizedSweeping prioritizedSweeping;

	private double epsilon = 0.1;
	private EpsilonGreedyPolicy epsilonGreedyPolicy;
	
	//private DirectedExploration directedExploration;
	//private BoltzmannActionSelection boltzmannPolicy;
	//private double temperatureDiscountFactor = 0.9995; 
	
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
		
		List<CircleSector> obstacleSectors = Arrays.asList(
				new CircleSector(-45, -15),
				new CircleSector(-15, 15),
				new CircleSector(15, 45));
		stateDiscretizer = new BumperStateDiscretizer(obstacleSectors);
		
		qTable = new ArrayQTable(stateDiscretizer.getNumberOfStates(), BumperAction.values().length);
		
		model = new BumperModel(rewardFunction, stateDiscretizer);
		BumperModelUtils.setPriorCollisionProbabilities(model, stateDiscretizer, 0.8, 0.99);
		
		prioritizedSweeping = new PrioritizedSweeping();
		prioritizedSweeping.setDiscountFactor(discountFactor);
		prioritizedSweeping.setStateDiscretizer(stateDiscretizer);
		prioritizedSweeping.setModel(model);
		prioritizedSweeping.setQValueChangeThreshold(1E-4);
		prioritizedSweeping.setQTable(qTable);
		
		//directedExploration = new DirectedExploration(qTable, 0.02, 0.1, BumperAction.values());
		epsilonGreedyPolicy = new EpsilonGreedyPolicy(epsilon, qTable, BumperAction.values());
		//boltzmannPolicy = new BoltzmannActionSelection(directedExploration, 1, BumperAction.values());
		
		Thread sweeperThread = new Thread(new Sweeper());
		sweeperThread.start();
	}

	@Override
	public int getActionId(double[] currentPerceptValues) {
		actionRequested = true;
		
		BumperPercept currentPercept = new BumperPercept(currentPerceptValues);
		if (currentPercept.isCollided()) {
			accumulatedCollisions++;
			model.printCollisionProbabilities();
		}
		
		ModeledBumperState currentState;
		if (previousAction != null) {
			currentState = previousState.afterActionAndObservation(previousAction, currentPercept);
		} else {
			currentState = ModeledBumperState.initialize(currentPercept);
		}
		
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

			int currentStateId = stateDiscretizer.getId(currentState);
			action = BumperAction.getAction(epsilonGreedyPolicy.getActionId(currentStateId));
			//action = BumperAction.getAction(boltzmannPolicy.getActionId(currentStateId));
			//boltzmannPolicy.setTemperature(boltzmannPolicy.getTemperature() * temperatureDiscountFactor);
			actionRequested = false;
			
			//directedExploration.recordStateAction(new DiscretizedStateAction(currentStateId, action.getId()));

			prioritizedSweeping.setSweepStartStateAction(new StateAction(currentState, action));
			prioritizedSweeping.notify();
		}
		
		//System.out.println("Collision prob = " + model.getCollisionProbability(currentState, action));
		
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
						model.update(transitionReward);
					}
					sweepsBetweenObservations += prioritizedSweeping.performIterations(1); // Increase the number of iterations to ensure minimum
					
				}
			}
			
		}
		
	}

}
