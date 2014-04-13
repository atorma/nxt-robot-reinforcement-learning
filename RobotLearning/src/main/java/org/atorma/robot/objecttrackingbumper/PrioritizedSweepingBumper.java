package org.atorma.robot.objecttrackingbumper;

import java.util.*;

import org.atorma.robot.DiscreteRobotController;
import org.atorma.robot.learning.ArrayQTable;
import org.atorma.robot.learning.QTable;
import org.atorma.robot.learning.prioritizedsweeping.PrioritizedSweeping;
import org.atorma.robot.mdp.*;
import org.atorma.robot.objecttracking.CircleSector;
import org.atorma.robot.policy.EpsilonGreedyPolicy;
import org.atorma.robot.simplebumper.*;

public class PrioritizedSweepingBumper implements DiscreteRobotController {
	
	private BumperStateDiscretizer stateDiscretizer;
	private BumperRewardFunction rewardFunction = new BumperRewardFunction();
	private BumperModel model;
	
	private QTable qTable;
	private double discountFactor = 0.9;
	private PrioritizedSweeping prioritizedSweeping;

	private double epsilon = 0.1;
	private EpsilonGreedyPolicy epsilonGreedyPolicy;
	
	private ModeledBumperState previousState;
	private BumperAction previousAction;
	
	private double accumulatedReward = 0;
	private int accumulatedCollisions = 0;
	
	private BumperLogWriter logWriter;
	
	private volatile boolean actionRequested;
	
	
	public PrioritizedSweepingBumper(String logFile) {
		this();
		logWriter = new BumperLogWriter(logFile);
	}
	
	public PrioritizedSweepingBumper() {
//		List<CircleSector> obstacleSectors = Arrays.asList(
//				new CircleSector(-65.7, -22.5),
//				new CircleSector(-22.5, 22.5),
//				new CircleSector(22.5, 67.5));
		List<CircleSector> obstacleSectors = Arrays.asList(
				new CircleSector(-180, -60),
				new CircleSector(-60, 60),
				new CircleSector(60, 120));
		stateDiscretizer = new BumperStateDiscretizer(obstacleSectors);
		
		qTable = new ArrayQTable(stateDiscretizer.getNumberOfStates(), BumperAction.values().length);
		
		model = new BumperModel(rewardFunction, stateDiscretizer);
		model.setDefaultCollisionProbabilityPrior(2, 10);
		BumperModelUtils.setPriorCollisionProbabilities(model, stateDiscretizer, 0.8, 0.99);
		
		prioritizedSweeping = new PrioritizedSweeping();
		prioritizedSweeping.setDiscountFactor(discountFactor);
		prioritizedSweeping.setStateDiscretizer(stateDiscretizer);
		prioritizedSweeping.setModel(model);
		prioritizedSweeping.setQValueChangeThreshold(0.01);
		prioritizedSweeping.setQTable(qTable);
		
		epsilonGreedyPolicy = new EpsilonGreedyPolicy(epsilon, qTable, BumperAction.values());
		
		Thread sweeperThread = new Thread(new Sweeper());
		sweeperThread.start();
	}

	@Override
	public int getActionId(double[] currentPerceptValues) {
		
		
		BumperPercept currentPercept = new BumperPercept(currentPerceptValues);
		if (currentPercept.isCollided()) {
			accumulatedCollisions++;
//			model.printCollisionProbabilities();
		}
		
		ModeledBumperState currentState;
		TransitionReward transitionReward = null;
		if (previousAction != null) {
			currentState = previousState.afterActionAndObservation(previousAction, currentPercept);
			Transition transition = new Transition(previousState, previousAction, currentState);
			double reward = rewardFunction.getReward(transition);
			accumulatedReward += reward;
			transitionReward = new TransitionReward(transition, reward);
		} else {
			currentState = ModeledBumperState.initialize(currentPercept);
		}
		
		BumperAction action;
		actionRequested = true;
		synchronized (prioritizedSweeping) { // synchronize on observedTransitions, qTable, prioritizedSweeping
			
			if (transitionReward != null) {
				model.update(transitionReward);
			}

			int currentStateId = stateDiscretizer.getId(currentState);
			action = BumperAction.getAction(epsilonGreedyPolicy.getActionId(currentStateId));
			actionRequested = false;

			prioritizedSweeping.setSweepStartStateAction(new StateAction(currentState, action));
			prioritizedSweeping.notify();
		}
		
		if (logWriter != null) {
			logWriter.log(accumulatedReward, accumulatedCollisions, currentState.isCollided(), action);
		}
				
		previousState = currentState;
		previousAction = action;

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
//						System.out.println("sweeps between observations " + sweepsBetweenObservations);
						sweepsBetweenObservations = 0;
					}
				}
				
				synchronized (prioritizedSweeping) {
					sweepsBetweenObservations += prioritizedSweeping.performIterations(1200); // Increase the number of iterations to ensure minimum
				}
				
			}
			
		}
		
	}

}
