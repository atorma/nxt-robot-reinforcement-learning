package org.atorma.robot.objecttrackingbumper.montecarlo;

import java.io.File;
import java.util.Arrays;
import java.util.List;

import org.atorma.robot.DiscreteRobotController;
import org.atorma.robot.learning.*;
import org.atorma.robot.learning.montecarlo.QLearningUctPlanning;
import org.atorma.robot.learning.montecarlo.QLearningUctPlanningParameters;
import org.atorma.robot.logging.CsvLogWriter;
import org.atorma.robot.mdp.*;
import org.atorma.robot.objecttracking.CircleSector;
import org.atorma.robot.objecttrackingbumper.*;
import org.atorma.robot.simplebumper.BumperAction;
import org.atorma.robot.simplebumper.BumperPercept;

public class ObjectTrackingMonteCarloBumper implements DiscreteRobotController {

	private BumperModel model;

	private RewardFunction rewardFunction = new BumperRewardFunction();
	private List<CircleSector> obstacleSectors;
	private BumperStateDiscretizer stateDiscretizer;
	private StateActionDiscretizer transitionDiscretizer;

	private QTable qTable;
	private QLearning qLearning;
	private double discountFactor = 0.9;
	private double learningRate = 0.2;
	private double traceDecay = 0.9;
	private EligibilityTraces traces;
	
	private QLearningUctPlanning uctPlanning;
	private int planningHorizon = 20;
	
	private ModeledBumperState previousState;
	private BumperAction previousAction;
	
	private double accumulatedReward = 0;
	private int accumulatedCollisions = 0;
	private CsvLogWriter logWriter;
	
	private volatile boolean actionRequested;

	
	
	
	public ObjectTrackingMonteCarloBumper(String logFile) {
		this();
		logWriter = new CsvLogWriter(new File(logFile), "Accumulated reward", "Accumulated collisions", "Collided", "Action"); 
	}
	

	public ObjectTrackingMonteCarloBumper() {
		obstacleSectors = Arrays.asList(
				new CircleSector(-77, -30),
				new CircleSector(-30, 30),
				new CircleSector(30, 75));
		stateDiscretizer = new BumperStateDiscretizer(obstacleSectors);
		
		transitionDiscretizer = new StateActionDiscretizer(stateDiscretizer, rewardFunction);
		
		model = new BumperModel(rewardFunction, stateDiscretizer);
//		BumperModelUtils.setPriorCollisionProbabilities(model, stateDiscretizer, 0.8, 0.99);
		
		qTable = new ArrayQTable(stateDiscretizer.getNumberOfStates(), BumperAction.values().length);
		traces = new ReplacingEligibilityTraces(discountFactor, traceDecay);
		qLearning = new QLearning(learningRate, traces, qTable);
		
		QLearningUctPlanningParameters uctParams = new QLearningUctPlanningParameters();
		uctParams.model = model;
		uctParams.allActions = BumperAction.values();
		uctParams.stateDiscretizer = stateDiscretizer;
		uctParams.planningHorizon = planningHorizon;
		uctParams.learningRate = learningRate;
		uctParams.eligibilityTraces = new ReplacingEligibilityTraces(discountFactor, traceDecay);
		uctParams.uctConstant = (1 + 100)/(1- discountFactor);
		uctParams.longTermQValues = qTable;
		uctPlanning = new QLearningUctPlanning(uctParams);
		
		Thread sweeperThread = new Thread(new Sweeper());
		sweeperThread.start();
	}

	
	@Override
	public int getActionId(double[] currentPerceptValues) {
		
		BumperPercept currentPercept = new BumperPercept(currentPerceptValues);
		if (currentPercept.isCollided()) {
			accumulatedCollisions++;
			model.printCollisionProbabilities();
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
		
//		for (CircleSector cs : obstacleSectors) {
//			System.out.println(cs + " " + currentState.getNearestInSectorDegrees(cs.getFromAngleDeg(), cs.getToAngleDeg()));
//		}

		actionRequested = true;
		BumperAction action;
		synchronized(uctPlanning) {
			if (transitionReward != null) {
				model.update(transitionReward);
				qLearning.update(transitionDiscretizer.discretize(transitionReward));
			}
			
			action = (BumperAction) uctPlanning.getPlannedAction(currentState);
			actionRequested = false;
			
			uctPlanning.setRolloutStartState(currentState);
			uctPlanning.notify();
		}
		
		previousState = currentState;
		previousAction = action;
		
		if (logWriter != null) {
			logWriter.addRow(accumulatedReward, accumulatedCollisions, currentState.isCollided(), action);
		}
		
		return action.getId();
	}
	

	private class Sweeper implements Runnable {

		@Override
		public void run() {
			int rolloutsBetweenActions = 0;
			
			while (true) {
				synchronized (uctPlanning) {
					
					while (actionRequested) {
						try {
							uctPlanning.wait();
						} catch (InterruptedException e) {}
						//System.out.println("Rollouts: " + rolloutsBetweenActions);
						rolloutsBetweenActions = 0;
					}

					uctPlanning.performRollouts(1); // Increase the number to ensure minimum
					rolloutsBetweenActions++;
				}
			}
			
		}
		
	}
}
