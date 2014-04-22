package org.atorma.robot.objecttrackingbumper;

import java.util.*;

import org.atorma.robot.DiscreteRobotController;
import org.atorma.robot.learning.*;
import org.atorma.robot.learning.montecarlo.QLearningUctPlanning;
import org.atorma.robot.learning.montecarlo.QLearningUctPlanningParameters;
import org.atorma.robot.mdp.*;
import org.atorma.robot.objecttracking.CircleSector;
import org.atorma.robot.simplebumper.*;

public class QLearningUctPlanningBumper implements DiscreteRobotController {

	private BumperModel model;

	private RewardFunction rewardFunction = new BumperRewardFunction();
	private List<CircleSector> obstacleSectors;
	private BumperStateDiscretizer stateDiscretizer;
	private StateActionDiscretizer transitionDiscretizer;
	
	private double discountFactor = 0.7;

	private QTable qTable;
	private QLearning qLearning;
	private double learningRate = 0.2;
	private double traceDecay = 0.9;
	private EligibilityTraces traces;
	private Random random = new Random();
	private double epsilon = 0.1;
	
	private QLearningUctPlanning uctPlanning;
	private int planningHorizon = 10;
	private double learningRatePlanning = 0.2;
	private double traceDecayPlanning = 0.8;
	private double uctConstant = (1 + 100)/(1- discountFactor);
	
	private ModeledBumperState previousState;
	private BumperAction previousAction;
	
	private double accumulatedReward = 0;
	private int accumulatedCollisions = 0;
	
	private BumperLogWriter logWriter;
	
	private volatile boolean actionRequested;

	
	public QLearningUctPlanningBumper(String logFile) {
		this();
		logWriter = new BumperLogWriter(logFile); 
	}
	

	public QLearningUctPlanningBumper() {
		obstacleSectors = Arrays.asList(
				new CircleSector(-67.5, -22.5),
				new CircleSector(-22.5, 22.5),
				new CircleSector(22.5, 67.5));
		stateDiscretizer = new BumperStateDiscretizer(obstacleSectors);
		
		transitionDiscretizer = new StateActionDiscretizer(stateDiscretizer, rewardFunction);
		
		model = new BumperModel(rewardFunction, stateDiscretizer);
		model.setDefaultCollisionProbabilityPrior(2, 10);
		BumperModelUtils.setPriorCollisionProbabilities(model, stateDiscretizer, 0.8, 0.99);
		
		qTable = new ArrayQTable(stateDiscretizer.getNumberOfStates(), BumperAction.values().length);
		traces = new ReplacingEligibilityTraces(discountFactor, traceDecay);
		qLearning = new QLearning(learningRate, traces, qTable);
		
		QLearningUctPlanningParameters uctParams = new QLearningUctPlanningParameters();
		uctParams.model = model;
		uctParams.allActions = BumperAction.values();
		uctParams.stateDiscretizer = stateDiscretizer;
		uctParams.planningHorizon = planningHorizon;
		uctParams.learningRate = learningRatePlanning;
		uctParams.eligibilityTraces = new ReplacingEligibilityTraces(discountFactor, traceDecayPlanning);
		uctParams.uctConstant = uctConstant;
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
		
		actionRequested = true;
		BumperAction action;
		synchronized(uctPlanning) {
			if (transitionReward != null) {
				model.update(transitionReward);
				qLearning.update(transitionDiscretizer.discretize(transitionReward));
			}
			
			action = (BumperAction) uctPlanning.getPlannedAction(currentState);
			if (random.nextDouble() < epsilon) {
				action = BumperAction.values()[random.nextInt(BumperAction.values().length)];
			}
			actionRequested = false;
			
			uctPlanning.setRolloutStartState(currentState);
			uctPlanning.notify();
		}
		
		previousState = currentState;
		previousAction = action;
		
		if (logWriter != null) {
			logWriter.log(accumulatedReward, accumulatedCollisions, currentState.isCollided(), action);
		}
		
		return action.getId();
	}
	

	private class Sweeper implements Runnable {
		
		private final int minRollouts = 1;

		@Override
		public void run() {
			int rolloutsBetweenActions = 0;
			
			while (true) {
				
				synchronized (uctPlanning) {
					while (actionRequested) {
						try {
							uctPlanning.wait();
						} catch (InterruptedException e) {}
//						System.out.println("Rollouts: " + rolloutsBetweenActions);
						rolloutsBetweenActions = 0;
					}
				}
				
				synchronized (uctPlanning) {
					uctPlanning.performRollouts(minRollouts); // Increase the number to ensure minimum	
				}
				rolloutsBetweenActions += minRollouts;
			}
			
		}
		
	}
}
