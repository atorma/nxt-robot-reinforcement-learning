package org.atorma.robot.objecttrackingbumper.prioritizedsweeping;

import java.io.File;
import java.util.LinkedList;
import java.util.Queue;

import org.atorma.robot.DiscreteRobotController;
import org.atorma.robot.learning.ArrayQTable;
import org.atorma.robot.learning.prioritizedsweeping.PrioritizedSweeping;
import org.atorma.robot.logging.CsvLogWriter;
import org.atorma.robot.mdp.StateAction;
import org.atorma.robot.mdp.Transition;
import org.atorma.robot.mdp.TransitionReward;
import org.atorma.robot.objecttrackingbumper.BumperRewardFunction;
import org.atorma.robot.objecttrackingbumper.BumperStateDiscretizer;
import org.atorma.robot.objecttrackingbumper.ModeledBumperState;
import org.atorma.robot.policy.EpsilonGreedyPolicy;
import org.atorma.robot.simplebumper.BumperAction;
import org.atorma.robot.simplebumper.BumperPercept;
import org.atorma.robot.simplebumper.ObstacleDistanceDiscretizer;

public class ObjectTrackingPrioritizedSweepingBumper implements DiscreteRobotController {
	
	private BumperStateDiscretizer stateDiscretizer = new BumperStateDiscretizer();
	private BumperRewardFunction rewardFunction = new BumperRewardFunction();
	
	private BumperModel model;
	
	private double discountFactor = 0.9;
	private PrioritizedSweeping prioritizedSweeping;
	
	private double epsilon = 0.1;
	private EpsilonGreedyPolicy epsilonGreedyPolicy;
	
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
		model = new BumperModel(rewardFunction, new ObstacleDistanceDiscretizer());
		
		prioritizedSweeping = new PrioritizedSweeping();
		prioritizedSweeping.setDiscountFactor(discountFactor);
		prioritizedSweeping.setStateDiscretizer(stateDiscretizer);
		prioritizedSweeping.setModel(model);
		prioritizedSweeping.setQValueChangeThreshold(1E-4);
		prioritizedSweeping.setQTable(new ArrayQTable(stateDiscretizer.getNumberOfStates(), BumperAction.values().length));
		
		epsilonGreedyPolicy = new EpsilonGreedyPolicy(epsilon, prioritizedSweeping, BumperAction.values());
		
		Thread sweeperThread = new Thread(new Sweeper());
		sweeperThread.setPriority(Thread.NORM_PRIORITY - 1);
		sweeperThread.start();
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
				currentState = previousState.afterAction(previousAction);
			} else {
				currentState = new ModeledBumperState();
			}
			currentState.addObservation(currentPercept);
			
			if (previousAction != null) {
				Transition transition = new Transition(previousState, previousAction, currentState);
				double reward = rewardFunction.getReward(transition);
				accumulatedReward += reward;
				TransitionReward transitionReward = new TransitionReward(transition, reward);
				observedTransitions.add(transitionReward);
			}
			
			if (logWriter != null) {
				logWriter.addRow(accumulatedReward, accumulatedCollisions);
			}
			
			int currentStateId = stateDiscretizer.getId(currentState);
			BumperAction action = BumperAction.getAction(epsilonGreedyPolicy.getActionId(currentStateId));
			prioritizedSweeping.setSweepStartStateAction(new StateAction(currentState, action));
			
			previousState = currentState;
			previousAction = action;
			
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
						System.out.println("sweeps between observations " + sweepsBetweenObservations);
						sweepsBetweenObservations = 0;
						prioritizedSweeping.updateModel(transitionReward);
					}
					sweepsBetweenObservations += prioritizedSweeping.performIterations(1);
				}
			}
			
		}
		
	}

}
