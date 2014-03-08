package org.atorma.robot.objecttrackingbumper.prioritizedsweeping;

import java.io.File;

import org.atorma.robot.DiscreteRobotController;
import org.atorma.robot.learning.prioritizedsweeping.PrioritizedSweeping;
import org.atorma.robot.logging.CsvLogWriter;
import org.atorma.robot.mdp.Transition;
import org.atorma.robot.mdp.TransitionReward;
import org.atorma.robot.objecttrackingbumper.*;
import org.atorma.robot.policy.EpsilonGreedyPolicy;
import org.atorma.robot.simplebumper.BumperAction;
import org.atorma.robot.simplebumper.BumperPercept;

public class ObjectTrackingPrioritizedSweepingBumper implements DiscreteRobotController {
	
	private BumperStateDiscretizer stateDiscretizer = new BumperStateDiscretizer();
	private BumperRewardFunction rewardFunction = new BumperRewardFunction();
	
	private double discountFactor = 0.9;
	private PrioritizedSweeping prioritizedSweeping;
	
	private double epsilon = 0.1;
	private EpsilonGreedyPolicy epsilonGreedyPolicy;
	
	private ModeledBumperState previousState;
	private BumperAction previousAction;
	
	private double accumulatedReward = 0;
	private int accumulatedCollisions = 0;
	
	private CsvLogWriter logWriter;
	
	
	public ObjectTrackingPrioritizedSweepingBumper(String logFile) {
		this();
		logWriter = new CsvLogWriter(new File(logFile), "Accumulated reward", "Accumulated collisions"); 
	}
	
	public ObjectTrackingPrioritizedSweepingBumper() {
		BumperModel model = new BumperModel(rewardFunction, new ObstacleDistanceDiscretizer());
		
		prioritizedSweeping = new PrioritizedSweeping();
		prioritizedSweeping.setDiscountFactor(discountFactor);
		prioritizedSweeping.setStateDiscretizer(stateDiscretizer);
		prioritizedSweeping.setModel(model);
		
		epsilonGreedyPolicy = new EpsilonGreedyPolicy(epsilon, prioritizedSweeping, BumperAction.values());
	}
	
	
	@Override
	public int getActionId(double[] currentPerceptValues) {
		
		BumperPercept currentPercept = new BumperPercept(currentPerceptValues);
		if (currentPercept.isCollided()) {
			accumulatedCollisions++;
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
			prioritizedSweeping.updateModel(transitionReward);
			
			prioritizedSweeping.setSweepStartStateAction(transition.getFromStateAction());
			prioritizedSweeping.performIterations(50);
		}
		
		if (logWriter != null) {
			logWriter.addRow(accumulatedReward, accumulatedCollisions);
		}
		
		int currentStateId = stateDiscretizer.getId(currentState);
		BumperAction action = BumperAction.getAction(epsilonGreedyPolicy.getActionId(currentStateId));
		
		previousState = currentState;
		previousAction = action;
		
		return action.getId();
			
	}

}
