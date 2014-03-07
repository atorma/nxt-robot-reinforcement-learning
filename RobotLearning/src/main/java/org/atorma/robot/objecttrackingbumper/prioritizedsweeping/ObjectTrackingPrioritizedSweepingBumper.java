package org.atorma.robot.objecttrackingbumper.prioritizedsweeping;

import java.io.File;

import org.atorma.robot.DiscreteRobotController;
import org.atorma.robot.learning.prioritizedsweeping.PrioritizedSweeping;
import org.atorma.robot.logging.CsvLogWriter;
import org.atorma.robot.mdp.*;
import org.atorma.robot.objecttracking.ObjectTrackingModel;
import org.atorma.robot.objecttracking.TrackedObject;
import org.atorma.robot.objecttrackingbumper.BumperRewardFunction;
import org.atorma.robot.objecttrackingbumper.BumperStateDiscretizer;
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
		BumperModel model = new BumperModel();
		
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
		ModeledBumperState currentState = estimateObstacleLocations(currentPercept);

		if (previousState != null) {
			Transition transition = new Transition(previousState, previousAction, currentState);
			double reward = rewardFunction.getReward(transition);
			accumulatedReward += reward;
			TransitionReward transitionReward = new TransitionReward(transition, rewardFunction.getReward(transition));
			prioritizedSweeping.updateModel(transitionReward);
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
	
	private ModeledBumperState estimateObstacleLocations(BumperPercept currentPercept) {
		ObjectTrackingModel model = new ModeledBumperState();
		if (previousAction != null && previousState != null) {
			switch(previousAction) {
			case FORWARD:
				model = previousState.afterAgentMoves(BumperAction.DRIVE_DISTANCE_CM);
				break;
			case BACKWARD:
				model = previousState.afterAgentMoves(-BumperAction.DRIVE_DISTANCE_CM);
				break;
			case LEFT:
				model = previousState.afterAgentRotatesDeg(-BumperAction.TURN_DEGREES);
				break;
			case RIGHT:
				model = previousState.afterAgentRotatesDeg(BumperAction.TURN_DEGREES);
				break;
			}
		}
		model.addObservation(TrackedObject.inPolarDegreeCoordinates(currentPercept.getDistanceToObstacleInFrontCm(), 0));
		return (ModeledBumperState) model;
	}

}
