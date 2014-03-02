package org.atorma.robot.objecttrackingbumper;

import java.io.File;

import org.atorma.robot.*;
import org.atorma.robot.learning.QLearning;
import org.atorma.robot.learning.Transition;
import org.atorma.robot.logging.CsvLogWriter;
import org.atorma.robot.objecttracking.ObjectTrackingModel;
import org.atorma.robot.objecttracking.TrackedObject;
import org.atorma.robot.simplebumper.BumperAction;
import org.atorma.robot.simplebumper.BumperPercept;

public class ObjectTrackingQLearningBumper implements DiscreteRobotController {
	
	private BumperStateDiscretizer stateDiscretizer = new BumperStateDiscretizer();
	
	private BumperRewardFunction rewardFunction = new BumperRewardFunction();
	
	private double learningRate = 0.1;
	private double discountFactor = 0.9;
	private QLearning qLearning;
	
	private double epsilon = 0.1;
	private EpsilonGreedyPolicy epsilonGreedyPolicy;
	
	private ModeledBumperState previousState;
	private BumperAction previousAction;
	
	private ObjectTrackingModel objectTrackingModel;
	
	private int accumulatedCollisions = 0;
	
	private CsvLogWriter logWriter;
	
	
	public ObjectTrackingQLearningBumper(String logFile) {
		this();
		logWriter = new CsvLogWriter(new File(logFile), "Accumulated reward", "Accumulated collisions"); 
	}
	
	public ObjectTrackingQLearningBumper() {
		qLearning = new QLearning(stateDiscretizer, rewardFunction, learningRate, discountFactor);
		epsilonGreedyPolicy = new EpsilonGreedyPolicy(epsilon, BumperAction.values(), qLearning);
		objectTrackingModel = new ObjectTrackingModel();
	}
	
	
	@Override
	public int getActionId(double[] currentPerceptValues) {
		BumperPercept currentPercept = new BumperPercept(currentPerceptValues);
		if (currentPercept.isCollided()) {
			accumulatedCollisions++;
		}
		updateObjectTrackingModel(currentPercept);
		ModeledBumperState currentState = new ModeledBumperState(objectTrackingModel, currentPercept.isCollided());
		System.out.println(currentState);
		
		int currentStateId = stateDiscretizer.getId(currentState.getValues());
		BumperAction currentAction = BumperAction.getAction(epsilonGreedyPolicy.getActionId(currentStateId));
		
		if (previousState != null) {
			Transition transition = new Transition(previousState, previousAction, currentState);
			qLearning.update(transition);
			//System.out.println("Total reward: " + qLearning.getAccumulatedReward());
		}
		
		if (logWriter != null) {
			logWriter.addRow(qLearning.getAccumulatedReward(), accumulatedCollisions);
		}
		
		previousState = currentState;
		previousAction = currentAction;
		
		return currentAction.getId();
			
	}
	
	private void updateObjectTrackingModel(BumperPercept currentPercept) {
		if (previousAction != null) {
			switch(previousAction) {
			case FORWARD:
				objectTrackingModel.agentMoves(BumperAction.DRIVE_DISTANCE_CM);
				break;
			case BACKWARD:
				objectTrackingModel.agentMoves(-BumperAction.DRIVE_DISTANCE_CM);
				break;
			case LEFT:
				objectTrackingModel.agentRotatesDeg(-BumperAction.TURN_DEGREES);
				break;
			case RIGHT:
				objectTrackingModel.agentRotatesDeg(BumperAction.TURN_DEGREES);
				break;
			}
		}
		objectTrackingModel.addObservation(TrackedObject.inPolarDegreeCoordinates(currentPercept.getDistanceToObstacleInFrontCm(), 0));
		//System.out.println(objectTrackingModel);
	}

}
