package org.atorma.robot.objecttrackingbumper;

import org.atorma.robot.*;
import org.atorma.robot.learning.QLearning;
import org.atorma.robot.learning.Transition;
import org.atorma.robot.objecttracking.ObjectTrackingModel;
import org.atorma.robot.objecttracking.TrackedObject;
import org.atorma.robot.simplebumper.BumperAction;
import org.atorma.robot.simplebumper.BumperPercept;

public class ObjectTrackingQLearningBumper implements DiscreteActionController {
	
	private BumperStateIdFunction stateIdMap = new BumperStateIdFunction();
	
	private BumperRewardFunction rewardFunction = new BumperRewardFunction();
	
	private double learningRate = 0.1;
	private double discountFactor = 0.9;
	private QLearning qLearning;
	
	private double epsilon = 0.1;
	private EpsilonGreedyPolicy epsilonGreedyPolicy;
	
	private ModeledBumperState previousState;
	private BumperAction previousAction;
	
	private ObjectTrackingModel objectTrackingModel;
	
	
	public ObjectTrackingQLearningBumper() {
		qLearning = new QLearning(stateIdMap, rewardFunction, learningRate, discountFactor);
		epsilonGreedyPolicy = new EpsilonGreedyPolicy(epsilon, BumperAction.values(), qLearning);
		objectTrackingModel = new ObjectTrackingModel();
	}
	
	
	@Override
	public int getActionId(double[] currentPerceptValues) {
		BumperPercept currentPercept = new BumperPercept(currentPerceptValues);
		updateObjectTrackingModel(currentPercept);
		ModeledBumperState currentState = new ModeledBumperState(objectTrackingModel);
		//System.out.println(currentState);
		
		int currentStateId = stateIdMap.getId(currentState.getValues());
		BumperAction currentAction = BumperAction.getAction(epsilonGreedyPolicy.getActionId(currentStateId));
		
		if (previousState != null) {
			Transition transition = new Transition(previousState, previousAction, currentState);
			qLearning.update(transition);
			//System.out.println("Total reward: " + qLearning.getAccumulatedReward());
		}
		
		previousState = currentState;
		previousAction = currentAction;
		
		return currentAction.getId();
			
	}
	
	private void updateObjectTrackingModel(BumperPercept currentPercept) {
		if (previousAction != null) {
			//System.out.println("Action: " + previousAction);
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
		if (currentPercept.isCollided()) {
			objectTrackingModel.addObservation(TrackedObject.inPolarDegreeCoordinates(0, 0));
		}
		//System.out.println(objectTrackingModel);
		//System.out.println("Number of objects: " + objectTrackingModel.getObjects().size());
	}

}
