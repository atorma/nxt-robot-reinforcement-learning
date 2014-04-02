package org.atorma.robot.objecttrackingbumper.montecarlo;

import java.io.File;

import org.atorma.robot.DiscreteRobotController;
import org.atorma.robot.discretization.StateDiscretizer;
import org.atorma.robot.learning.*;
import org.atorma.robot.learning.montecarlo.*;
import org.atorma.robot.logging.CsvLogWriter;
import org.atorma.robot.mdp.*;
import org.atorma.robot.objecttrackingbumper.BumperRewardFunction;
import org.atorma.robot.objecttrackingbumper.ModeledBumperState;
import org.atorma.robot.objecttrackingbumper.prioritizedsweeping.BumperModel;
import org.atorma.robot.objecttrackingbumper.prioritizedsweeping.SingleSectorCollisionStateDiscretizer;
import org.atorma.robot.simplebumper.BumperAction;
import org.atorma.robot.simplebumper.BumperPercept;

public class ObjectTrackingMonteCarloBumper implements DiscreteRobotController {

	private BumperModel model;

	private RewardFunction rewardFunction = new BumperRewardFunction();
	private StateDiscretizer stateDiscretizer = new SingleSectorCollisionStateDiscretizer(90);
	private double discountFactor = 0.8;
	
	private QTable qTable;
	private QLearning qLearning;
	private double learningRate = 0.1;
	
	private FirstVisitUctPlanning uctPlanning;
	private int planningHorizon = 15;
	
	private ModeledBumperState previousState;
	private BumperAction previousAction;
	
	private double accumulatedReward = 0;
	private int accumulatedCollisions = 0;
	
	private StateActionDiscretizer transitionDiscretizer = new StateActionDiscretizer(stateDiscretizer, rewardFunction);
	
	private CsvLogWriter logWriter;
	
	
	public ObjectTrackingMonteCarloBumper(String logFile) {
		this();
		logWriter = new CsvLogWriter(new File(logFile), "Accumulated reward", "Accumulated collisions"); 
	}
	

	public ObjectTrackingMonteCarloBumper() {
		qTable = new ArrayQTable(stateDiscretizer.getNumberOfStates(), BumperAction.values().length);
		model = new BumperModel(rewardFunction, stateDiscretizer);
		//setPriorCollisionProbabilities(0.8, 0.99);
		
		qLearning = new QLearning(learningRate, discountFactor, qTable);
		
		FirstVisitUctPlanningParameters uctParams = new FirstVisitUctPlanningParameters();
		uctParams.discountFactor = discountFactor;
		uctParams.planningHorizon = planningHorizon;
		uctParams.uctConstant = 15;
		uctParams.model = model;
		uctParams.stateDiscretizer = stateDiscretizer;
		uctParams.longTermQValues = qTable;
		uctPlanning = new FirstVisitUctPlanning(uctParams);
	}

	
	@Override
	public int getActionId(double[] currentPerceptValues) {
		
		BumperPercept currentPercept = new BumperPercept(currentPerceptValues);
		if (currentPercept.isCollided()) {
			accumulatedCollisions++;
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
			TransitionReward rewardTransition = new TransitionReward(transition, reward); 
			
			model.update(rewardTransition);
			qLearning.update(transitionDiscretizer.discretize(rewardTransition));
		}
		
		uctPlanning.setRolloutStartState(currentState);
		uctPlanning.performRollouts(100);
		BumperAction action = (BumperAction) uctPlanning.getPlannedAction(currentState);
		
		previousState = currentState;
		previousAction = action;
		
		if (logWriter != null) {
			logWriter.addRow(accumulatedReward, accumulatedCollisions);
		}
		
		return action.getId();
	}
	

}
