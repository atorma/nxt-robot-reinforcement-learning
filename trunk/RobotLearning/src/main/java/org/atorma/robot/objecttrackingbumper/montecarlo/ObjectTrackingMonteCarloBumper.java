package org.atorma.robot.objecttrackingbumper.montecarlo;

import java.io.File;

import org.atorma.robot.DiscreteRobotController;
import org.atorma.robot.discretization.StateDiscretizer;
import org.atorma.robot.learning.*;
import org.atorma.robot.learning.montecarlo.*;
import org.atorma.robot.logging.CsvLogWriter;
import org.atorma.robot.mdp.*;
import org.atorma.robot.objecttrackingbumper.*;
import org.atorma.robot.objecttrackingbumper.prioritizedsweeping.BumperModel;
import org.atorma.robot.objecttrackingbumper.prioritizedsweeping.SingleSectorCollisionStateDiscretizer;
import org.atorma.robot.simplebumper.BumperAction;
import org.atorma.robot.simplebumper.BumperPercept;

public class ObjectTrackingMonteCarloBumper implements DiscreteRobotController {

	private BumperModel model;

	private RewardFunction rewardFunction = new BumperRewardFunction();
	private StateDiscretizer stateDiscretizer = new SingleSectorCollisionStateDiscretizer(60);
	//private BumperStateDiscretizer stateDiscretizer = new BumperStateDiscretizer();
	private double discountFactor = 0.8;
	
	private QTable qTable;
	private QLearning qLearning;
	private double learningRate = 0.1;
	private double traceDecay = 0.8;
	private EligibilityTraces traces;
	
	private QLearningUctPlanning uctPlanning;
	private int planningHorizon = 30;
	
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
		traces = new ReplacingEligibilityTraces(discountFactor, traceDecay);
		qLearning = new QLearning(learningRate, traces, qTable);
		
		QLearningUctPlanningParameters uctParams = new QLearningUctPlanningParameters();
		uctParams.model = model;
		uctParams.allActions = BumperAction.values();
		uctParams.stateDiscretizer = stateDiscretizer;
		uctParams.planningHorizon = planningHorizon;
		uctParams.learningRate = learningRate;
		uctParams.eligibilityTraces = new ReplacingEligibilityTraces(discountFactor, traceDecay);
		uctParams.uctConstant = 3;
		uctParams.longTermQValues = qTable;
		uctPlanning = new QLearningUctPlanning(uctParams);
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
		uctPlanning.performRollouts(50);
		BumperAction action = (BumperAction) uctPlanning.getPlannedAction(currentState);
		
		previousState = currentState;
		previousAction = action;
		
		if (logWriter != null) {
			logWriter.addRow(accumulatedReward, accumulatedCollisions);
		}
		
		return action.getId();
	}
	

}
