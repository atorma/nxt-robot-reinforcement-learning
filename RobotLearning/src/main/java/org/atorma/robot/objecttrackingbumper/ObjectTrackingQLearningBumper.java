package org.atorma.robot.objecttrackingbumper;

import java.util.Arrays;
import java.util.List;

import org.atorma.robot.DiscreteRobotController;
import org.atorma.robot.learning.*;
import org.atorma.robot.mdp.DiscretizedTransitionReward;
import org.atorma.robot.mdp.StateActionDiscretizer;
import org.atorma.robot.objecttracking.CircleSector;
import org.atorma.robot.policy.EpsilonGreedyPolicy;
import org.atorma.robot.simplebumper.*;

public class ObjectTrackingQLearningBumper implements DiscreteRobotController {
	
	private BumperStateDiscretizer stateDiscretizer;
	private BumperRewardFunction rewardFunction = new BumperRewardFunction();
	private StateActionDiscretizer transitionDiscretizer;
	
	private double learningRate = 0.1;
	private double discountFactor = 0.9;
	private double traceDecay = 0.9;
	private EligibilityTraces traces = new ReplacingEligibilityTraces(discountFactor, traceDecay);
	private QTable qTable;
	private QLearning qLearning;
	
	private double epsilon = 0.1;
	private EpsilonGreedyPolicy epsilonGreedyPolicy;
	
	private ModeledBumperState previousState;
	private BumperAction previousAction;

	private int accumulatedCollisions = 0;
	private double accumulatedReward = 0;
	
	private BumperLogWriter logWriter;
	
	
	public ObjectTrackingQLearningBumper(String logFile) {
		this();
		logWriter = new BumperLogWriter(logFile); 
	}
	
	public ObjectTrackingQLearningBumper() {
//		List<CircleSector> obstacleSectors = Arrays.asList(
//				new CircleSector(-180, -60),
//				new CircleSector(-60, 60),
//				new CircleSector(60, 180));
		List<CircleSector> obstacleSectors = Arrays.asList(new CircleSector(-180, 180));
		stateDiscretizer = new BumperStateDiscretizer(obstacleSectors);
		
		transitionDiscretizer = new StateActionDiscretizer(stateDiscretizer, rewardFunction);
		
		qTable = new ArrayQTable(stateDiscretizer.getNumberOfStates(), BumperAction.values().length);
//		qLearning = new QLearning(learningRate, traces, qTable);
		qLearning = new QLearning(learningRate, discountFactor, qTable);
		epsilonGreedyPolicy = new EpsilonGreedyPolicy(epsilon, qLearning, BumperAction.values());
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
			DiscretizedTransitionReward transition = transitionDiscretizer.discretizeAndComputeReward(previousState, previousAction, currentState);
			qLearning.update(transition);
			accumulatedReward += transition.getReward();
		} else {
			currentState = ModeledBumperState.initialize(currentPercept);
		}

		int currentStateId = stateDiscretizer.getId(currentState);
		BumperAction action = BumperAction.getAction(epsilonGreedyPolicy.getActionId(currentStateId));
		
		if (logWriter != null) {
			logWriter.log(accumulatedReward, accumulatedCollisions, currentState.isCollided(), action);
		}
		
		previousState = currentState;
		previousAction = action;
		
		return action.getId();
			
	}

}
