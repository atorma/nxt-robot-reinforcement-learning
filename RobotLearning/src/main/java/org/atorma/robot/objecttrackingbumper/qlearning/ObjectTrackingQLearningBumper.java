package org.atorma.robot.objecttrackingbumper.qlearning;

import java.io.File;
import java.util.Arrays;
import java.util.List;

import org.atorma.robot.DiscreteRobotController;
import org.atorma.robot.learning.*;
import org.atorma.robot.logging.CsvLogWriter;
import org.atorma.robot.mdp.DiscretizedTransitionReward;
import org.atorma.robot.mdp.StateActionDiscretizer;
import org.atorma.robot.objecttracking.CircleSector;
import org.atorma.robot.objecttrackingbumper.*;
import org.atorma.robot.policy.EpsilonGreedyPolicy;
import org.atorma.robot.simplebumper.BumperAction;
import org.atorma.robot.simplebumper.BumperPercept;

public class ObjectTrackingQLearningBumper implements DiscreteRobotController {
	
	private BumperStateDiscretizer stateDiscretizer;
	private BumperRewardFunction rewardFunction = new BumperRewardFunction();
	private StateActionDiscretizer transitionDiscretizer;
	
	private double learningRate = 0.1;
	private double discountFactor = 0.5;
	private double traceDecay = 0.3;
	private EligibilityTraces traces = new ReplacingEligibilityTraces(discountFactor, traceDecay);
	private QTable qTable;
	private QLearning qLearning;
	
	private double epsilon = 0.1;
	private EpsilonGreedyPolicy epsilonGreedyPolicy;
	
	private ModeledBumperState previousState;
	private BumperAction previousAction;

	private int accumulatedCollisions = 0;
	private double accumulatedReward = 0;
	
	private CsvLogWriter logWriter;
	
	
	public ObjectTrackingQLearningBumper(String logFile) {
		this();
		logWriter = new CsvLogWriter(new File(logFile), "Accumulated reward", "Accumulated collisions"); 
	}
	
	public ObjectTrackingQLearningBumper() {
		List<CircleSector> obstacleSectors = Arrays.asList(
				new CircleSector(270, 330),
				new CircleSector(330, 30),
				new CircleSector(30, 90));
//		List<CircleSector> obstacleSectors = Arrays.asList(new CircleSector(330, 30));
		stateDiscretizer = new BumperStateDiscretizer(obstacleSectors);
		
		transitionDiscretizer = new StateActionDiscretizer(stateDiscretizer, rewardFunction);
		
		qTable = new ArrayQTable(stateDiscretizer.getNumberOfStates(), BumperAction.values().length);
		qLearning = new QLearning(learningRate, traces, qTable);
		//qLearning = new QLearning(learningRate, discountFactor, qTable);
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
		} else {
			currentState = ModeledBumperState.initialize(currentPercept);
		}

		if (previousAction != null) {
			DiscretizedTransitionReward transition = transitionDiscretizer.discretizeAndComputeReward(previousState, previousAction, currentState);
			qLearning.update(transition);
			accumulatedReward += transition.getReward();
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