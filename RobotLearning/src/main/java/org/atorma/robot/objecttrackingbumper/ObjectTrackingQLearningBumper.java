package org.atorma.robot.objecttrackingbumper;

import java.io.File;

import org.atorma.robot.DiscreteRobotController;
import org.atorma.robot.learning.QLearning;
import org.atorma.robot.logging.CsvLogWriter;
import org.atorma.robot.mdp.StateActionDiscretizer;
import org.atorma.robot.policy.EpsilonGreedyPolicy;
import org.atorma.robot.simplebumper.BumperAction;
import org.atorma.robot.simplebumper.BumperPercept;

public class ObjectTrackingQLearningBumper implements DiscreteRobotController {
	
	private BumperStateDiscretizer stateDiscretizer = new BumperStateDiscretizer();
	private BumperRewardFunction rewardFunction = new BumperRewardFunction();
	private StateActionDiscretizer transitionDiscretizer = new StateActionDiscretizer(stateDiscretizer, rewardFunction);
	
	private double learningRate = 0.1;
	private double discountFactor = 0.9;
	private QLearning qLearning;
	
	private double epsilon = 0.1;
	private EpsilonGreedyPolicy epsilonGreedyPolicy;
	
	private ModeledBumperState previousState;
	private BumperAction previousAction;

	private int accumulatedCollisions = 0;
	
	private CsvLogWriter logWriter;
	
	
	public ObjectTrackingQLearningBumper(String logFile) {
		this();
		logWriter = new CsvLogWriter(new File(logFile), "Accumulated reward", "Accumulated collisions"); 
	}
	
	public ObjectTrackingQLearningBumper() {
		qLearning = new QLearning(learningRate, discountFactor);
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
		//System.out.println(currentState);

		if (previousAction != null) {
			qLearning.update(transitionDiscretizer.discretizeAndComputeReward(previousState, previousAction, currentState));
			//System.out.println("Total reward: " + qLearning.getAccumulatedReward());
		}
		
		if (logWriter != null) {
			logWriter.addRow(qLearning.getAccumulatedReward(), accumulatedCollisions);
		}
		
		int currentStateId = stateDiscretizer.getId(currentState);
		BumperAction action = BumperAction.getAction(epsilonGreedyPolicy.getActionId(currentStateId));
		
		previousState = currentState;
		previousAction = action;
		
		return action.getId();
			
	}

}
