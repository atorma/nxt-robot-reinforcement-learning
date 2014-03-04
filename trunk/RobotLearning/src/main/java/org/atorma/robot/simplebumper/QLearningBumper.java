package org.atorma.robot.simplebumper;

import java.io.File;

import org.atorma.robot.*;
import org.atorma.robot.learning.QLearning;
import org.atorma.robot.logging.CsvLogWriter;
import org.atorma.robot.mdp.Transition;
import org.atorma.robot.policy.EpsilonGreedyPolicy;

public class QLearningBumper implements DiscreteRobotController {
	
	private BumperStateDiscretizer stateDiscretizer = new BumperStateDiscretizer();
	private BumperRewardFunction rewardFunction = new BumperRewardFunction();
	private double learningRate = 0.1;
	private double discountFactor = 0.9;
	private QLearning<BumperPercept, BumperAction> qLearning;
	
	private double epsilon = 0.1;
	private EpsilonGreedyPolicy epsilonGreedyPolicy;
	
	private BumperPercept previousState;
	private BumperAction previousAction;
	
	private int accumulatedCollisions = 0;
	
	private CsvLogWriter logWriter;
	
	
	public QLearningBumper(String logFile) {
		this();
		logWriter = new CsvLogWriter(new File(logFile), "Accumulated reward", "Accumulated collisions"); 
	}
	
	public QLearningBumper() {
		qLearning = new QLearning<>(stateDiscretizer, rewardFunction, learningRate, discountFactor);
		epsilonGreedyPolicy = new EpsilonGreedyPolicy(epsilon, qLearning, BumperAction.values());
	}
	
	
	// One step of Q-learning is so fast that there's no point in doing learning in a separate thread
	@Override
	public int getActionId(double[] state) {
		BumperPercept currentState = new BumperPercept(state);
		if (currentState.isCollided()) {
			accumulatedCollisions++;
		}
		BumperAction currentAction = BumperAction.getAction(epsilonGreedyPolicy.getActionId(stateDiscretizer.getId(state)));

		if (previousState != null) {
			Transition<BumperPercept, BumperAction> transition = new Transition<>(previousState, previousAction, currentState);
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

}
