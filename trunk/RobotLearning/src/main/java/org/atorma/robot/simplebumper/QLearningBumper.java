package org.atorma.robot.simplebumper;

import org.atorma.robot.DiscreteRobotController;
import org.atorma.robot.learning.*;
import org.atorma.robot.mdp.DiscretizedTransitionReward;
import org.atorma.robot.mdp.StateActionDiscretizer;
import org.atorma.robot.policy.EpsilonGreedyPolicy;

public class QLearningBumper implements DiscreteRobotController {
	
	private BumperPerceptDiscretizer stateDiscretizer = new BumperPerceptDiscretizer();
	private BumperRewardFunction rewardFunction = new BumperRewardFunction();
	private StateActionDiscretizer transitionDiscretizer = new StateActionDiscretizer(stateDiscretizer, rewardFunction);
	
	private double learningRate = 0.1;
	private double discountFactor = 0.9;
	private QTable qTable;
	private QLearning qLearning;
	
	private double epsilon = 0.1;
	private EpsilonGreedyPolicy epsilonGreedyPolicy;
	
	private BumperPercept previousState;
	private BumperAction previousAction;
	
	private int accumulatedCollisions = 0;
	private double accumulatedReward = 0;
	
	private BumperLogWriter logWriter;
	
	
	public QLearningBumper(String logFile) {
		this();
		logWriter = new BumperLogWriter(logFile); 
	}
	
	public QLearningBumper() {
		qTable = new ArrayQTable(stateDiscretizer.getNumberOfStates(), BumperAction.values().length);
		qLearning = new QLearning(learningRate, discountFactor, qTable);
		epsilonGreedyPolicy = new EpsilonGreedyPolicy(epsilon, qLearning, BumperAction.values());
	}
	
	
	// One step of Q-learning is so fast that there's no point in doing learning in a separate thread
	@Override
	public int getActionId(double[] state) {
		BumperPercept currentState = new BumperPercept(state);
		if (currentState.isCollided()) {
			accumulatedCollisions++;
		}
	
		if (previousState != null) {
			DiscretizedTransitionReward transition = transitionDiscretizer.discretizeAndComputeReward(previousState, previousAction, currentState);
			qLearning.update(transition);
			accumulatedReward += transition.getReward();
		}

		BumperAction action = BumperAction.getAction(epsilonGreedyPolicy.getActionId(stateDiscretizer.getId(currentState)));
		
		if (logWriter != null) {
			logWriter.log(accumulatedReward, accumulatedCollisions, currentState.isCollided(), action);
		}
		
		previousState = currentState;
		previousAction = action;
		
		return action.getId();
			
	}

}
