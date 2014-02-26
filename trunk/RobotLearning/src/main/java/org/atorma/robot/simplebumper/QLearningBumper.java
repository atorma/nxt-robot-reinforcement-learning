package org.atorma.robot.simplebumper;

import org.atorma.robot.EpsilonGreedyPolicy;
import org.atorma.robot.communications.ActionIdProvider;
import org.atorma.robot.learning.QLearning;
import org.atorma.robot.learning.Transition;

public class QLearningBumper implements ActionIdProvider {
	
	private BumperStateIdMap stateIdMap = new BumperStateIdMap();
	private BumperRewardFunction rewardFunction = new BumperRewardFunction();
	private double learningRate = 0.1;
	private double discountFactor = 0.9;
	private QLearning qLearning;
	
	private double epsilon = 0.1;
	private EpsilonGreedyPolicy epsilonGreedyPolicy;
	
	private BumperState previousState;
	private BumperAction previousAction;
	
	public QLearningBumper() {
		
		qLearning = new QLearning(stateIdMap, rewardFunction, learningRate, discountFactor);
		
		epsilonGreedyPolicy = new EpsilonGreedyPolicy(new int[] {
				BumperAction.FORWARD.getId(), BumperAction.BACKWARD.getId(),
				BumperAction.LEFT.getId(), BumperAction.RIGHT.getId()}, 
				epsilon);
	}
	
	
	// One step of Q-learning is so fast that there's no point in doing learning in a separate thread
	@Override
	public int getActionId(double[] state) {
		BumperState currentState = new BumperState(state);
		BumperAction currentAction = BumperAction.getAction(epsilonGreedyPolicy.getActionId(stateIdMap.getId(state)));
		System.out.println("State: " + currentState);
		System.out.println("Action: " + currentAction);

		if (previousState != null) {
			Transition transition = new Transition(previousState, previousAction, currentState);
			qLearning.update(transition);
			System.out.println("Total reward: " + qLearning.getAccumulatedReward());
			epsilonGreedyPolicy.setDeterministicPolicy(qLearning.getLearnedPolicy());
		}
		
		previousState = currentState;
		previousAction = currentAction;
		
		return currentAction.getId();
			
	}

}
