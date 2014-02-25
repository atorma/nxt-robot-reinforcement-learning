package org.atorma.robot.simplebumper;

import org.atorma.robot.PolicyIdMap;
import org.atorma.robot.communications.CommunicationException;
import org.atorma.robot.communications.RobotCommunications;
import org.atorma.robot.communications.StateAndAction;
import org.atorma.robot.learning.QLearning;
import org.atorma.robot.learning.Transition;

public class QLearningBumper implements Runnable {
	private RobotCommunications comms;
	
	private BumperStateIdMap stateIdMap = new BumperStateIdMap();
	private BumperActionIdMap actionIdMap = new BumperActionIdMap();
	private BumperRewardFunction rewardFunction = new BumperRewardFunction();
	private double learningRate = 0.1;
	private double discountFactor = 0.9;
	private QLearning qLearning;
	private StateAndAction previousStateAndAction;
	
	public QLearningBumper(RobotCommunications comms) {
		this.comms = comms;
		qLearning = new QLearning(stateIdMap, actionIdMap, rewardFunction, learningRate, discountFactor);
	}

	@Override
	public void run() {
		
		previousStateAndAction = comms.takeStateAndAction();
		
		while (true) {
			try {
				//for (int updates = 0; updates < 20; updates++) {
					qLearning.update(getNextTransition());
					System.out.println("Total reward " + qLearning.getAccumulatedReward());
				//}
				comms.updatePolicy((PolicyIdMap) qLearning.getLearnedPolicy().clone());
			} catch (CommunicationException e) {
				System.err.println("Communication error");
			}
		}
		
	}
	
	private Transition getNextTransition() {
		
		StateAndAction stateAndAction = comms.takeStateAndAction();
		
		BumperState fromState = new BumperState(previousStateAndAction.getStateValues());
		BumperAction action = new BumperAction(previousStateAndAction.getActionValues());
		BumperState toState = new BumperState(stateAndAction.getStateValues()); 
		Transition transition = new Transition(fromState, action, toState);
		
		System.out.println(toState);
		
		previousStateAndAction = stateAndAction;
		
		return transition;
	}

}
