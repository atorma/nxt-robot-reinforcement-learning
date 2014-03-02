package org.atorma.robot;

import org.atorma.robot.util.Ranmar;

public class EpsilonGreedyPolicy implements DiscretePolicy {

	private DiscretePolicy deterministicPolicy;
	private Ranmar random = new Ranmar(); // Using this instead of Java's Random because it's implementation in Lejos is buggy! 
	private double epsilon = 0.1;
	private int[] allActionIds;
	
	public EpsilonGreedyPolicy(double epsilon, int[] allActionIds) {
		this.allActionIds = allActionIds;
		this.epsilon = epsilon;
	}
		
	public EpsilonGreedyPolicy(double epsilon, DiscreteAction[] allActions) {
		this.allActionIds = new int[allActions.length]; 
		for (int i=0; i<allActions.length; i++) {
			this.allActionIds[i] = allActions[i].getId();
		}
		this.epsilon = epsilon;
	}

	@Override
	public Integer getActionId(int stateId) {
		if (random.nextFloat() < epsilon || deterministicPolicy == null || deterministicPolicy.getActionId(stateId) == null) {
			return allActionIds[random.nextInt(allActionIds.length)];
		} else {
			return deterministicPolicy.getActionId(stateId);
		}
	}
	
	

	public DiscretePolicy getDeterministicPolicy() {
		return deterministicPolicy;
	}

	public void setDeterministicPolicy(DiscretePolicy policy) {
		this.deterministicPolicy = policy;
	}

	public double getEpsilon() {
		return epsilon;
	}

	public void setEpsilon(double epsilon) {
		this.epsilon = epsilon;
	}
	
	
}