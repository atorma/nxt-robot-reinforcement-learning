package org.atorma.robot.policy;

import java.util.Random;

import org.atorma.robot.mdp.DiscreteAction;

public class EpsilonGreedyPolicy implements DiscretePolicy {

	private DiscretePolicy deterministicPolicy;
	private Random random = new Random();
	private double epsilon = 0.1;
	private int[] allActionIds;
	
	public EpsilonGreedyPolicy(double epsilon, DiscretePolicy deterministicPolicy, int... allActionIds) {
		this.allActionIds = allActionIds;
		this.epsilon = epsilon;
		this.deterministicPolicy = deterministicPolicy;
	}
		
	public EpsilonGreedyPolicy(double epsilon, DiscretePolicy deterministicPolicy, DiscreteAction... allActions) {
		this.allActionIds = new int[allActions.length]; 
		for (int i=0; i<allActions.length; i++) {
			this.allActionIds[i] = allActions[i].getId();
		}
		this.epsilon = epsilon;
		this.deterministicPolicy = deterministicPolicy;
	}

	/**
	 * Returns the id of an action given a state id according to epsilon greedy policy.
	 * The action id is never null.
	 */
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
