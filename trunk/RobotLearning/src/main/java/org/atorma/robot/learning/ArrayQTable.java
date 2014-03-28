package org.atorma.robot.learning;

import java.util.*;

import org.atorma.robot.mdp.DiscretizedStateAction;

/**
 * A Q-table that stores the values in a 2D array. State and action ids
 * are required to be integers in the range [0..numStates), [0..numActions).
 */
public class ArrayQTable implements QTable {
	
	public static final double DEFAULT_Q_VALUE = 0;
	
	private final double defaultQValue;
	private final int numStates;
	private final int numActions;
	private double[][] qTable;
	private Random random = new Random();

	public ArrayQTable(int numStates, int numActions) {
		this(numStates, numActions, DEFAULT_Q_VALUE);
	}
	
	public ArrayQTable(int numStates, int numActions, double defaultQValue) {
		if (numStates <= 0 || numActions <= 0) {
			throw new IllegalArgumentException();
		}
		this.numStates = numStates;
		this.numActions = numActions;
		this.defaultQValue = defaultQValue;
		
		initQTable();
	}

	private void initQTable() {
		this.qTable = new double[numStates][numActions];
		for (int i = 0; i < numStates; i++) {
			for (int j = 0; j < numActions; j++) {
				qTable[i][j] = defaultQValue;
			}
		}
	}
	
	@Override
	public double getValue(DiscretizedStateAction stateIdActionId) {
		return qTable[stateIdActionId.getStateId()][stateIdActionId.getActionId()];
	}

	@Override
	public void setValue(DiscretizedStateAction stateIdActionId, double qValue) {
		qTable[stateIdActionId.getStateId()][stateIdActionId.getActionId()] = qValue;
	}
	
	@Override
	public DiscretizedStateAction getBestActionInState(int stateId) {
		Double bestActionValue = null;
		List<Integer> bestActions = new ArrayList<>();
		
		for (int actionId = 0; actionId < numActions; actionId++) {
			double q = getValue(new DiscretizedStateAction(stateId, actionId));
			if (bestActionValue == null || q > bestActionValue) {
				bestActionValue = q;
				bestActions.clear();
				bestActions.add(actionId);
			} else if (q == bestActionValue) {
				bestActions.add(actionId);
			}
		}
		
		Integer bestActionId = bestActions.get(random.nextInt(bestActions.size()));
		
		return new DiscretizedStateAction(stateId, bestActionId);
	}
	
	@Override
	public double getMaxValueInState(int stateId) {
		return getValue(getBestActionInState(stateId));
	}

	@Override
	public Integer getActionId(int stateId) {
		return getBestActionInState(stateId).getActionId();
	}

	
}
