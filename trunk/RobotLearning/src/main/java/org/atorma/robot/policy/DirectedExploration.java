package org.atorma.robot.policy;

import java.util.*;

import org.atorma.robot.learning.QTable;
import org.atorma.robot.mdp.DiscretizedStateAction;

/**
 * A Q-Table decorator that gives an exploration bonus to the
 * underlying Q-table as in Bakker et al (2006).
 */
public class DirectedExploration implements QTable {
	
	private QTable qTable;
	private double bonusMultiplier;
	private double defaultCount;
	private int[] actionIds;
	private Map<DiscretizedStateAction, Integer> stepWhenLastTriedMap = new HashMap<>();
	private Map<DiscretizedStateAction, Integer> timesTriedMap = new HashMap<>();
	private int totalTimeSteps = 0;
	
	public DirectedExploration(QTable qTable, double bonusMultiplier, double defaultCount, int... actionIds) {
		this.qTable = qTable;
		this.bonusMultiplier = bonusMultiplier;
		this.defaultCount = defaultCount;
		this.actionIds = Arrays.copyOf(actionIds, actionIds.length);
	}
	

	@Override
	public Integer getActionId(int stateId) {
		DiscretizedStateAction stateIdActionId = getBestActionInState(stateId);
		return stateIdActionId != null ? stateIdActionId.getActionId() : null;
	}

	@Override
	public double getValue(DiscretizedStateAction stateIdActionId) {
		int stepWhenLastTried = stepWhenLastTriedMap.get(stateIdActionId) != null ? stepWhenLastTriedMap.get(stateIdActionId) : 0;
		int stepsSinceLastTried = totalTimeSteps - stepWhenLastTried;
		
		double timesTried = timesTriedMap.get(stateIdActionId) != null ? timesTriedMap.get(stateIdActionId) : defaultCount;
		
		return qTable.getValue(stateIdActionId) + bonusMultiplier*Math.sqrt(stepsSinceLastTried)/timesTried;
	}

	@Override
	public void setValue(DiscretizedStateAction stateIdActionId, double qValue) {
		qTable.setValue(stateIdActionId, qValue);
	}

	@Override
	public double getMaxValueInState(int stateId) {
		return getValue(getBestActionInState(stateId));
	}

	@Override
	public DiscretizedStateAction getBestActionInState(int stateId) {
		DiscretizedStateAction bestStateAction = null;
		double bestQValue = Double.MIN_VALUE; 
		for (int actionId : actionIds) {
			DiscretizedStateAction stateIdActionId = new DiscretizedStateAction(stateId, actionId);
			double qValue = getValue(stateIdActionId);
			if (qValue > bestQValue) {
				bestQValue = qValue;
				bestStateAction = stateIdActionId;
			}
		}
		return bestStateAction;
	}


	public void recordStateAction(DiscretizedStateAction stateIdActionId) {
		totalTimeSteps++;
		stepWhenLastTriedMap.put(stateIdActionId, totalTimeSteps);
		
		Integer timesTried = timesTriedMap.get(stateIdActionId);
		if (timesTried == null) {
			timesTriedMap.put(stateIdActionId, 1);
		} else {
			timesTriedMap.put(stateIdActionId, timesTried + 1);
		}
	}

}
