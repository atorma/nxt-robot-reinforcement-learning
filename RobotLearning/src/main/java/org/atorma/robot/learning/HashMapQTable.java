package org.atorma.robot.learning;

import java.util.*;

import org.atorma.robot.mdp.DiscreteAction;
import org.atorma.robot.mdp.DiscretizedStateAction;

/**
 * A Q-table that does not need to know state and actions ids beforehand
 * and they are not restricted to positive integers.
 */
public class HashMapQTable extends AbstractQTable {	
	public static final double DEFAULT_Q_VALUE = 0;
	
	private Map<DiscretizedStateAction, Double> qTable = new HashMap<>();
	private double defaultQValue = DEFAULT_Q_VALUE;
	private Set<Integer> stateIds = new HashSet<>();
	private Set<Integer> actionIds = new HashSet<>();
	
	public HashMapQTable() {
	}
	
	public HashMapQTable(double defaultQValue) {
		this.defaultQValue = defaultQValue;
	}
	
	public HashMapQTable(double defaultQValue, int... actionIds) {
		this.defaultQValue = defaultQValue;
		for (int actionId : actionIds) {
			addActionId(actionId);
		}
	}
	
	public HashMapQTable(double defaultQValue, DiscreteAction... actions) {
		this.defaultQValue = defaultQValue;
		for (DiscreteAction action : actions) {
			addActionId(action.getId());
		}
	}
	
	
	public Set<Integer> getStateIds() {
		return Collections.unmodifiableSet(stateIds);
	}

	public void addStateId(int stateId) {
		this.stateIds.add(stateId);
	}
	
	@Override
	public Set<Integer> getActionIds() {
		return Collections.unmodifiableSet(actionIds);
	}
	
	public void addActionId(int actionId) {
		this.actionIds.add(actionId);
	}
	
	@Override
	public double getValue(DiscretizedStateAction stateIdActionId) {
		Double qValue = qTable.get(stateIdActionId);
		return qValue != null ? qValue : defaultQValue;
	}

	@Override
	public void setValue(DiscretizedStateAction stateIdActionId, double qValue) {
		this.stateIds.add(stateIdActionId.getStateId());
		this.actionIds.add(stateIdActionId.getActionId());
		qTable.put(stateIdActionId, qValue);
	}
	
}
