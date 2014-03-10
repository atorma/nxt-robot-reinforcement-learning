package org.atorma.robot.learning;

import java.util.*;

import org.atorma.robot.mdp.DiscretizedStateAction;

/**
 * A Q-table that does not need to know state and actions ids beforehand
 * and they are not restricted to positive integers.
 */
public class HashMapQTable extends HashMap<DiscretizedStateAction, Double> implements QTable {
	private static final long serialVersionUID = -1394200057253869720L;
	
	public static final double DEFAULT_Q_VALUE = 0;
	
	private double defaultQValue = DEFAULT_Q_VALUE;
	private Set<Integer> stateIds = new HashSet<>();
	private Set<Integer> actionIds = new HashSet<>();
	
	
	public HashMapQTable() {
	}
	
	public HashMapQTable(double defaultQValue) {
		this.defaultQValue = defaultQValue;
	}
	
	
	public Set<Integer> getStateIds() {
		return Collections.unmodifiableSet(stateIds);
	}

	public void addStateId(int stateId) {
		this.stateIds.add(stateId);
	}
	
	public void removestateId(int stateId) {
		this.stateIds.remove(stateId);
	}
	
	public Set<Integer> getActionIds() {
		return Collections.unmodifiableSet(actionIds);
	}
	
	public void addActionId(int actionId) {
		this.actionIds.add(actionId);
	}
	
	public void removeActionId(int actionId) {
		this.actionIds.remove(actionId);
	}
	
	
	@Override
	public double getValue(DiscretizedStateAction stateIdActionId) {
		Double qValue = this.get(stateIdActionId);
		return qValue != null ? qValue : defaultQValue;
	}

	@Override
	public void setValue(DiscretizedStateAction stateIdActionId, double qValue) {
		this.stateIds.add(stateIdActionId.getStateId());
		this.actionIds.add(stateIdActionId.getActionId());
		this.put(stateIdActionId, qValue);
	}
	
	@Override
	public DiscretizedStateAction getBestActionInState(int stateId) {
		if (this.actionIds.isEmpty()) {
			throw new IllegalStateException("No actions known");
		}
		
		Double bestActionValue = null;
		Integer bestActionId = null;
		
		for (int actionId : actionIds) {
			double q = getValue(new DiscretizedStateAction(stateId, actionId));
			if (bestActionValue == null || q > bestActionValue) {
				bestActionValue = q;
				bestActionId = actionId;
			}
		}
		
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
