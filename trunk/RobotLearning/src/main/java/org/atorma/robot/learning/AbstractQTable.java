package org.atorma.robot.learning;

import java.util.*;

import org.atorma.robot.mdp.DiscretizedStateAction;

public abstract class AbstractQTable implements QTable {

	private Random random = new Random();
	
	
	protected abstract Set<Integer> getActionIds();
	
	@Override
	public DiscretizedStateAction getBestActionInState(int stateId) {
		Double bestActionValue = null;
		List<Integer> bestActions = new ArrayList<>();
		
		for (int actionId : getActionIds()) {
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
