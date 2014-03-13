package org.atorma.robot.learning.prioritizedsweeping;

import java.util.Set;

import org.atorma.robot.discretization.StateDiscretizer;
import org.atorma.robot.learning.HashMapQTable;
import org.atorma.robot.learning.QTable;
import org.atorma.robot.mdp.*;
import org.atorma.robot.policy.DiscretePolicy;


public class PrioritizedSweeping implements DiscretePolicy {
	
	public static final double DEFAULT_Q_VALUE_CHANGE_THRESHOLD = 1E-5;
	public static final double DEFAULT_DISCOUNT_FACTOR = 1.0;

	private QTable qTable;
	private PrioritizedSweepingModel model;
	private StateDiscretizer stateDiscretizer;
	private DiscretizingStateActionPriorityQueue stateActionQueue;
	private double discountFactor = DEFAULT_DISCOUNT_FACTOR;
	private double qValueChangeThreshold = DEFAULT_Q_VALUE_CHANGE_THRESHOLD;
	boolean isInitialized = false;

	private StateAction sweepStartStateAction;
	
	
	public void setSweepStartStateAction(StateAction stateAction) {
		this.sweepStartStateAction = stateAction;
	}
	
	public void clearPriorityQueue() {
		this.stateActionQueue = new DiscretizingStateActionPriorityQueue(stateDiscretizer);
	}

	public int performIterations(int num) {
		if (!isInitialized) {
			initialize();
		}
		
		int i;
		for (i = 0; i < num; i++) {
			
			if (stateActionQueue.isEmpty() && sweepStartStateAction == null) {
				return 0;
			}
			
			StateAction stateAction;
			if (sweepStartStateAction != null) {
				stateAction = sweepStartStateAction;
				sweepStartStateAction = null;
			} else {
				stateAction = stateActionQueue.pollMin();
			}
			
			Set<StochasticTransitionReward> transitions = model.getOutgoingTransitions(stateAction);
			if (transitions.isEmpty()) { 
				break;
			}
			
			int stateId = stateDiscretizer.getId(stateAction.getState());
			int actionId = stateAction.getAction().getId();
			DiscretizedStateAction stateActionId = new DiscretizedStateAction(stateId, actionId);
			double oldQ = qTable.getValue(stateActionId);
			
			double updatedQ = 0;
			for (StochasticTransitionReward tr : transitions) {
				int toStateId = stateDiscretizer.getId(tr.getToState());
				updatedQ += tr.getProbability() * ( tr.getReward() + discountFactor*qTable.getMaxValueInState(toStateId) );
			}
			qTable.setValue(stateActionId, updatedQ);
				
			double qValueChange = Math.abs(updatedQ - oldQ);
			
			if (qValueChange > qValueChangeThreshold) {
				for (StochasticTransitionReward predecessor : model.getIncomingTransitions(stateAction.getState())) {
					if (qValueChange * predecessor.getProbability() > qValueChangeThreshold) {
						// Our priority queue stores minimum priority first and priority must be integer
						int priority = (int) Math.round(-qValueChange * predecessor.getProbability() / qValueChangeThreshold);
						stateActionQueue.addOrDecreasePriority(predecessor.getFromStateAction(), priority);
					}
				}
			}
		}
		return i;
	}
	
	private void initialize() {
		this.stateActionQueue = new DiscretizingStateActionPriorityQueue(stateDiscretizer);
		
		if (qTable instanceof HashMapQTable) { // a bit ugly...
			for (DiscreteAction action : model.getAllActions()) {
				((HashMapQTable) qTable).addActionId(action.getId());
			}
		}
		
		isInitialized = true;
	}

	public void updateModel(TransitionReward transitionReward) {
		model.updateModel(transitionReward);
	}
	
	
	@Override
	public Integer getActionId(int stateId) {
		if (!isInitialized) {
			initialize();
		}
		return qTable.getActionId(stateId);
	}

	public double getDiscountFactor() {
		return discountFactor;
	}

	public void setDiscountFactor(double discountFactor) {
		if (discountFactor < 0 || discountFactor > 1) {
			throw new IllegalArgumentException("Illegal discount factor " + discountFactor);
		}
		this.discountFactor = discountFactor;
	}

	public PrioritizedSweepingModel getModel() {
		return model;
	}

	public void setModel(PrioritizedSweepingModel model) {
		this.model = model;
	}
	
	public QTable getQTable() {
		return qTable;
	}

	public void setQTable(QTable qTable) {
		this.qTable = qTable;
	}

	public StateDiscretizer getStateDiscretizer() {
		return stateDiscretizer;
	}

	public void setStateDiscretizer(StateDiscretizer stateDiscretizer) {
		this.stateDiscretizer = stateDiscretizer;
	}

	public double getQValueChangeThreshold() {
		return qValueChangeThreshold;
	}

	public void setQValueChangeThreshold(double qValueChangeThreshold) {
		this.qValueChangeThreshold = qValueChangeThreshold;
	}

	public DiscretizingStateActionPriorityQueue getPriorityQueue() {
		return this.stateActionQueue;
	}

	
	
}
