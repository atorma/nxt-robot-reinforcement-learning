package org.atorma.robot.learning.prioritizedsweeping;

import java.util.Set;

import org.atorma.robot.discretization.StateDiscretizer;
import org.atorma.robot.learning.HashMapQTable;
import org.atorma.robot.learning.QTable;
import org.atorma.robot.mdp.*;
import org.atorma.robot.policy.DiscretePolicy;


public class PrioritizedSweeping implements DiscretePolicy {

	private QTable qTable;
	private PrioritizedSweepingModel model;
	private StateActionPriorityQueue stateActionQueue = new StateActionPriorityQueue();
	private StateDiscretizer stateDiscretizer;
	private double discountFactor = 1.0;
	private double qValueChangeThreshold = 0.01;

	private StateAction sweepStartStateAction;
	
	public void setSweepStartStateAction(StateAction stateAction) {
		this.sweepStartStateAction = stateAction;
	}

	public void performIterations(int num) {
		for (int i = 0; i < num; i++) {
			
			if (stateActionQueue.isEmpty() && sweepStartStateAction == null) {
				return;
			}
			
			StateAction stateAction;
			if (sweepStartStateAction != null) {
				stateAction = sweepStartStateAction;
				sweepStartStateAction = null;
			} else {
				stateAction = stateActionQueue.poll();
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
			double maxQ = qTable.getMaxValueInState(stateId);
			
			if (updatedQ == maxQ && qValueChange > qValueChangeThreshold) {
				for (StochasticTransitionReward predecessor : model.getIncomingTransitions(stateAction.getState())) {
					double priority = qValueChange * predecessor.getProbability();
					if (priority > qValueChangeThreshold) {
						stateActionQueue.addOrUpdate(predecessor.getFromStateAction(), -priority);
					}
				}
			}
		}
	}
	
	public void updateModel(TransitionReward transitionReward) {
		model.updateModel(transitionReward);
	}
	
	
	@Override
	public Integer getActionId(int stateId) {
		return qTable.getActionId(stateId);
	}

	
	public double getDiscountFactor() {
		return discountFactor;
	}

	public void setDiscountFactor(double discountFactor) {
		this.discountFactor = discountFactor;
	}

	public PrioritizedSweepingModel getModel() {
		return model;
	}

	public void setModel(PrioritizedSweepingModel model) {
		this.model = model;
		HashMapQTable qTable = new HashMapQTable();
		for (DiscreteAction action : model.getAllActions()) {
			qTable.addActionId(action.getId());
		}
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
	
}
