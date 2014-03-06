package org.atorma.robot.learning.prioritizedsweeping;

import java.util.*;

import org.atorma.robot.discretization.VectorDiscretizer;
import org.atorma.robot.learning.QTable;
import org.atorma.robot.mdp.*;


public class PrioritizedSweeping {

	private QTable qTable;
	private PrioritizedSweepingModel model;
	private StateActionPriorityQueue stateActionQueue = new StateActionPriorityQueue();
	private VectorDiscretizer stateDiscretizer;
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
			
			int stateId = stateDiscretizer.getId(stateAction.getState().getValues());
			int actionId = stateAction.getAction().getId();
			DiscretizedStateAction stateActionId = new DiscretizedStateAction(stateId, actionId);
			double oldQ = qTable.getValue(stateActionId);
			
			double updatedQ = 0;
			for (StochasticTransitionReward tr : transitions) {
				int toStateId = stateDiscretizer.getId(tr.getToState().getValues());
				updatedQ += tr.getProbability() * ( tr.getReward() + discountFactor*qTable.getMaxValueForState(toStateId) );
			}
			qTable.setValue(stateActionId, updatedQ);
				
			double qValueChange = Math.abs(updatedQ - oldQ);
			double maxQ = qTable.getMaxValueForState(stateId);
			
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
	

	public double getDiscountFactor() {
		return discountFactor;
	}

	public void setDiscountFactor(double discountFactor) {
		this.discountFactor = discountFactor;
	}

	public QTable getQTable() {
		return qTable;
	}

	public void setQTable(QTable qFunction) {
		this.qTable = qFunction;
	}

	public PrioritizedSweepingModel getModel() {
		return model;
	}

	public void setModel(PrioritizedSweepingModel model) {
		this.model = model;
	}

	public VectorDiscretizer getStateDiscretizer() {
		return stateDiscretizer;
	}

	public void setStateDiscretizer(VectorDiscretizer stateDiscretizer) {
		this.stateDiscretizer = stateDiscretizer;
	}

	public double getQValueChangeThreshold() {
		return qValueChangeThreshold;
	}

	public void setQValueChangeThreshold(double qValueChangeThreshold) {
		this.qValueChangeThreshold = qValueChangeThreshold;
	}
	
}
