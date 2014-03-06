package org.atorma.robot.learning;

import java.util.*;

import org.atorma.robot.discretization.VectorDiscretizer;
import org.atorma.robot.mdp.*;


public class PrioritizedSweeping {

	private QTable qTable;
	private MarkovModel model;
	private StateActionQueue stateActionQueue = new StateActionQueue();
	private VectorDiscretizer stateDiscretizer;
	private double discountFactor = 1.0;
	private double qValueChangeThreshold = 0.01;

	
	public void setCurrentStateAction(StateAction stateAction) {
		PrioritzedStateAction prioritized = new PrioritzedStateAction(stateAction, Double.MIN_VALUE);
		stateActionQueue.add(prioritized);
	}
	
	protected PriorityQueue<PrioritzedStateAction> getPriorityQueue() {
		return stateActionQueue;
	}
	
	public void performIterations(int num) {
		for (int i = 0; i < num; i++) {
			
			if (stateActionQueue.isEmpty()) {
				return;
			}
			
			StateAction stateAction = stateActionQueue.poll().stateAction;
			int stateId = stateDiscretizer.getId(stateAction.getState().getValues());
			int actionId = stateAction.getAction().getId();
			DiscretizedStateAction stateActionId = new DiscretizedStateAction(stateId, actionId);
			double oldQ = qTable.getValue(stateActionId);
			double maxQ = qTable.getMaxValueForState(stateId);
			
			
			Set<StochasticTransitionReward> transitions = model.getOutgoingTransitions(stateAction);
			if (!transitions.isEmpty()) { 
				break;
			}
			
			double updatedQ = 0;
			for (StochasticTransitionReward tr : transitions) {
				int toStateId = stateDiscretizer.getId(tr.getToState().getValues());
				updatedQ += tr.getProbability() * ( tr.getReward() + discountFactor*qTable.getMaxValueForState(toStateId) );
			}
			qTable.setValue(stateActionId, updatedQ);
			
			double qValueChange = Math.abs(updatedQ - oldQ);
			if (updatedQ >= maxQ && qValueChange > qValueChangeThreshold) {
				for (StochasticTransitionReward predecessor : model.getIncomingTransitions(stateAction.getState())) {
					double priority = qValueChange * predecessor.getProbability();
					if (priority > qValueChangeThreshold) {
						stateActionQueue.removeStateAction(predecessor.getFromStateAction());
						stateActionQueue.add(new PrioritzedStateAction(predecessor.getFromStateAction(), -priority));
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

	public MarkovModel getModel() {
		return model;
	}

	public void setModel(MarkovModel model) {
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


	@SuppressWarnings("serial")
	protected class StateActionQueue extends PriorityQueue<PrioritzedStateAction> {
		
		public boolean removeStateAction(StateAction stateAction) {
			
			DiscretizedStateAction stateIdActionId = new DiscretizedStateAction(
					PrioritizedSweeping.this.stateDiscretizer.getId(stateAction.getState().getValues()), 
					stateAction.getAction().getId()); 
			
			Iterator<PrioritzedStateAction> iter = this.iterator();
			while (iter.hasNext()) {
				PrioritzedStateAction element  = iter.next();
				if (element.stateIdActionId.equals(stateIdActionId)) {
					iter.remove();
					return true;
				}
			}
			return false;
		}
		
	}

	protected class PrioritzedStateAction implements Comparable<PrioritzedStateAction> {
		private DiscretizedStateAction stateIdActionId;
		private StateAction stateAction;
		private double priority;
		
		private PrioritzedStateAction(StateAction stateAction, double priority) {
			this.stateIdActionId = new DiscretizedStateAction(
					PrioritizedSweeping.this.stateDiscretizer.getId(stateAction.getState().getValues()), 
					stateAction.getAction().getId()); 
			this.stateAction = stateAction;
			this.priority = priority;
		}

		public StateAction getStateAction() {
			return stateAction;
		}
		
		public DiscretizedStateAction getStateIdActionId() {
			return stateIdActionId;
		}

		public double getPriority() {
			return priority;
		}

		@Override
		public int compareTo(PrioritzedStateAction o) {
			return (int) Math.signum(this.priority - o.priority);
		}
	
		
		
	}

	




	


	
}
