package org.atorma.robot.learning;

import java.util.Iterator;
import java.util.PriorityQueue;

import org.atorma.robot.discretization.VectorDiscretizer;
import org.atorma.robot.mdp.*;


public class PrioritizedSweeping {

	private DiscreteQFunction qFunction;
	private MarkovModel model;
	private StateActionQueue stateActionQueue = new StateActionQueue();
	private VectorDiscretizer stateDiscretizer;
	private double discountFactor;
	private double qValueChangeThreshold = 0.01;

	public void setCurrentStateAction(StateAction<?, ?> stateAction) {
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
			double oldQ = qFunction.getValue(stateActionId);
			double maxQ = qFunction.getMaxValueForState(stateId);
			
			double updatedQ = 0; // TODO this will set q-value to 0 if no transition from stateAction. OK?
			for (StochasticTransitionWithReward tr : model.getTransitions(stateAction)) {
				int toStateId = stateDiscretizer.getId(tr.getToState().getValues());
				updatedQ += tr.getProbability() * ( tr.getReward() + discountFactor*qFunction.getMaxValueForState(toStateId) );
			}
			qFunction.setValue(stateActionId, updatedQ);
			
			double qValueChange = Math.abs(updatedQ - oldQ);
			if (updatedQ >= maxQ && qValueChange > qValueChangeThreshold) {
				for (StateAction predecessor : model.getPredecessors(stateAction.getState())) {
					double priority = qValueChange * model.getTransitionProbability(new Transition<State, DiscreteAction>(predecessor, stateAction.getState()));
					if (priority > qValueChangeThreshold) {
						stateActionQueue.removeStateAction(predecessor);
						stateActionQueue.add(new PrioritzedStateAction(predecessor, -priority));
					}
				}
			}
		}
	}
	

	public double getDiscountFactor() {
		return discountFactor;
	}

	public void setDiscountFactor(double discountFactor) {
		this.discountFactor = discountFactor;
	}

	public void setQFunction(DiscreteQFunction qFunction) {
		this.qFunction = qFunction;
	}

	public void setModel(MarkovModel model) {
		this.model = model;
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
		private StateAction<?, ?> stateAction;
		private double priority;
		
		private PrioritzedStateAction(StateAction<?, ?> stateAction, double priority) {
			this.stateIdActionId = new DiscretizedStateAction(
					PrioritizedSweeping.this.stateDiscretizer.getId(stateAction.getState().getValues()), 
					stateAction.getAction().getId()); 
			this.stateAction = stateAction;
			this.priority = priority;
		}

		public StateAction<?, ?> getStateAction() {
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
