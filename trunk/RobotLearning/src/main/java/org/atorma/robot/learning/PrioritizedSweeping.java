package org.atorma.robot.learning;

import java.util.PriorityQueue;

import org.atorma.robot.discretization.VectorDiscretizer;
import org.atorma.robot.mdp.*;


public class PrioritizedSweeping {

	private DiscreteQFunction qFunction;
	private MarkovModel model;
	private PriorityQueue<PrioritzedStateAction> stateActionQueue = new PriorityQueue<>();
	private VectorDiscretizer stateDiscretizer;
	private double discountFactor;
	

	public void setCurrentStateAction(StateAction<?, ?> stateAction) {
		PrioritzedStateAction prioritized = new PrioritzedStateAction(stateAction, Double.MAX_VALUE);
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
			double updatedQ = 0;
			for (StochasticTransitionWithReward tr : model.getTransitions(stateAction)) {
				int toStateId = stateDiscretizer.getId(tr.getToState().getValues());
				updatedQ += tr.getProbability() * ( tr.getReward() + discountFactor*qFunction.getMaxValueForState(toStateId) );
			}
			qFunction.setValue(new DiscretizedStateAction(stateId, actionId), updatedQ);
		}
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




	protected class PrioritzedStateAction implements Comparable<PrioritzedStateAction> {
		private StateAction<?, ?> stateAction;
		private double priority;
		
		private PrioritzedStateAction(StateAction<?, ?> stateAction, double priority) {
			this.stateAction = stateAction;
			this.priority = priority;
		}

		public StateAction<?, ?> getStateAction() {
			return stateAction;
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
