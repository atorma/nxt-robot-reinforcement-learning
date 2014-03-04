package org.atorma.robot.learning;

import java.util.PriorityQueue;

import org.atorma.robot.discretization.VectorDiscretizer;
import org.atorma.robot.mdp.*;


public class PrioritizedSweeping<S extends State, A extends DiscreteAction> {

	private DiscreteQFunction qFunction;
	private MarkovModel<S, A> model;
	private PriorityQueue<PrioritzedStateAction> stateActionQueue = new PriorityQueue<>();
	private VectorDiscretizer stateDiscretizer;
	private double discountFactor;
	

	public void setCurrentStateAction(StateAction<S, A> stateAction) {
		PrioritzedStateAction prioritized = new PrioritzedStateAction(stateAction, Double.MAX_VALUE);
		stateActionQueue.add(prioritized);
	}
	
	protected PriorityQueue<PrioritzedStateAction> getPriorityQueue() {
		return stateActionQueue;
	}
	
	public void performIterations(int num) {
		// TODO Auto-generated method stub
		
	}
	
	public void setDiscountFactor(double discountFactor) {
		this.discountFactor = discountFactor;
	}

	public void setQFunction(DiscreteQFunction qFunction) {
		this.qFunction = qFunction;
	}

	public void setModel(MarkovModel<S, A> model) {
		this.model = model;
	}

	public void setStateDiscretizer(VectorDiscretizer stateDiscretizer) {
		this.stateDiscretizer = stateDiscretizer;
	}




	protected class PrioritzedStateAction implements Comparable<PrioritzedStateAction> {
		private StateAction<S, A> stateAction;
		private double priority;
		
		private PrioritzedStateAction(StateAction<S, A> stateAction, double priority) {
			this.stateAction = stateAction;
			this.priority = priority;
		}

		public StateAction<S, A> getStateAction() {
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
