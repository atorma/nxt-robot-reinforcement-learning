package org.atorma.robot.learning;

import java.util.PriorityQueue;

import org.atorma.robot.mdp.*;


public class PrioritizedSweeping<S extends State, A extends DiscreteAction> {

	private DiscreteQFunction qFunction;
	private DiscreteStochasticModel<S, A> model;
	private PriorityQueue<StateAction<S, A>> stateActionQueue;
	
	public PrioritizedSweeping(DiscreteQFunction qFunction, DiscreteStochasticModel model) {
		this.qFunction = qFunction;
		this.model = model;
	}
	
	
	
}
