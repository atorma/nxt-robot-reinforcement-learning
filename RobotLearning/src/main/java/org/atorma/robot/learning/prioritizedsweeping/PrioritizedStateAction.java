package org.atorma.robot.learning.prioritizedsweeping;

import org.atorma.robot.mdp.StateAction;

class PrioritizedStateAction implements Comparable<PrioritizedStateAction> {
	
	final StateAction stateAction;
	final double priority;
	
	public PrioritizedStateAction(StateAction stateAction, double priority) {
		this.stateAction = stateAction;
		this.priority = priority;
	}

	@Override
	public int compareTo(PrioritizedStateAction o) {
		return (int) Math.signum(this.priority - o.priority);
	}
	
}