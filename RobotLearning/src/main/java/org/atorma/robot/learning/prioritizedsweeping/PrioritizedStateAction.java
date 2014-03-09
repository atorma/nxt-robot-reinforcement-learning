package org.atorma.robot.learning.prioritizedsweeping;

import org.atorma.robot.mdp.DiscretizedStateAction;
import org.atorma.robot.mdp.StateAction;

class PrioritizedStateAction implements Comparable<PrioritizedStateAction> {
	
	final StateAction stateAction;
	final DiscretizedStateAction discretization;
	final double priority;
	
	public PrioritizedStateAction(StateAction stateAction, DiscretizedStateAction discretization, double priority) {
		this.stateAction = stateAction;
		this.discretization = discretization;
		this.priority = priority;
	}

	@Override
	public int compareTo(PrioritizedStateAction o) {
		return (int) Math.signum(this.priority - o.priority);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result
				+ ((discretization == null) ? 0 : discretization.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		PrioritizedStateAction other = (PrioritizedStateAction) obj;
		if (discretization == null) {
			if (other.discretization != null)
				return false;
		} else if (!discretization.equals(other.discretization))
			return false;
		return true;
	}
	
	
}