package org.atorma.robot.mdp;

public class DiscretizedStateAction {

	private final int stateId;
	private final int actionId;
	
	public DiscretizedStateAction(int stateId, int actionId) {
		this.stateId = stateId;
		this.actionId = actionId;
	}

	public int getStateId() {
		return stateId;
	}

	public int getActionId() {
		return actionId;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + actionId;
		result = prime * result + stateId;
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
		DiscretizedStateAction other = (DiscretizedStateAction) obj;
		if (actionId != other.actionId)
			return false;
		if (stateId != other.stateId)
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "StateIdActionId [stateId=" + stateId + ", actionId=" + actionId + "]";
	}
	
	
	
}
