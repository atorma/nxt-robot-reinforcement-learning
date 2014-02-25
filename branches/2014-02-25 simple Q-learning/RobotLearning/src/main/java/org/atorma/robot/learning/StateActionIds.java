package org.atorma.robot.learning;

public class StateActionIds {

	private final int stateId;
	private final int actionId;
	
	public StateActionIds(int stateId, int actionId) {
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
		StateActionIds other = (StateActionIds) obj;
		if (actionId != other.actionId)
			return false;
		if (stateId != other.stateId)
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "StateActionIds [stateId=" + stateId + ", actionId=" + actionId
				+ "]";
	}
	
	
	
}
