package org.atorma.robot.mdp;

public class DiscretizedTransition {

	private final int fromStateId;
	private final int byActionId;
	private final int toStateId;

	public DiscretizedTransition(int fromStateId, int byActionId, int toStateId) {
		this.fromStateId = fromStateId;
		this.byActionId = byActionId;
		this.toStateId = toStateId;
	}

	public int getFromStateId() {
		return fromStateId;
	}

	public int getByActionId() {
		return byActionId;
	}

	public int getToStateId() {
		return toStateId;
	}
	
	public DiscretizedStateAction getFromStateIdActionId() {
		return new DiscretizedStateAction(fromStateId, byActionId);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + byActionId;
		result = prime * result + fromStateId;
		result = prime * result + toStateId;
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
		DiscretizedTransition other = (DiscretizedTransition) obj;
		if (byActionId != other.byActionId)
			return false;
		if (fromStateId != other.fromStateId)
			return false;
		if (toStateId != other.toStateId)
			return false;
		return true;
	}
	
	
}