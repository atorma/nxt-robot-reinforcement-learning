package org.atorma.robot;

import java.util.HashMap;

public class StateIdToActionIdMap extends HashMap<Integer, Integer> implements DiscretePolicy {
	
	private static final long serialVersionUID = -4602441174721502396L;

	@Override
	public Integer getActionId(int stateId) {
		return this.get(stateId);
	}

}
