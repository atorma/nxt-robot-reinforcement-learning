package org.atorma.robot.policy;

public interface DiscretePolicy {

	/**
	 * @return action id for given state id, or <tt>null</tt> if action undefined
	 */
	Integer getActionId(int stateId);

}
