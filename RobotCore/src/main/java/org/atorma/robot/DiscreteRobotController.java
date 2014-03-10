package org.atorma.robot;

public interface DiscreteRobotController {

	/**
	 * Method invoked by a robot when it requires an action from the controller.
	 * Learning can be hooked on this method.
	 */
	int getActionId(double[] stateValues);
}
