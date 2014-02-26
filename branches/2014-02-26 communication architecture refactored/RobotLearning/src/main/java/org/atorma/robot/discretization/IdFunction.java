package org.atorma.robot.discretization;

public interface IdFunction {

	/**
	 * Returns a unique id for the given vector value.
	 */
	int getId(double[] value);
	
	
}
