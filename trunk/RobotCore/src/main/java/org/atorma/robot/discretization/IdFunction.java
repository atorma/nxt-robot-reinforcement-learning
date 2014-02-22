package org.atorma.robot.discretization;

public interface IdFunction {

	/**
	 * Returns a unique id for the given value (vector).
	 */
	int getId(double[] value);
	
	
}
