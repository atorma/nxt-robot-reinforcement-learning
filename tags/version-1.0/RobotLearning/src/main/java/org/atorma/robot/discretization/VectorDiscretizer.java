package org.atorma.robot.discretization;

public interface VectorDiscretizer {

	/**
	 * Discretizes the given vector value
	 */
	int getId(double[] value);
	
	/**
	 * Returns the number of possible discretized values.
	 */
	int getNumberOfValues();
}
