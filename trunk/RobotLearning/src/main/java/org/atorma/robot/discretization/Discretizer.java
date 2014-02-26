package org.atorma.robot.discretization;

public interface Discretizer {

	/**
	 * Returns the bin ordinal [0..nBins-1] of the given value.
	 * Values smaller than the minimum are discretized to 0 and 
	 * larger than the maximum to nBins-1.
	 * 
	 * @param value
	 * @return discretization of the value to range [0..nBins-1]
	 */
	int discretize(double value);

	/**
	 * @return number of bins in this discretization (>= 1).
	 */
	int getNumberOfBins();

}