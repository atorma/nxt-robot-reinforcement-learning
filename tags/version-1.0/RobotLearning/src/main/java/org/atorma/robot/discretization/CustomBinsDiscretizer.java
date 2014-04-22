package org.atorma.robot.discretization;

import java.util.Arrays;

/**
 * A discretizer with bins defined by custom divider points:
 * [-Inf,dividers[0]), [dividers[0],dividers[1]), ... , [dividers[n-1],Inf) 
 * where values in the first interval are discretized to 0.
 */
public class CustomBinsDiscretizer implements Discretizer {
	
	private final double[] bins;

	public CustomBinsDiscretizer(double[] dividers) {
		if (dividers == null) {
			throw new NullPointerException();
		}
		Arrays.sort(dividers);
		this.bins = dividers;
	}
	

	@Override
	public int discretize(double value) {
		
		int index = Arrays.binarySearch(bins, value);
		if (index >= 0) {
			return index + 1;
		} else {
			return -index - 1;
		}
	}

	@Override
	public int getNumberOfBins() {
		return bins.length + 1;
	}

}
