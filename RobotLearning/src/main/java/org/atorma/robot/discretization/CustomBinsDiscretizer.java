package org.atorma.robot.discretization;

import java.util.Arrays;

/**
 * A discretizer with bin widths defined by user given divider points:
 * [-Inf,bins[0]), [bins[0],bins[1]), ... , [bins[n-1],Inf) 
 * where values in the first interval are discretized to 0.
 */
public class CustomBinsDiscretizer implements Discretizer {
	
	private final double[] bins;

	public CustomBinsDiscretizer(double[] bins) {
		if (bins == null) {
			throw new NullPointerException();
		}
		Arrays.sort(bins);
		this.bins = bins;
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
