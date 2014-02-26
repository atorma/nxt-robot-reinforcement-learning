package org.atorma.robot.discretization;

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
		this.bins = bins;
	}
	

	@Override
	public int discretize(double value) {
		if (bins.length == 0) {
			return 0;
		}
		
		for (int i = 0; i < bins.length; i++) {
			if (value < bins[i]) {
				return i;
			}
		}
		
		return bins.length;
	}

	@Override
	public int getNumberOfBins() {
		return bins.length + 1;
	}

}
