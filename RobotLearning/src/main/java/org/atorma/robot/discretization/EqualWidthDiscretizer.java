package org.atorma.robot.discretization;

/**
 * Discretization where bin widths are equal and their ordinals are
 * 0 = [min, min+binWidth), 1 = [min+binWidth, min+2*binWidth), ..., nBins-1: [min+(nBins-1)*binWidth, min+nBins*binWidth).
 * Values smaller than the minimum are discretized to 0, larger than maximum to nBins-1.
 */
public class EqualWidthDiscretizer implements Discretizer {

	private final double min;
	private final double max;
	private final int nBins;
	private final double binWidth;

	public EqualWidthDiscretizer(double min, double max, int nBins) {
		if (nBins < 1) {
			throw new IllegalArgumentException("Number of bins must be >= 1");
		}
		
		if (min <= max) {
			this.min = min;
			this.max = max;
		} else {
			this.min = max;
			this.max = min;
		}
		
		this.nBins = nBins;
		this.binWidth = (max - min)/nBins;
	}
	
	@Override
	public int discretize(double value) {
		if (value <= min) {
			return 0;
		} else if (value >= max) {
			return nBins - 1;
		}
		
		return (int) ((value - min)/binWidth);
	}

	
	public double getMin() {
		return min;
	}

	public double getMax() {
		return max;
	}

	@Override
	public int getNumberOfBins() {
		return nBins;
	}

	public double getBinWidth() {
		return binWidth;
	}

	
	
	
	
	
	
}
