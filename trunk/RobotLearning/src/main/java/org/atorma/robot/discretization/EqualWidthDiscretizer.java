package org.atorma.robot.discretization;

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
		if (value < min) {
			return 0;
		} else if (value > max) {
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
