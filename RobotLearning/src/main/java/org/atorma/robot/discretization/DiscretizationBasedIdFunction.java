package org.atorma.robot.discretization;

public class DiscretizationBasedIdFunction implements IdFunction {

	private final Discretizer[] discretizers;
	private final int numberOfValues;

	
	public DiscretizationBasedIdFunction(Discretizer... discretizers) {
		if (discretizers == null || discretizers.length == 0) {
			throw new IllegalArgumentException("Must define at least one discretizer");
		}
		
		this.discretizers = discretizers;
		
		int n = discretizers[0].getNumberOfBins();
		for (int i = 1; i < discretizers.length; i++) {
			n = n*discretizers[i].getNumberOfBins(); 
		}
		this.numberOfValues = n;
	}

	/**
	 * @return the id of the input value in range [0, numberOfValues-1]
	 */
	@Override
	public int getId(double[] value) {
		if (value.length != discretizers.length) {
			throw new IllegalArgumentException("Illegal value dimensions");
		}
		
		int id = discretizers[0].discretize(value[0]);
		int n = discretizers[0].getNumberOfBins();
		for (int i = 1; i < value.length; i++) {
			id = id + discretizers[i].discretize(value[i])*n;
			n = n*discretizers[i].getNumberOfBins(); 
		}

		return id;
	}
	
	public int getNumberOfValues() {
		 return this.numberOfValues;
	}
}
