package org.atorma.robot.discretization;

/**
 * Produces a scalar id representing a discretized vector value.
 * Each dimension of the vector is assigned its own discretizer.
 */
public class VectorDiscretizerImpl implements VectorDiscretizer {

	private final Discretizer[] discretizers;
	private final int numberOfValues;

	
	public VectorDiscretizerImpl(Discretizer... discretizers) {
		if (discretizers == null || discretizers.length == 0) {
			throw new IllegalArgumentException("Must define at least one discretizer");
		}
		
		this.discretizers = discretizers;
		
		int n = discretizers[0].getNumberOfBins();
		for (int i = 1; i < discretizers.length; i++) {
			n = n*discretizers[i].getNumberOfBins(); 
			
			// Check against overflow. Number of state combinations easily explode and cause overflow!
			if (n < 0) {
				throw new IllegalArgumentException("Number of states overflow!");
			}
		}
		this.numberOfValues = n;

	}

	/**
	 * @return the id of the input value in range [0, numberOfValues-1]
	 */
	@Override
	public int getId(double[] value) {
		if (value.length != discretizers.length) {
			throw new IllegalArgumentException("Illegal value dimensions. There are " + discretizers.length + " discretizers but " + value.length + " values were given.");
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
