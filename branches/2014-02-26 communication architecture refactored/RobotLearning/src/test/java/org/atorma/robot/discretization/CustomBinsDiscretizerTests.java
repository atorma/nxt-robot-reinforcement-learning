package org.atorma.robot.discretization;

import static org.junit.Assert.*;

import org.junit.Test;

public class CustomBinsDiscretizerTests {

	@Test
	public void test_discretization_with_custom_bins() {
		double[] bins = new double[] {-1, 0, 0.1, 0.3};
		Discretizer discretizer = new CustomBinsDiscretizer(bins);
		
		assertEquals(5, discretizer.getNumberOfBins());
		
		assertEquals(0, discretizer.discretize(-100));
		assertEquals(1, discretizer.discretize(-1.0));
		assertEquals(1, discretizer.discretize(-0.5));
		assertEquals(2, discretizer.discretize(0));
		assertEquals(2, discretizer.discretize(0.08));
		assertEquals(3, discretizer.discretize(0.1));
		assertEquals(3, discretizer.discretize(0.2));
		assertEquals(4, discretizer.discretize(0.3));
		assertEquals(4, discretizer.discretize(100));
	}
}

