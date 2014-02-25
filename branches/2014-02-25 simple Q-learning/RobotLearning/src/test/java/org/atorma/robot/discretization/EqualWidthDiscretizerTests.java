package org.atorma.robot.discretization;

import static org.junit.Assert.*;

import org.atorma.robot.discretization.Discretizer;
import org.atorma.robot.discretization.EqualWidthDiscretizer;
import org.junit.Test;

public class EqualWidthDiscretizerTests {

	
	@Test
	public void correct_discretization_when_bin_width_equals_one() {
		double min = -5, max = 5;
		int nBins = 10;
		
		Discretizer discretizer = new EqualWidthDiscretizer(min, max, nBins);
		
		int expectedBin = 0;
		for (double x = -5; x <= 5; x = x + 1) {
			assertEquals(expectedBin, discretizer.discretize(x));
			expectedBin++;
		}
		
		assertEquals(0, discretizer.discretize(min - 10));
		assertEquals(nBins - 1, discretizer.discretize(max + 10));
		assertEquals(0, discretizer.discretize(-4.9));
		assertEquals(1, discretizer.discretize(-3.1));
	}
	
	@Test
	public void discretization_when_bin_width_is_zero() {
		double min = 1, max = 1;
		int nBins = 10;
		
		Discretizer discretizer = new EqualWidthDiscretizer(min, max, nBins);
		
		assertEquals(0, discretizer.discretize(min - 10));
		assertEquals(0, discretizer.discretize(min));
		assertEquals(0, discretizer.discretize(max));
		assertEquals(nBins - 1, discretizer.discretize(max + 10));
	}
	
}
