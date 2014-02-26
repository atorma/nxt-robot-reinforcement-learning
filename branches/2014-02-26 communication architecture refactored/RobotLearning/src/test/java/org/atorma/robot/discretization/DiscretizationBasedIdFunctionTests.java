package org.atorma.robot.discretization;

import static org.junit.Assert.*;

import org.atorma.robot.discretization.DiscretizationBasedIdFunction;
import org.atorma.robot.discretization.Discretizer;
import org.atorma.robot.discretization.EqualWidthDiscretizer;
import org.junit.Test;

public class DiscretizationBasedIdFunctionTests {

	
	@Test
	public void id_computed_correctly_when_three_dimensions_with_different_number_of_bins() {
		
		Discretizer discretizer1 = new EqualWidthDiscretizer(0, 1, 3);
		Discretizer discretizer2 = new EqualWidthDiscretizer(0, 1, 2);
		Discretizer discretizer3 = new EqualWidthDiscretizer(0, 1, 4);
		
		DiscretizationBasedIdFunction idFunction = new DiscretizationBasedIdFunction(discretizer1, discretizer2, discretizer3);
		
		// There are 3*2*4 = 24 distinct values
		assertEquals(24, idFunction.getNumberOfValues());
		
		// Values between 0..23
		assertEquals(0, idFunction.getId(new double[] {0.1, 0.4, 0.1}));
		assertEquals(1, idFunction.getId(new double[] {0.5, 0.4, 0.1}));
		assertEquals(2, idFunction.getId(new double[] {0.8, 0.4, 0.1}));
		assertEquals(3, idFunction.getId(new double[] {0.1, 0.8, 0.1}));
		assertEquals(4, idFunction.getId(new double[] {0.5, 0.8, 0.1}));
		assertEquals(5, idFunction.getId(new double[] {0.8, 0.8, 0.1}));
		assertEquals(6, idFunction.getId(new double[] {0.1, 0.4, 0.4}));
		assertEquals(7, idFunction.getId(new double[] {0.5, 0.4, 0.4}));
		assertEquals(8, idFunction.getId(new double[] {0.8, 0.4, 0.4}));
		assertEquals(9, idFunction.getId(new double[] {0.1, 0.8, 0.4}));
		assertEquals(10, idFunction.getId(new double[] {0.5, 0.8, 0.4}));
		assertEquals(11, idFunction.getId(new double[] {0.8, 0.8, 0.4}));
		assertEquals(12, idFunction.getId(new double[] {0.1, 0.4, 0.7}));
		assertEquals(13, idFunction.getId(new double[] {0.5, 0.4, 0.7}));
		assertEquals(14, idFunction.getId(new double[] {0.8, 0.4, 0.7}));
		assertEquals(15, idFunction.getId(new double[] {0.1, 0.8, 0.7}));
		assertEquals(16, idFunction.getId(new double[] {0.5, 0.8, 0.7}));
		assertEquals(17, idFunction.getId(new double[] {0.8, 0.8, 0.7}));
		assertEquals(18, idFunction.getId(new double[] {0.1, 0.4, 0.9}));
		assertEquals(19, idFunction.getId(new double[] {0.5, 0.4, 0.9}));
		assertEquals(20, idFunction.getId(new double[] {0.8, 0.4, 0.9}));
		assertEquals(21, idFunction.getId(new double[] {0.1, 0.8, 0.9}));
		assertEquals(22, idFunction.getId(new double[] {0.5, 0.8, 0.9}));
		assertEquals(23, idFunction.getId(new double[] {0.8, 0.8, 0.9}));
		
		// Values out of bounds
		assertEquals(0, idFunction.getId(new double[] {-1, -1, -1}));
		assertEquals(23, idFunction.getId(new double[] {10, 10, 10}));
	}
	
}
