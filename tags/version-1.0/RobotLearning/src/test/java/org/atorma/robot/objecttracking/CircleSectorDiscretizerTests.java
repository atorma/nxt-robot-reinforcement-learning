package org.atorma.robot.objecttracking;

import static org.junit.Assert.*;

import org.junit.Test;

public class CircleSectorDiscretizerTests {

	private CircleSectorDiscretizer discretizer;
	
	
	@Test
	public void test_discretization_with_6_sectors() {
		discretizer = new CircleSectorDiscretizer(6);
		
		// See the javadoc of CircleSectorDiscretizer 
		assertEquals(0, discretizer.discretize(330));
		assertEquals(0, discretizer.discretize(-30));
		assertEquals(0, discretizer.discretize(29.9));
		
		assertEquals(1, discretizer.discretize(30));
		assertEquals(1, discretizer.discretize(60));
		assertEquals(1, discretizer.discretize(89.9));
		
		assertEquals(2, discretizer.discretize(95));
		
		assertEquals(3, discretizer.discretize(180));
		
		assertEquals(4, discretizer.discretize(265));

		assertEquals(5, discretizer.discretize(329.9));
	}
}
