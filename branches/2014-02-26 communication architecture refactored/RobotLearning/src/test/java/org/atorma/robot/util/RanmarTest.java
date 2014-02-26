package org.atorma.robot.util;

import static org.junit.Assert.*;

import org.junit.Test;

public class RanmarTest {

	@Test
	public void test_getting_integers() {
		// Random integer r between 0..3
		// -> multinomial distribution with each event probability p = 0.25. 
		// Individual counts are binomially distributed and independent of the others 
		// for any three chosen integers (the fourth always fixed by the total trials).
		// With probability 0.999, the count is between 9716..10286.
		// Thus, if the numbers are random, the test below fails with by chance
		// with probability 1 - 0.999^3 = 0.003.
		
		int n = 40000; 
		int[] counts = new int[4];
		Ranmar random = new Ranmar();
		for (int i = 0; i < n; i++) {
			int r = random.nextInt(4);
			counts[r]++;
		}
		
		for (int i = 0; i <= 2; i++) {
			System.out.println(counts[i]);
			assertTrue(counts[i] <= 10286);
			assertTrue(counts[i] >= 9716);
		}
	}
}
