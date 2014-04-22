package org.atorma.robot.util;

import static org.junit.Assert.assertTrue;

import org.junit.Ignore;
import org.junit.Test;

public class LejosRandomTest {
	
	// Lejos' Random class is buggy! Typically it won't produce any instances of one specific integer out of 0..3
	// and their counts in a sequence are suspiciously even. The test below usually (always) fails. 
	@Test @Ignore
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
		Random random = new Random();
		for (int i = 0; i < n; i++) {
			int r = random.nextInt(4);
			counts[r]++;
		}
		
		for (int i = 0; i <= 3; i++) {
			assertTrue(counts[i] <= 10286);
			assertTrue(counts[i] >= 9716);
		}
	}
	
	/**
	 * Pseudo-random number generation from Lejos NXT Java.
	 */
	public static class Random
	{
		//TODO make this class JDK compliant
		
	  private int iPrevSeed, iSeed;
	  private boolean haveNextNextGaussian;
	  private double nextNextGaussian;
	  
	  public Random (long seed)
	  {
	    iPrevSeed = 1;
	    iSeed = (int) seed;
	  }

	    public Random()
	    {
		this(System.currentTimeMillis());
	    }
	  
	  /**
	   * @return A random positive or negative integer.
	   */
	  public int nextInt()
	  {
	    int pNewSeed = (iSeed * 48271) ^ iPrevSeed;
	    iPrevSeed = iSeed;
	    iSeed = pNewSeed;
	    return pNewSeed;
	  }
	  
	  	private int next(int bits)
	  	{
	  		// just some work in progress, should replace nextInt()
	  		int mask;
	  		if (bits < 32)
	  			mask = (1 << bits) - 1;
	  		else
	  			mask = -1;  		
	  		return nextInt() & mask;
	  	}

	    /**
	     * Returns a random integer in the range 0...n-1.
	     * @param n  the bound
	     * @return A random integer in the range 0...n-1.
	     */
	    public int nextInt (int n)
	    {
		int m = nextInt() % n;
		return m >= 0 ? m : m + n;
	    }
	    
	    public long nextLong()
	    {
			int n1 = this.next(32);
			int n2 = this.next(32);
			return ((long)n1 << 32) + n2;
	    }

	    /**
	     * Returns a random boolean in the range 0-1.
	     * @return A random boolean in the range 0-1.
	     */
	    public boolean nextBoolean()
	    {
			return this.next(1) != 0;    	
	    }

	    public float nextFloat()
	    {
	    	// we need 24 bits number to create 23 bit mantissa
			return this.next(24) * 0x1p-24f;
	    }
	    
	    public double nextDouble()
	    {
	    	// we need 53 bits number to create 52 bit mantissa
			int n1 = this.next(26);	//26 bits
			int n2 = this.next(27);	//27 bits		
			long r = ((long)n1 << 27) | n2;
			return r * 0x1p-53;
	    }
	    
	    /**
	     * Returns the next pseudorandom, Gaussian ("normally") distributed double value with mean 0.0 and standard deviation 1.0 from this random number generator's sequence.
	     * @return Returns the next pseudorandom, Gaussian ("normally") distributed double value
	     */
	    public double nextGaussian(){
	    	//http://java.sun.com/j2se/1.4.2/docs/api/java/util/Random.html#nextGaussian()
	        if (haveNextNextGaussian) {
	            haveNextNextGaussian = false;
	            return nextNextGaussian;
		    } else {
		            double v1, v2, s;
		            do { 
		                    v1 = 2 * nextDouble() - 1;   // between -1.0 and 1.0
		                    v2 = 2 * nextDouble() - 1;   // between -1.0 and 1.0
		                    s = v1 * v1 + v2 * v2;
		            } while (s >= 1 || s == 0);
		            double multiplier = Math.sqrt(-2 * Math.log(s)/s);
		            nextNextGaussian = v2 * multiplier;
		            haveNextNextGaussian = true;
		            return v1 * multiplier;
		    }  	
	    }  
	}


}

