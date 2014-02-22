package org.atorma.robot.util;

//
//RANMAR random number generator and initialization routine.
//Proposed by Marsaglia and Zaman in FSU-SCRI-87-50
//Modified my F.James 1988 to generate an array and translated to C
//by Gabor Csanyi 1992.
//
//The bug fixed and zero values excluded by Jarkko Heinonen 1997.
//Java class implementation written by Petri Nikunen 2003.
//
//The routine is VERY FAST with a period of 2^144
//

public class Ranmar {
	private float u[];
	private float c, cd, cm;
	private int i97, j97;

	public Ranmar() {
		this((int) System.currentTimeMillis());
	}
	
	/*
	 * To reproduce the original numbers of the author, call the constructor
	 * with seed 54211737. However, any seed gives a disjoint sequence.
	 */
	public Ranmar(int seed) {
		int ij, kl, i, j, k, l, ii, jj, m;
		float s, t;

		u = new float[98];
		ij = seed / 30082;
		kl = seed - 30082 * ij;
		i = (ij / 177) % 177 + 2;
		j = ij % 177 + 2;
		k = (kl / 169) % 178 + 1;

		l = kl % 169; // l = kl % 169 + 1; This was the error corrected by J.H.

		for (ii = 1; ii < 98; ii++) {
			s = 0.0f;
			t = 0.5f;
			for (jj = 1; jj < 25; jj++) {
				m = (((i * j) % 179) * k) % 179;
				i = j;
				j = k;
				k = m;
				l = (53 * l + 1) % 169;
				if ((l * m) % 64 >= 32) {
					s = s + t;
				}
				t *= 0.5f;
			}
			u[ii] = s;
		}
		c = 362436.0f / 16777216.0f;
		cd = 7654321.0f / 16777216.0f;
		cm = 16777213.0f / 16777216.0f;
		i97 = 97;
		j97 = 33;
	}

	public float nextFloat() {
		float uni;

		uni = u[i97] - u[j97];
		if (uni < 0)
			uni += 1.0f;
		u[i97] = uni;
		i97 -= 1;
		if (i97 == 0)
			i97 = 97;
		j97 -= 1;
		if (j97 == 0)
			j97 = 97;
		c -= cd;
		if (c < 0)
			c += cm;
		uni -= c;
		if (uni < 0)
			uni += 1;
		/* exclusion of zero added by J.H. */
		if (uni == 0.0f) {
			/* replace exact zeros by uniform distr. times 2^{-24} */
			uni = u[2] / 16777216.0f;
			if (uni == 0.0f) {
				uni = (1.0f / 16777216.0f) * (1.0f / 16777216.0f);
			}
		}
		return uni;
	}

	public float[] nextFloats(int len) {
		float[] array = new float[len];
		for (int i = 0; i < len; i++) {
			array[i] = nextFloat();
		}
		return array;
	}

	/* Addition by Anssi */
	public int nextInt(int n) {
		return (int) (nextFloat() * n);
	}
}
