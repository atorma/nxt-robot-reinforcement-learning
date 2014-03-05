package org.atorma.robot.learning.cliffworld;

import java.util.Arrays;

import org.atorma.robot.discretization.VectorDiscretizer;

public class CliffWorldStateDiscretizer implements VectorDiscretizer {

	@Override
	public int getId(double[] value) {
		return Arrays.hashCode(value);
	}

}
