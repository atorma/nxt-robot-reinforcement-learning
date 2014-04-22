package org.atorma.robot.simplebumper;

import org.atorma.robot.discretization.Discretizer;

public class CollisionDiscretizer implements Discretizer {

	@Override
	public int discretize(double value) {
		return value == 0 ? 0 : 1;
	}

	@Override
	public int getNumberOfBins() {
		return 2;
	}

}
