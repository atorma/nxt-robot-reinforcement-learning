package org.atorma.robot.simplebumper;

import org.atorma.robot.discretization.CustomBinsDiscretizer;
import org.atorma.robot.discretization.DiscretizationBasedIdFunction;
import org.atorma.robot.discretization.Discretizer;
import org.atorma.robot.discretization.IdFunction;

public class BumperStateIdMap implements IdFunction {
		
	private DiscretizationBasedIdFunction idFunction;

	public BumperStateIdMap() {
		idFunction = new DiscretizationBasedIdFunction(getUltrasonicDistanceDiscretizer());
	}

	private Discretizer getUltrasonicDistanceDiscretizer() {
		double[] bins = new double[5];
		for (int i = 0; i < bins.length; i++) {
				bins[i] = BumperState.MIN_ULTRASONIC_DIST * (i + 1);
		}
		Discretizer discretizer = new CustomBinsDiscretizer(bins);
		return discretizer;
	}
	
	@Override
	public int getId(double[] values) {
		return idFunction.getId(values);
	}

}
