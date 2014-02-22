package org.atorma.robot.simplebumper;

import org.atorma.robot.discretization.CustomBinsDiscretizer;
import org.atorma.robot.discretization.DiscretizationBasedIdFunction;
import org.atorma.robot.discretization.Discretizer;
import org.atorma.robot.discretization.IdFunction;

public class BumperStateIdMap implements IdFunction {
		
	private DiscretizationBasedIdFunction idFunction;

	public BumperStateIdMap() {
		idFunction = new DiscretizationBasedIdFunction(getUltrasonicDistanceDiscretizer(), getCollisionDiscretizer());
	}

	private Discretizer getUltrasonicDistanceDiscretizer() {
		double[] bins = new double[5];
		for (int i = 0; i < bins.length; i++) {
				bins[i] = BumperState.MIN_ULTRASONIC_DIST * (i + 1);
		}
		Discretizer discretizer = new CustomBinsDiscretizer(bins);
		return discretizer;
	}
	
	private Discretizer getCollisionDiscretizer() {
		return new Discretizer() {
			
			@Override
			public int getNumberOfBins() {
				return 2;
			}
			
			@Override
			public int discretize(double value) {
				return value == 0 ? 0 : 1;
			}
		};
	}
	
	@Override
	public int getId(double[] values) {
		return idFunction.getId(values);
	}

}
