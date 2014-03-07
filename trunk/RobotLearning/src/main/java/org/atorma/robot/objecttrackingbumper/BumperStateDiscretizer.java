package org.atorma.robot.objecttrackingbumper;

import org.atorma.robot.discretization.*;
import org.atorma.robot.mdp.State;

public class BumperStateDiscretizer implements StateDiscretizer {
		
	private VectorDiscretizer idFunction;

	public BumperStateDiscretizer() {
		Discretizer[] discretizers = new Discretizer[ModeledBumperState.NUMBER_OF_SECTORS_FOR_OBJECT_TRACKING + 1];
		for (int i = 0; i < ModeledBumperState.NUMBER_OF_SECTORS_FOR_OBJECT_TRACKING; i++) {
			discretizers[i] = createObjectDistanceDiscretizer();
		}
		discretizers[ModeledBumperState.NUMBER_OF_SECTORS_FOR_OBJECT_TRACKING] = getCollisionDiscretizer();
		
		idFunction = new VectorDiscretizerImpl(discretizers);
	}

	private Discretizer createObjectDistanceDiscretizer() {
		//double[] bins = new double[] {10, 40};  
		//Discretizer discretizer = new CustomBinsDiscretizer(bins);
		Discretizer discretizer = new EqualWidthDiscretizer(10, 50, 4);
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
			
	private Discretizer getLightValueDiscretizer() {
		return new EqualWidthDiscretizer(0, 800, 5); // TODO sensible discretization
	}

	@Override
	public int getId(State state) {
		ModeledBumperState modeledState = (ModeledBumperState) state;
		return idFunction.getId(modeledState.getValues());
	}
	
	

}
