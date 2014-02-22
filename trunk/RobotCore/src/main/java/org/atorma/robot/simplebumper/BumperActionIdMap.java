package org.atorma.robot.simplebumper;

import org.atorma.robot.discretization.IdFunction;

public class BumperActionIdMap implements IdFunction {
	
	@Override
	public int getId(double[] value) {
		return (int) value[0]; // Bumper action value already ids 
	}

}
