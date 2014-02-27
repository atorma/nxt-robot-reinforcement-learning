package org.atorma.robot.objecttracking;

import org.atorma.robot.discretization.Discretizer;
import org.atorma.robot.discretization.EqualWidthDiscretizer;

/**
 * Discretizes circle angles in degrees into sector indexes.
 * E.g. with 6 sectors the sectors, as [left, mid, right), are 
 * <ul>
 * <li>0: [-30 (=330), 0, 30)
 * <li>1: [30, 60, 90)
 * <li>2: [90, 120, 150)
 * <li>3: [150, 180, 210)
 * <li>4: [210, 240, 270)
 * <li>5: [270, 300, 330)
 * </ul>
 * The first sector is always centered at degrees zero and spans
 * the zero "axis" on both sides.
 */
public class CircleSectorDiscretizer implements Discretizer {
	
	private final int numberOfSectors;
	private EqualWidthDiscretizer equalWidthDiscretizer;
	
	private final double frontSectorLeft; // front sector from this to 360 = 0
	private final double frontSectorRight; // .. and from 0 to this
	
	public CircleSectorDiscretizer(int numberOfSectors) {
		if (numberOfSectors < 1) {
			throw new IllegalArgumentException("Must have at least 1 sector");
		}
		this.numberOfSectors = numberOfSectors;
		
		// The first sector is the one and only that spans both sides of the zero angle, it must be handled separately.
		// The rest we can handle by equal width discretization
		frontSectorLeft = 360.0 - 360.0/numberOfSectors/2;
		frontSectorRight = 360.0/numberOfSectors/2;
		equalWidthDiscretizer = new EqualWidthDiscretizer(frontSectorRight, frontSectorLeft, numberOfSectors - 1);
	}

	@Override
	public int discretize(double value) {
		if (numberOfSectors == 1) {
			return 0;
		}
		
		double normalizedDeg = ObjectTrackingUtils.normalizeDegrees(value);
		if (normalizedDeg >= frontSectorLeft || normalizedDeg < frontSectorRight) {
			return 0;
		} else {
			return equalWidthDiscretizer.discretize(normalizedDeg) + 1;
		}
		
	}

	@Override
	public int getNumberOfBins() {
		return numberOfSectors;
	}

}
