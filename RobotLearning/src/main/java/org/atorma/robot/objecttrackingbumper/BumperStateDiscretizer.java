package org.atorma.robot.objecttrackingbumper;

import java.util.*;

import org.atorma.robot.discretization.*;
import org.atorma.robot.mdp.State;
import org.atorma.robot.objecttracking.CircleSector;
import org.atorma.robot.objecttracking.TrackedObject;
import org.atorma.robot.simplebumper.CollisionDiscretizer;
import org.atorma.robot.simplebumper.ObstacleDistanceDiscretizer;

/**
 * A discretizer for {@link ModeledBumperState}s. The state
 * is first reduced to a vector of nearest object distances.
 * Each distance is taken from a specified obstacle tracking sector.
 * The collision state (boolean) is then appended to the distance vector.
 */
public class BumperStateDiscretizer implements StateDiscretizer {
	
	private static final int DEFAULT_NUMBER_OF_SECTORS = 6;
	
	private VectorDiscretizer vectorDiscretizer;
	private List<CircleSector> obstacleSectors;
	private int stateDimensions;
	
	private ObstacleDistanceDiscretizer distanceDiscretizer = new ObstacleDistanceDiscretizer();
	private CollisionDiscretizer collisionDiscretizer = new CollisionDiscretizer();

	/**
	 * Creates a discretizer with 6*60 degree obstacle tracking sectors around the robot. 
     * The first sector is between degrees [-30, 30). 
	 */
	public BumperStateDiscretizer() {
		double sectorWidth = 360.0/DEFAULT_NUMBER_OF_SECTORS;
		List<CircleSector> obstacleSectors = new ArrayList<CircleSector>();
		double fromAngleDeg = 360.0 - sectorWidth/2;
		for (int i = 0; i < DEFAULT_NUMBER_OF_SECTORS; i++) {
			double toAngleDeg = fromAngleDeg + sectorWidth;
			obstacleSectors.add(new CircleSector(fromAngleDeg, toAngleDeg));
			fromAngleDeg = toAngleDeg;
		}
		
		init(obstacleSectors);
	}
	
	/**
	 * Creates discretizer for specified obstacle tracking sectors.
	 */
	public BumperStateDiscretizer(Collection<CircleSector> obstacleSectors) {
		init(obstacleSectors);
	}

	private void init(Collection<CircleSector> obstacleSectors) {
		this.obstacleSectors = new ArrayList<>(obstacleSectors);
		this.stateDimensions = obstacleSectors.size() + 1;
		
		Discretizer[] discretizers = new Discretizer[stateDimensions];
		for (int i = 0; i < obstacleSectors.size(); i++) {
			discretizers[i] = distanceDiscretizer;
		}
		discretizers[obstacleSectors.size()] = collisionDiscretizer;
		
		vectorDiscretizer = new VectorDiscretizerImpl(discretizers);
	}
	

	@Override
	public int getId(State state) {
		ModeledBumperState bumperState = (ModeledBumperState) state;
		
		double[] stateValues = new double[stateDimensions];
		for (int i = 0; i < obstacleSectors.size(); i++) {
			CircleSector sector = obstacleSectors.get(i);
			TrackedObject obstacle = bumperState.getNearestInSectorDegrees(sector.getFromAngleDeg(), sector.getToAngleDeg());
			double distance = obstacle != null ? obstacle.getDistance() : Double.MAX_VALUE;
			stateValues[i] = distance;
		}

		stateValues[stateDimensions - 1] = bumperState.isCollided() ? 1 : 0;
		
		return vectorDiscretizer.getId(stateValues);
	}
	
	public int getNumberOfStates() {
		return vectorDiscretizer.getNumberOfValues();
	}
	
	public int getNumberOfSectors() {
		return obstacleSectors.size();
	}
	
	public List<CircleSector> getSectors() {
		return Collections.unmodifiableList(obstacleSectors);
	}

	public double getMinDistance() {
		return distanceDiscretizer.getMin();
	}
	
	public double getDistanceBinWidth() {
		return distanceDiscretizer.getBinWidth();
	}
	
	public double getMaxDistance() {
		return distanceDiscretizer.getMax();
	}
	
	public int getNumberOfDistanceBins() {
		return distanceDiscretizer.getNumberOfBins();
	}
}
