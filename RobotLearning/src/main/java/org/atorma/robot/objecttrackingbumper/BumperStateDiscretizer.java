package org.atorma.robot.objecttrackingbumper;

import java.util.*;

import org.atorma.robot.discretization.*;
import org.atorma.robot.mdp.State;
import org.atorma.robot.objecttracking.CircleSector;
import org.atorma.robot.objecttracking.TrackedObject;
import org.atorma.robot.simplebumper.CollisionDiscretizer;
import org.atorma.robot.simplebumper.ObstacleDistanceDiscretizer;

public class BumperStateDiscretizer implements StateDiscretizer {
	
	public static final int NUMBER_OF_SECTORS = 6;
	
	private VectorDiscretizer idFunction;
	private List<CircleSector> obstacleSectors;
	private int stateDimensions;

	public BumperStateDiscretizer() {
		double sectorWidth = 360.0/NUMBER_OF_SECTORS;
		List<CircleSector> obstacleSectors = new ArrayList<CircleSector>();
		double fromAngleDeg = 360.0 - sectorWidth/2;
		for (int i = 0; i < NUMBER_OF_SECTORS; i++) {
			double toAngleDeg = fromAngleDeg + sectorWidth;
			obstacleSectors.add(new CircleSector(fromAngleDeg, toAngleDeg));
			fromAngleDeg = toAngleDeg;
		}
		
		init(obstacleSectors);
	}
	
	public BumperStateDiscretizer(Collection<CircleSector> obstacleSectors) {
		init(obstacleSectors);
	}

	private void init(Collection<CircleSector> obstacleSectors) {
		this.obstacleSectors = new ArrayList<>(obstacleSectors);
		this.stateDimensions = obstacleSectors.size() + 1;
		
		Discretizer[] discretizers = new Discretizer[stateDimensions];
		for (int i = 0; i < obstacleSectors.size(); i++) {
			discretizers[i] = new ObstacleDistanceDiscretizer();
		}
		discretizers[obstacleSectors.size()] = new CollisionDiscretizer();
		
		idFunction = new VectorDiscretizerImpl(discretizers);
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
		
		return idFunction.getId(stateValues);
	}
	
	public int getNumberOfStates() {
		return idFunction.getNumberOfValues();
	}
	
	public int getNumberOfSectors() {
		return obstacleSectors.size();
	}
	
	public List<CircleSector> getSectors() {
		return Collections.unmodifiableList(obstacleSectors);
	}
	
	public double getSectorWidthDegrees() {
		return 360.0/NUMBER_OF_SECTORS;
	}
	
	public double getMinDistance() {
		return (new ObstacleDistanceDiscretizer()).getMin();
	}
	
	public double getDistanceBinWidth() {
		return (new ObstacleDistanceDiscretizer()).getBinWidth();
	}
	
	public double getMaxDistance() {
		return (new ObstacleDistanceDiscretizer()).getMax();
	}
	
	public int getNumberOfDistanceBins() {
		return (new ObstacleDistanceDiscretizer()).getNumberOfBins();
	}
}
