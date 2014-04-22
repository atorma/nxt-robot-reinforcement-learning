package org.atorma.robot.objecttracking;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map.Entry;
import java.util.Set;

import org.atorma.robot.mdp.State;
import org.junit.Before;
import org.junit.Test;

public class ObjectTrackingModelTests {

	private int numberOfSectors = 6;
	private ObjectTrackingModel model;
	
	@Before
	public void setUp() {
		model = new ObjectTrackingModel(numberOfSectors);
	}
	
	@Test
	public void agent_action_updates_observations_in_model() {
		TrackedObject obj1 = TrackedObject.inPolarDegreeCoordinates(30, 60);
		TrackedObject obj2 = TrackedObject.inPolarDegreeCoordinates(40, -90);
		
		model.addObservation(obj1);
		model.addObservation(obj2);
		
		double agentMove = 10;
		double agentTurnDeg = 35;
		
		model = model.afterAgentMoves(agentMove);
		model = model.afterAgentRotatesDeg(agentTurnDeg);
		
		List<TrackedObject> objects = new ArrayList<>(model.getObjects());
		Collections.sort(objects, new TrackedObjectAngleComparator()); 
				
		TrackedObject expected1 = obj1.afterObserverMoves(agentMove).afterObserverRotatesDeg(agentTurnDeg);
		assertEquals(expected1.getAngleDeg(), objects.get(0).getAngleDeg(), 0);
		assertEquals(expected1.getDistance(), objects.get(0).getDistance(), 0);
		
		TrackedObject expected2 = obj2.afterObserverMoves(agentMove).afterObserverRotatesDeg(agentTurnDeg);
		assertEquals(expected2.getAngleDeg(), objects.get(1).getAngleDeg(), 0);
		assertEquals(expected2.getDistance(), objects.get(1).getDistance(), 0);
	}
	

	@Test
	public void get_object_in_given_direction() {
		// With 6 sectors each one is 60 degrees. There is a front sector that
		// spreads +/- 30 degrees to each side of the agent's heading. 
		// As [left, mid, right] sectors are placed at 
		// [-30 (=330), 0, 30), [30, 60, 90), [90, 120, 150), 
		// [150, 180, 210), [210, 240, 270), [270, 300, 330)
		
		TrackedObject obj1 = TrackedObject.inPolarDegreeCoordinates(10, 0); 
		TrackedObject obj2 = TrackedObject.inPolarDegreeCoordinates(20, -35);
		model.addObservation(obj1);
		model.addObservation(obj2);
		
		assertEquals(obj1, model.getObjectInDirectionDegrees(0));
		assertEquals(obj1, model.getObjectInDirectionDegrees(-25));
		assertEquals(obj1, model.getObjectInDirectionDegrees(25));
		
		assertEquals(obj2, model.getObjectInDirectionDegrees(-40));
		assertEquals(obj2, model.getObjectInDirectionDegrees(300));
		
		assertNull(model.getObjectInDirectionDegrees(30));
		assertNull(model.getObjectInDirectionDegrees(60));
	}
	
	@Test
	public void get_nearest_in_given_sector() {
		model = new ObjectTrackingModel(36); // 10 degree sectors
		
		TrackedObject obj1 = TrackedObject.inPolarDegreeCoordinates(10, 0); 
		TrackedObject obj2 = TrackedObject.inPolarDegreeCoordinates(15, 20); 
		TrackedObject obj3 = TrackedObject.inPolarDegreeCoordinates(7, -10); 
		TrackedObject obj4 = TrackedObject.inPolarDegreeCoordinates(5, 90); 
		model.addObservation(obj1);
		model.addObservation(obj2);
		model.addObservation(obj3);
		model.addObservation(obj4);
		
		assertEquals(obj3, model.getNearestInSectorDegrees(-30, 30));
		assertEquals(obj4, model.getNearestInSectorDegrees(20, 110));
		
	}
	
	@Test
	public void get_objects_by_sectors() {
		TrackedObject obj1 = TrackedObject.inPolarDegreeCoordinates(10, 0); 
		TrackedObject obj2 = TrackedObject.inPolarDegreeCoordinates(20, -35);
		model.addObservation(obj1);
		model.addObservation(obj2);
		
		Set<Entry<Integer, TrackedObject>> objects = model.getObjectsBySectors();
		assertEquals(2, objects.size());
	}
	
	@Test
	public void state_vector_representation() {
		TrackedObject obj1 = TrackedObject.inPolarDegreeCoordinates(10, 0); 
		TrackedObject obj2 = TrackedObject.inPolarDegreeCoordinates(20, -35);
		model.addObservation(obj1);
		model.addObservation(obj2);
		
		State state = (State) model;
		double[] distances = state.getValues();
		assertEquals(numberOfSectors, distances.length);
		assertEquals(10, distances[0], 0);
		assertEquals(Double.MAX_VALUE, distances[1], 0);
		assertEquals(Double.MAX_VALUE, distances[2], 0);
		assertEquals(Double.MAX_VALUE, distances[3], 0);
		assertEquals(Double.MAX_VALUE, distances[4], 0);
		assertEquals(20, distances[5], 0);
	}
	

	@Test
	public void new_observation_replaces_object_in_same_sector() {
		model.addObservation(TrackedObject.inPolarDegreeCoordinates(10, 20));
		model = model.afterAgentRotatesDeg(20); // the tracked object now directly in front
		
		model.addObservation(TrackedObject.inPolarDegreeCoordinates(50, 0));
		
		assertEquals(1, model.getObjects().size());
		TrackedObject obj = model.getObjects().iterator().next();
		assertEquals(50, obj.getDistance(), 0);
		assertEquals(0, obj.getAngleDeg(), 0);
	}
	
	@Test
	public void closer_estimate_replaces_further_estimate_in_same_sector() {
		model = new ObjectTrackingModel(10);
		model.addObservation(TrackedObject.inCartesianCoordinates(1, 1));
		model.addObservation(TrackedObject.inPolarDegreeCoordinates(1000, 90));
		model = model.afterAgentMoves(1); // the first observation should be close on the right, "obscuring" the further one

		assertEquals(1, model.getObjects().size());
		TrackedObject obj = model.getObjects().iterator().next();
		assertEquals(1, obj.getDistance(), 0);
		assertEquals(90, obj.getAngleDeg(), 0);
	}
	
	@Test
	public void copy_model_with_same_number_of_sectors_maintains_all_estimates() {
		TrackedObject obj1 = TrackedObject.inPolarDegreeCoordinates(61, 0);
		TrackedObject obj2 = TrackedObject.inPolarDegreeCoordinates(25, 35);
		TrackedObject obj3 = TrackedObject.inPolarDegreeCoordinates(76, 360 - 35);
		model.addObservation(obj1);
		model.addObservation(obj2);
		model.addObservation(obj3);
		
		ObjectTrackingModel copy = model.copy();
		
		List<TrackedObject> objectsInCopy = new ArrayList<>(copy.getObjects());
		Collections.sort(objectsInCopy, new TrackedObjectAngleComparator());
		
		assertEquals(61, objectsInCopy.get(0).getDistance(), 0);
		assertEquals(0, objectsInCopy.get(0).getAngleDeg(), 0);
		assertEquals(25, objectsInCopy.get(1).getDistance(), 0);
		assertEquals(35, objectsInCopy.get(1).getAngleDeg(), 0);
		assertEquals(76, objectsInCopy.get(2).getDistance(), 0);
		assertEquals(360 - 35, objectsInCopy.get(2).getAngleDeg(), 0);
	}
	
	@Test
	public void copy_model_with_smaller_number_of_sectors_leaves_closest_object_per_sector() {
		// These are all within the 90 degree forward facing sector
		TrackedObject obj1 = TrackedObject.inPolarDegreeCoordinates(61, 0);
		TrackedObject obj2 = TrackedObject.inPolarDegreeCoordinates(25, 35);
		TrackedObject obj3 = TrackedObject.inPolarDegreeCoordinates(76, 360 - 35);
		model.addObservation(obj1);
		model.addObservation(obj2);
		model.addObservation(obj3);
		
		ObjectTrackingModel copy = model.copyAndChangeNumberOfSectors(4);
		
		List<TrackedObject> objectsInCopy = new ArrayList<>(copy.getObjects());
		assertEquals(1, objectsInCopy.size());
		assertEquals(25, objectsInCopy.get(0).getDistance(), 0);
		assertEquals(35, objectsInCopy.get(0).getAngleDeg(), 0);
	}
	
	
	
	
}
