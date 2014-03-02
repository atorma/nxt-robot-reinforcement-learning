package org.atorma.robot.objecttracking;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map.Entry;
import java.util.Set;

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
		
		model.agentMoves(agentMove);
		model.agentRotatesDeg(agentTurnDeg);
		
		List<TrackedObject> objects = new ArrayList<>(model.getObjects());
		Collections.sort(objects); // natural order is first by angle [0, 360), then by distance
				
		TrackedObject expected1 = obj1.afterObserverMoves(agentMove).afterObserverRotatesDeg(agentTurnDeg);
		assertEquals(expected1.getAngleDeg(), objects.get(0).getAngleDeg(), 0);
		assertEquals(expected1.getDistance(), objects.get(0).getDistance(), 0);
		
		TrackedObject expected2 = obj2.afterObserverMoves(agentMove).afterObserverRotatesDeg(agentTurnDeg);
		assertEquals(expected2.getAngleDeg(), objects.get(1).getAngleDeg(), 0);
		assertEquals(expected2.getDistance(), objects.get(1).getDistance(), 0);
	}
	

	@Test
	public void get_object_at_given_sector() {
		// With 6 sectors each one is 60 degrees. There is a front sector that
		// spreads +/- 30 degrees to each side of the agent's heading. 
		// As [left, mid, right] sectors are placed at 
		// [-30 (=330), 0, 30), [30, 60, 90), [90, 120, 150), 
		// [150, 180, 210), [210, 240, 270), [270, 300, 330)
		
		TrackedObject obj1 = TrackedObject.inPolarDegreeCoordinates(10, 0); 
		TrackedObject obj2 = TrackedObject.inPolarDegreeCoordinates(20, -35);
		model.addObservation(obj1);
		model.addObservation(obj2);
		
		assertEquals(obj1, model.getObjectInSectorDegree(0));
		assertEquals(obj1, model.getObjectInSectorDegree(-25));
		assertEquals(obj1, model.getObjectInSectorDegree(25));
		
		assertEquals(obj2, model.getObjectInSectorDegree(-40));
		assertEquals(obj2, model.getObjectInSectorDegree(300));
		
		assertNull(model.getObjectInSectorDegree(30));
		assertNull(model.getObjectInSectorDegree(60));
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
	public void new_observation_replaces_object_in_same_sector() {
		model.addObservation(TrackedObject.inPolarDegreeCoordinates(10, 20));
		model.agentRotatesDeg(20); // the tracked object now directly in front
		
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
		model.agentMoves(1); // the first observation should be close on the right, "obscuring" the further one

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
		Collections.sort(objectsInCopy);
		
		assertEquals(61, objectsInCopy.get(0).getDistance(), 0);
		assertEquals(0, objectsInCopy.get(0).getAngleDeg(), 0);
		assertEquals(25, objectsInCopy.get(1).getDistance(), 0);
		assertEquals(35, objectsInCopy.get(1).getAngleDeg(), 0);
		assertEquals(76, objectsInCopy.get(2).getDistance(), 0);
		assertEquals(360 - 35, objectsInCopy.get(2).getAngleDeg(), 0);
	}
	
	@Test
	public void copy_model_with_samller_number_of_sectors_leaves_closest_object_per_sector() {
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
