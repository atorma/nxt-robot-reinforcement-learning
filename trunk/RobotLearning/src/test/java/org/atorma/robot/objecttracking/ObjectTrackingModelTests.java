package org.atorma.robot.objecttracking;

import static org.junit.Assert.*;

import java.util.List;

import org.junit.Before;
import org.junit.Test;

public class ObjectTrackingModelTests {

	private ObjectTrackingModel model;
	
	@Before
	public void setUp() {
		model = new ObjectTrackingModel();
	}
	
	@Test
	public void add_observations() {
		double distanceCm1 = 4.5;
		double angleDeg1 = 45;
		model.addObservation(distanceCm1, angleDeg1);
		
		double distanceCm2 = 10;
		double angleDeg2 = 350;
		model.addObservation(distanceCm2, angleDeg2);
		
		List<TrackedObject> objectLocations = model.getObjectLocations();
		assertEquals(2, objectLocations.size());
		
		TrackedObject object1 = objectLocations.get(0);
		assertEquals(distanceCm1, object1.getDistance(), 0.0001);
		assertEquals(angleDeg1, object1.getAngleDeg(), 0.0001);
		
		TrackedObject object2 = objectLocations.get(1);
		assertEquals(distanceCm2, object2.getDistance(), 0.0001);
		assertEquals(angleDeg2, object2.getAngleDeg(), 0.0001);
	}
	
	@Test(expected = IllegalArgumentException.class) 
	public void negative_distance_is_invalid() {
		model.addObservation(-1, 30);
	}
	
	@Test
	public void observed_angles_converted_to_semi_open_interval_from_0_to_360_degrees() {
		TrackedObject object;
		
		model.addObservation(1, 360);
		object = model.getObjectLocations().get(0);
		assertEquals(0, object.getAngleDeg(), 0.0001);
		
		model.addObservation(2, 370);
		object = model.getObjectLocations().get(1);
		assertEquals(10, object.getAngleDeg(), 0.0001);
		
		model.addObservation(4, 730);
		object = model.getObjectLocations().get(2);
		assertEquals(10, object.getAngleDeg(), 0.0001);
		
		model.addObservation(3, -10);
		object = model.getObjectLocations().get(3);
		assertEquals(350, object.getAngleDeg(), 0.0001);
		
		model.addObservation(3, -730);
		object = model.getObjectLocations().get(4);
		assertEquals(350, object.getAngleDeg(), 0.0001);
	}
	
	
}
