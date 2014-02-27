package org.atorma.robot.objecttracking;

import static org.junit.Assert.*;
import static java.lang.Math.*;

import org.junit.Test;

public class TrackedObjectTests {
	
	@Test
	public void create_tracked_object() {
		TrackedObject object = TrackedObject.inPolarDegreeCoordinates(10, 45);
		assertEquals(10, object.getDistance(), 0);
		assertEquals(45, object.getAngleDeg(), 0);
	}

	@Test(expected = IllegalArgumentException.class) 
	public void negative_distance_is_invalid() {
		TrackedObject.inPolarDegreeCoordinates(-1, 0);
	}
	
	@Test
	public void observed_angles_converted_to_semi_open_interval_from_0_to_360_degrees() {
		TrackedObject object;
		
		object = TrackedObject.inPolarDegreeCoordinates(1, 360);
		assertEquals(0, object.getAngleDeg(), 0.0001);
		
		object = TrackedObject.inPolarDegreeCoordinates(2, 370);
		assertEquals(10, object.getAngleDeg(), 0.0001);
		
		object = TrackedObject.inPolarDegreeCoordinates(4, 730);
		assertEquals(10, object.getAngleDeg(), 0.0001);
		
		object = TrackedObject.inPolarDegreeCoordinates(3, -10);
		assertEquals(350, object.getAngleDeg(), 0.0001);
		
		object = TrackedObject.inPolarDegreeCoordinates(3, -730);
		assertEquals(350, object.getAngleDeg(), 0.0001);
	}
	
	@Test
	public void enter_object_in_cartesian_coordinates() {
		TrackedObject object;
		
		object = TrackedObject.inCartesianCoordinates(-1, 0);
		assertEquals(1, object.getDistance(), 0);
		assertEquals(270, object.getAngleDeg(), 0);
	}
	
	@Test
	public void from_polar_to_cartesian_coordinates_and_back() {
		TrackedObject object;
		
		object = TrackedObject.inPolarDegreeCoordinates(2, 0); // straight ahead, 2 cm away
		assertEquals(0, object.getX(), 0.0001);
		assertEquals(2, object.getY(), 0.0001); // convention is agent always heading along y-axis
		
		object = TrackedObject.inCartesianCoordinates(0, 2); // straight ahead, 2 cm away
		assertEquals(2, object.getDistance(), 0);
		assertEquals(0, object.getAngleDeg(), 0);
	}
		
	@Test
	public void compute_new_location_when_observer_rotates() {
		// Object 45 degrees to the right
		TrackedObject objectBefore = TrackedObject.inPolarDegreeCoordinates(10, 45);
		
		TrackedObject objectAfter;
		
		// Turn right more degrees than the object is observed at 
		objectAfter = objectBefore.afterObserverRotatesDeg(90);
		assertEquals(360 - 45, objectAfter.getAngleDeg(), 0);
		assertEquals(10, objectAfter.getDistance(), 0.0001);
		
		// Turn left
		objectAfter = objectBefore.afterObserverRotatesDeg(-90);
		assertEquals(135, objectAfter.getAngleDeg(), 0.0001);
		
		// Turn right over one full rotation
		objectAfter = objectBefore.afterObserverRotatesDeg(370);
		assertEquals(35, objectAfter.getAngleDeg(), 0.0001);
		
		// Turn left over one full rotation
		objectAfter = objectBefore.afterObserverRotatesDeg(-370);
		assertEquals(55, objectAfter.getAngleDeg(), 0.0001);
	}

	@Test
	public void compute_new_location_when_observer_moves() {
		// Object at Cartesian coordinates (3,3).
		TrackedObject objectBefore = TrackedObject.inCartesianCoordinates(3, 3);
		
		// Agent moves forward 3 so that the object should be directly to the right.
		// This is a simple right angle triangle case. 
		TrackedObject objectAfter = objectBefore.afterObserverMoves(3);
		assertEquals(3, objectAfter.getDistance(), 0.001);
		assertEquals(90, objectAfter.getAngleDeg(), 0.001);
		
		// Agent moves very, very far backward. We expect to see the object far in the horizon to the front.
		objectAfter = objectBefore.afterObserverMoves(-999999999.9);
		assertTrue(objectAfter.getDistance() >= 999999999.9);
		assertEquals(0, objectAfter.getAngleDeg(), 0.001);
	}
	
	@Test
	public void compute_new_location_when_observer_does_several_moves() {
		// This case you can plot on a 7x7 mm square grid paper.
		// Agent start facing north (up), the object is located 5 units to the right, 2 units up in global coordinates
		TrackedObject objectBefore = TrackedObject.inCartesianCoordinates(5, 2);
		
		TrackedObject objectAfter = objectBefore
				.afterObserverMoves(2)
				.afterObserverRotatesDeg(90)
				.afterObserverMoves(1); // two up, one right from starting point, agent now facing right
		assertEquals(4, objectAfter.getDistance(), 0);
			objectAfter = objectAfter.afterObserverMoves(3)
				.afterObserverRotatesDeg(-90)
				.afterObserverMoves(1)
				.afterObserverRotatesDeg(90)
				.afterObserverMoves(2)
				.afterObserverMoves(2)
				.afterObserverRotatesDeg(90)
				.afterObserverMoves(2)
				.afterObserverRotatesDeg(90)
				.afterObserverMoves(2)
				.afterObserverRotatesDeg(90);
			objectAfter = objectAfter.afterObserverMoves(1);
		
		// Should be one unit away, 90 degrees to the left
		assertEquals(1, objectAfter.getDistance(), 0.0001);
		assertEquals(360 - 90, objectAfter.getAngleDeg(), 0.0001);
	}
}
