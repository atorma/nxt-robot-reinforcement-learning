package org.atorma.robot.objecttracking;

import static org.junit.Assert.assertEquals;
import static java.lang.Math.*;

import org.junit.Test;

public class TrackedObjectTests {
	
	@Test
	public void create_tracked_object() {
		TrackedObject object = new TrackedObject(10, 45);
		assertEquals(10, object.getDistanceCm(), 0);
		assertEquals(45, object.getAngleDeg(), 0);
	}

	@Test(expected = IllegalArgumentException.class) 
	public void negative_distance_is_invalid() {
		new TrackedObject(-1, 30);
	}
	
	@Test
	public void observed_angles_converted_to_semi_open_interval_from_0_to_360_degrees() {
		TrackedObject object;
		
		object = new TrackedObject(1, 360);
		assertEquals(0, object.getAngleDeg(), 0);
		
		object = new TrackedObject(2, 370);
		assertEquals(10, object.getAngleDeg(), 0);
		
		object = new TrackedObject(4, 730);
		assertEquals(10, object.getAngleDeg(), 0);
		
		object = new TrackedObject(3, -10);
		assertEquals(350, object.getAngleDeg(), 0);
		
		object = new TrackedObject(3, -730);
		assertEquals(350, object.getAngleDeg(), 0);
	}
	
	@Test
	public void from_polar_to_cartesian_coordinates_and_back() {
		TrackedObject objectEnteredInPolarCoordinates = new TrackedObject(2, 0); // straight ahead, 2 cm away
		assertEquals(0, objectEnteredInPolarCoordinates.getXCm(), 0.0001);
		assertEquals(2, objectEnteredInPolarCoordinates.getYCm(), 0.0001); // convention is agent always heading along y-axis
		
		TrackedObject objectEnteredInCartesianCoordinates = TrackedObject.inCartesianCoordinates(0, 2); // straight ahead, 2 cm away
		assertEquals(2, objectEnteredInCartesianCoordinates.getDistanceCm(), 0);
		assertEquals(0, objectEnteredInCartesianCoordinates.getAngleDeg(), 0);
	}
	
	@Test
	public void compute_new_location_when_observer_rotates() {
		// Object 45 degrees to the right
		TrackedObject objectBefore = new TrackedObject(10, 45);
		
		TrackedObject objectAfter;
		
		// Turn right more degrees than the object is observed at 
		objectAfter = objectBefore.afterObserverRotates(90);
		assertEquals(360 - 45, objectAfter.getAngleDeg(), 0);
		assertEquals(10, objectAfter.getDistanceCm(), 0);
		
		// Turn left
		objectAfter = objectBefore.afterObserverRotates(-90);
		assertEquals(135, objectAfter.getAngleDeg(), 0);
		
		// Turn right over one full rotation
		objectAfter = objectBefore.afterObserverRotates(370);
		assertEquals(35, objectAfter.getAngleDeg(), 0);
		
		// Turn left over one full rotation
		objectAfter = objectBefore.afterObserverRotates(-370);
		assertEquals(55, objectAfter.getAngleDeg(), 0);
	}

	@Test
	public void compute_new_location_when_observer_moves() {
		// Object at sqrt(3^2 + 3^2) cm distance, 45 degrees to the right.
		// This case is a simple right angle triangle computation 
		TrackedObject objectBefore = new TrackedObject(sqrt(18), 45);
		TrackedObject objectAfter = objectBefore.afterObserverMoves(3);
		assertEquals(3, objectAfter.getDistanceCm(), 0.001);
		assertEquals(90, objectAfter.getAngleDeg(), 0.001);
	}
	
	@Test
	public void compute_new_location_when_observer_does_several_moves() {
		// This case you can plot on a 7x7 mm square grid paper.
		// Agent start facing right, the object is located 5 units to the right, 2 units up
		TrackedObject objectBefore = new TrackedObject(sqrt(pow(5, 2) + pow(2, 2)), -toDegrees(atan(2.0/5.0)));
		System.out.println(objectBefore);
		
		TrackedObject objectAfter = objectBefore
				.afterObserverMoves(1)
				.afterObserverRotates(-90)
				.afterObserverMoves(2); // one right, two up from starting point
		assertEquals(4, objectAfter.getDistanceCm(), 0.01);
		assertEquals(90, objectAfter.getAngleDeg(), 0.01);
				objectAfter = objectAfter.afterObserverRotates(90)
				.afterObserverMoves(3)
				.afterObserverRotates(-90)
				.afterObserverMoves(1)
				.afterObserverRotates(90)
				.afterObserverMoves(2)
				.afterObserverMoves(2)
				.afterObserverRotates(90)
				.afterObserverMoves(2)
				.afterObserverRotates(90)
				.afterObserverMoves(2)
				.afterObserverRotates(90)
				.afterObserverMoves(1);
		
		// Should be one unit away, 90 degrees to the left
		assertEquals(1, objectAfter.getDistanceCm(), 0.01);
		assertEquals(360 - 90, objectAfter.getAngleDeg(), 0.01);
	}
}
