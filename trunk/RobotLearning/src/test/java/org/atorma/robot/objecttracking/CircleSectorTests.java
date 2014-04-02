package org.atorma.robot.objecttracking;

import static org.junit.Assert.*;

import org.junit.Test;

public class CircleSectorTests {

	@Test
	public void test_circle_sector() {
		CircleSector sector;
		
		sector = new CircleSector(-45, 45);
		assertEquals(0, sector.getMidAngleDeg(), 0);	
		assertTrue(sector.contains(-45));
		assertTrue(sector.contains(-15));
		assertTrue(sector.contains(0));
		assertTrue(sector.contains(15));
		assertFalse(sector.contains(45));
		
		sector = new CircleSector(45, 135);
		assertEquals(90, sector.getMidAngleDeg(), 0);	
		assertTrue(sector.contains(45));
		assertTrue(sector.contains(90));
		assertFalse(sector.contains(135));
	}
}
