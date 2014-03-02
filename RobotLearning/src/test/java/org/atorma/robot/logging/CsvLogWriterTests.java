package org.atorma.robot.logging;

import static org.junit.Assert.*;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;

import org.junit.*;

import au.com.bytecode.opencsv.CSV;
import au.com.bytecode.opencsv.CSVReadProc;

public class CsvLogWriterTests {

	private String[] columnHeaders = new String[] {"Column 1", "Column 2"};
	
	private CsvLogWriter logWriter;
	
	
	@After
	public void tearDown() throws InterruptedException, IOException {
		Files.delete(logWriter.getOutputFile().toPath());
	}
	
	@Test
	public void row_written_some_time_after_adding_and_can_be_read_back() throws InterruptedException {
		logWriter = new CsvLogWriter(new File("test 1.csv"), columnHeaders);
		
		Object[] values = new String[] {"value 1", "value 2"};
		logWriter.addRow(values);
		logWriter.addRow(values);
		
		CSV csv = logWriter.getCsvConfiguration();
		
		TestCSVReadProc readProc = new TestCSVReadProc();
		csv.read(logWriter.getOutputFile(), readProc);
		
		assertEquals(3, readProc.rows.size());
		assertArrayEquals(columnHeaders, readProc.rows.get(0));
		assertArrayEquals(values, readProc.rows.get(1));
		assertArrayEquals(values, readProc.rows.get(2));
	}
	
	
	@Test(expected = IllegalArgumentException.class)
	public void exception_if_row_contains_different_number_of_cells_than_headers() {
		logWriter = new CsvLogWriter(new File("test 2.csv"), columnHeaders);
		logWriter.addRow("value 1", "value 2", "value 3");
	}
	
	@Test
	public void null_does_not_cause_exception() {
		logWriter = new CsvLogWriter(new File("test 3.csv"), columnHeaders);
		
		Object[] values = new String[] {null, "value 2"};
		logWriter.addRow(values);
	}
	
	
	private class TestCSVReadProc implements CSVReadProc {
		
		private List<String[]> rows = new ArrayList<String[]>();

		@Override
		public void procRow(int rowIndex, String... values) {
			rows.add(rowIndex, values);
		}
		
	}
}
