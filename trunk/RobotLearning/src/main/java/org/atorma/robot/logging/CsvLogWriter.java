package org.atorma.robot.logging;

import java.io.*;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

import au.com.bytecode.opencsv.*;

/**
 * Writes values to a CSV log file in a thread of its own.
 */
public class CsvLogWriter {
	
	private final String[] columnHeaders;
	private File outputFile;
	private BlockingQueue<Object[]> rows = new LinkedBlockingQueue<>();
	private CSV csv = CSV.separator('\t').create();
	private Thread rowWriterThread;
	
	public CsvLogWriter(File outputFile, String... columnHeaders) {
		this.outputFile = outputFile;
		this.columnHeaders = columnHeaders;
		
		writeColumnHeaders();
		startRowWriterThread();
	}
		
	public File getOutputFile() {
		return outputFile;
	}
	
	public CSV getCsvConfiguration() {
		return csv;
	}

	public void addRow(Object... row) {
		if (row.length != columnHeaders.length) {
			throw new IllegalArgumentException("Number of values does not match number of columns");
		}
		try {
			rows.put(row);
		} catch (InterruptedException e) {}
	}
	
	private void writeColumnHeaders() {
		csv.write(outputFile, new CSVWriteProc() {
			
			@Override
			public void process(CSVWriter out) {
				out.writeNext(CsvLogWriter.this.columnHeaders);
			}
			
		});
	}

	public void writeNextRow() {
		FileOutputStream fos;
		try {
			fos = new FileOutputStream(outputFile, true); // append to file
		} catch (FileNotFoundException e) {
			throw new RuntimeException(e);
		}
		
		try {
			Object[] row = rows.take();
			final String[] rowValues = new String[row.length];
			for (int i = 0; i < row.length; i++) {
				rowValues[i] = row[i] != null ? row[i].toString() : null;
			}
			
			csv.writeAndClose(fos, new CSVWriteProc() {

				@Override
				public void process(CSVWriter out) {
					out.writeNext(rowValues);
				}
				
			});
			
		} catch (InterruptedException e) {}
		
	}
	
	private void startRowWriterThread() {
		this.rowWriterThread = new Thread(new Runnable() {
			
			@Override
			public void run() {
				writeNextRow();
			}
		});
		this.rowWriterThread.setDaemon(true);
		this.rowWriterThread.start();
	}
	
	
}
