package me.drton.jmavsim;

/**
 * An object that can write its textual representation to a string builder.
 */
public interface ReportingObject {
    String newLine = System.getProperty("line.separator");

    void report(StringBuilder builder);
}
