package com.pedropathing.util;

/**
 * Virtual clock. The build step (pedro_bridge.py build_follow / build_follow.ps1) rewrites the handful of
 * Pedro 2.1.0 core classes that read System.nanoTime()/currentTimeMillis() (PIDF controllers, PoseTracker,
 * Timer, TemporalCallback, Follower) to read this clock instead, so the REAL algorithms run faster than real
 * time with exact dt. nanoTime() advances 1 microsecond per call so back-to-back reads never return dt = 0.
 */
public final class SimClock {
    private static long base = 1_000_000_000L;
    private static long calls = 0;
    private SimClock() {}
    public static void advance(long nanos) { base += nanos; }
    public static long nanoTime() { return base + (++calls) * 1000L; }
    public static long currentTimeMillis() { return nanoTime() / 1_000_000L; }
}
