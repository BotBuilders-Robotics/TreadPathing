package org.treadpathing.route;

/**
 * A piece of non-driving work: raise an arm, spin an intake, drop a claw.
 *
 * <p>Deliberately not a functional interface. OnBotJava's Java 8 support is an opt-in
 * checkbox that is off by default, so team code cannot rely on lambdas. Actions are written
 * as anonymous inner classes:
 *
 * <pre>
 * Action raiseArm = new Action() {
 *     public void start() { arm.setTargetPosition(HIGH); }
 *     public boolean run() { return !arm.isBusy(); }
 * };
 * </pre>
 */
public interface Action {

    /** Called once, on the loop the action becomes active. */
    void start();

    /** Called every loop after {@link #start()}. Return true when the work is finished. */
    boolean run();
}
