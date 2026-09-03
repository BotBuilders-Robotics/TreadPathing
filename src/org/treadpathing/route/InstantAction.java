package org.treadpathing.route;

/**
 * An {@link Action} that finishes the instant it starts — setting a servo, flipping a flag.
 *
 * <pre>
 * Action openClaw = new InstantAction() {
 *     public void execute() { claw.setPosition(OPEN); }
 * };
 * </pre>
 */
public abstract class InstantAction implements Action {

    public abstract void execute();

    @Override
    public final void start() {
        execute();
    }

    @Override
    public final boolean run() {
        return true;
    }
}
