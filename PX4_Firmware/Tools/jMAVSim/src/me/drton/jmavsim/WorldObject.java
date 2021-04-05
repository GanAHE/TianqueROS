package me.drton.jmavsim;

/**
 * User: ton Date: 02.02.14 Time: 11:33
 */
public abstract class WorldObject {
    protected final World world;

    public WorldObject(World world) {
        this.world = world;
    }

    public abstract void update(long t, boolean paused);

    public World getWorld() {
        return world;
    }
}
