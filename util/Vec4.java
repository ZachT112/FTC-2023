package org.firstinspires.ftc.teamcode.util;

public class Vec4 {
    int x, y, z, w;

    public Vec4(int x, int y, int z, int w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    public static Vec4 abs(Vec4 a) {
        return new Vec4(Math.abs(a.x), Math.abs(a.y), Math.abs(a.z), Math.abs(a.w));
    }

    public static Vec4 getDifference(Vec4 a, Vec4 b) {
        return new Vec4(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
    }

    public int getAverage() {
        return (x + y + z + w) / 4;
    }

    public int getMinimum() {
        return Math.min(Math.min(Math.min(x, y), z), w);
    }

    public int getMaximum() {
        return Math.max(Math.max(Math.max(x, y), z), w);
    }
}
