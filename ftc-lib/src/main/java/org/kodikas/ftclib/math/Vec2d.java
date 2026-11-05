package org.kodikas.ftclib.math;

/**
 * Mutable two-dimensional Cartesian vector in the right-handed NWU coordinate frame
 * ({@code +X = Forward}, {@code +Y = Left}, {@code +CCW = Positive Angle}).
 *
 * <p>Designed for strict zero-allocation ({@code 0 bytes/tick}) real-time control loops.
 * All in-place {@code *Into(Vec2d target)} operations use primitive stack temporaries so
 * aliasing ({@code target == this} or {@code target == other}) is always safe.</p>
 */
public final class Vec2d {
    /** Forward (+X) coordinate in meters (or meters/second for velocity vectors). */
    public double x;

    /** Left (+Y) coordinate in meters (or meters/second for velocity vectors). */
    public double y;

    /** Constructs a zero vector {@code (0.0, 0.0)}. */
    public Vec2d() {
        this(0.0, 0.0);
    }

    /**
     * Constructs a vector initialized to {@code (x, y)}.
     *
     * @param x forward (+X) component
     * @param y left (+Y) component
     */
    public Vec2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Static factory method for readable initialization outside the real-time loop.
     *
     * @param x forward (+X) component
     * @param y left (+Y) component
     * @return newly allocated {@link Vec2d}
     */
    public static Vec2d of(double x, double y) {
        return new Vec2d(x, y);
    }

    /**
     * Updates this vector's components in-place with zero allocations.
     *
     * @param x new forward (+X) component
     * @param y new left (+Y) component
     * @return {@code this} for fluent chaining
     */
    public Vec2d set(double x, double y) {
        this.x = x;
        this.y = y;
        return this;
    }

    /**
     * Copies components from {@code other} into this vector in-place.
     *
     * @param other source vector
     * @return {@code this} for fluent chaining
     */
    public Vec2d copyFrom(Vec2d other) {
        this.x = other.x;
        this.y = other.y;
        return this;
    }

    /**
     * Computes {@code target = this + other} with zero allocations.
     *
     * @param other  addend vector
     * @param target pre-allocated destination vector (may alias {@code this} or {@code other})
     */
    public void addInto(Vec2d other, Vec2d target) {
        double rx = this.x + other.x;
        double ry = this.y + other.y;
        target.x = rx;
        target.y = ry;
    }

    /**
     * Computes {@code target = this - other} with zero allocations.
     *
     * @param other  subtrahend vector
     * @param target pre-allocated destination vector (may alias {@code this} or {@code other})
     */
    public void subInto(Vec2d other, Vec2d target) {
        double rx = this.x - other.x;
        double ry = this.y - other.y;
        target.x = rx;
        target.y = ry;
    }

    /**
     * Computes {@code target = scalar * this} with zero allocations.
     *
     * @param scalar scale factor
     * @param target pre-allocated destination vector (may alias {@code this})
     */
    public void scaleInto(double scalar, Vec2d target) {
        target.x = this.x * scalar;
        target.y = this.y * scalar;
    }

    /**
     * Computes fused multiply-add {@code target = this + scale * other} with zero allocations.
     *
     * @param other  vector to scale and add
     * @param scale  scale factor applied to {@code other}
     * @param target pre-allocated destination vector (may alias {@code this} or {@code other})
     */
    public void addScaledInto(Vec2d other, double scale, Vec2d target) {
        double rx = this.x + other.x * scale;
        double ry = this.y + other.y * scale;
        target.x = rx;
        target.y = ry;
    }

    /**
     * Rotates this vector counter-clockwise by {@code angleRad} into {@code target}:
     * {@code [x cos(a) - y sin(a), x sin(a) + y cos(a)]}.
     *
     * @param angleRad counter-clockwise rotation angle in radians
     * @param target   pre-allocated destination vector (may alias {@code this})
     */
    public void rotateInto(double angleRad, Vec2d target) {
        rotateInto(Math.cos(angleRad), Math.sin(angleRad), target);
    }

    /**
     * Rotates this vector counter-clockwise using precomputed cosine and sine values into {@code target}.
     *
     * @param cos    {@code Math.cos(angleRad)}
     * @param sin    {@code Math.sin(angleRad)}
     * @param target pre-allocated destination vector (may alias {@code this})
     */
    public void rotateInto(double cos, double sin, Vec2d target) {
        double rx = this.x * cos - this.y * sin;
        double ry = this.x * sin + this.y * cos;
        target.x = rx;
        target.y = ry;
    }

    /**
     * Normalizes this vector to unit length into {@code target}. If the norm is below
     * {@link MathUtil#EPSILON}, writes {@code (0.0, 0.0)} to prevent division by zero.
     *
     * @param target pre-allocated destination vector (may alias {@code this})
     */
    public void normalizeInto(Vec2d target) {
        double n = Math.sqrt(this.x * this.x + this.y * this.y);
        if (n > MathUtil.EPSILON) {
            target.x = this.x / n;
            target.y = this.y / n;
        } else {
            target.x = 0.0;
            target.y = 0.0;
        }
    }

    /**
     * Rotates this vector {@code +90 degrees} counter-clockwise (left normal in NWU: {@code (-y, x)})
     * into {@code target}.
     *
     * @param target pre-allocated destination vector (may alias {@code this})
     */
    public void perpLeftInto(Vec2d target) {
        double rx = -this.y;
        double ry = this.x;
        target.x = rx;
        target.y = ry;
    }

    /**
     * Computes the Euclidean inner (dot) product {@code this.x * other.x + this.y * other.y}.
     *
     * @param other second vector
     * @return scalar dot product
     */
    public double dot(Vec2d other) {
        return this.x * other.x + this.y * other.y;
    }

    /**
     * Computes the 2D scalar (+Z) cross product {@code this.x * other.y - this.y * other.x}.
     *
     * @param other second vector
     * @return signed scalar cross product
     */
    public double cross(Vec2d other) {
        return this.x * other.y - this.y * other.x;
    }

    /**
     * Computes the Euclidean norm (magnitude) {@code sqrt(x^2 + y^2)}.
     *
     * @return vector magnitude
     */
    public double norm() {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    /**
     * Computes the squared Euclidean norm {@code x^2 + y^2}.
     *
     * @return squared vector magnitude
     */
    public double normSq() {
        return this.x * this.x + this.y * this.y;
    }

    /**
     * Computes the Euclidean distance between {@code this} and {@code other}.
     *
     * @param other target point
     * @return distance in meters
     */
    public double distanceTo(Vec2d other) {
        double dx = this.x - other.x;
        double dy = this.y - other.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Computes the squared Euclidean distance between {@code this} and {@code other}.
     *
     * @param other target point
     * @return squared distance in meters^2
     */
    public double distanceSqTo(Vec2d other) {
        double dx = this.x - other.x;
        double dy = this.y - other.y;
        return dx * dx + dy * dy;
    }

    /**
     * Computes the counter-clockwise polar angle {@code atan2(y, x)} in radians within {@code [-pi, pi]}.
     *
     * @return direction angle in radians
     */
    public double angleRad() {
        return Math.atan2(this.y, this.x);
    }

    @Override
    public String toString() {
        return "Vec2d(" + x + ", " + y + ")";
    }
}
