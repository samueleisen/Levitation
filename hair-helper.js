// hair-helper.js
// Visual debug helpers for hair physics system (Capsule & Sphere colliders + Particles)

import * as THREE from 'three';

const PALETTE = [
    { fill: 0x6ec8fb, wire: 0x93c5fd }, // Blue/Cyan
    { fill: 0xfbb16e, wire: 0xfde047 }, // Amber/Yellow
    { fill: 0x34d399, wire: 0x86efac }, // Emerald/Green
    { fill: 0xa78bfa, wire: 0xc4b5fd }, // Purple/Violet
    { fill: 0xf472b6, wire: 0xf9a8d4 }  // Pink/Rose
];

// Particle sphere sizes
const PARTICLE_RADIUS_FREE   = 0.018;
const PARTICLE_RADIUS_PINNED = 0.028;

export class HairColliderHelper {
    /**
     * @param {import('./hairphysic.js').HairPhysics} hairPhysics - The HairPhysics instance to visualize.
     */
    constructor(hairPhysics) {
        this._physics = hairPhysics;
        this.group = new THREE.Group();
        this.group.name = 'HairColliderHelper';

        this.items = [];
        this._buildVisuals();
    }

    _buildVisuals() {
        this.dispose();

        if (!this._physics || !this._physics.colliders) return;

        this._physics.colliders.forEach((c, idx) => {
            const colors = PALETTE[idx % PALETTE.length];
            let geo;
            if (c.type === 'sphere') {
                geo = new THREE.SphereGeometry(c.radius, 12, 8);
            } else {
                geo = new THREE.CapsuleGeometry(c.radius, c.height, 4, 10);
            }

            const fillMat = new THREE.MeshBasicMaterial({
                color: colors.fill,
                transparent: true,
                opacity: 0.18,
                depthWrite: false,
                side: THREE.DoubleSide,
            });

            const wireMat = new THREE.MeshBasicMaterial({
                color: colors.wire,
                wireframe: true,
                transparent: true,
                opacity: 0.7,
                depthTest: false,
            });

            const fill = new THREE.Mesh(geo, fillMat);
            const wire = new THREE.Mesh(geo, wireMat);

            this.group.add(fill, wire);

            this.items.push({ collider: c, fill, wire, geo, fillMat, wireMat });
        });
    }

    /** Show or hide all debug meshes. */
    setVisible(v) {
        this.group.visible = v;
    }

    /** Rebuild all debug meshes to match current collider definitions. */
    rebuild() {
        this._buildVisuals();
    }

    /**
     * Sync mesh transforms with current physics collider state.
     * Call once per frame, after hairPhysics.update().
     */
    update() {
        const p = this._physics;
        if (!p || !p.initialized || !p.colliders) return;

        // If collider list length or object instances changed, rebuild visuals
        const refsChanged = this.items.length > 0 && p.colliders.length > 0 && this.items[0].collider !== p.colliders[0];
        if (this.items.length !== p.colliders.length || refsChanged) {
            this._buildVisuals();
        }

        for (const item of this.items) {
            const c = item.collider;
            item.fill.position.copy(c.worldPos);
            item.wire.position.copy(c.worldPos);

            if (c.type === 'capsule' && c.worldQuat) {
                item.fill.quaternion.copy(c.worldQuat);
                item.wire.quaternion.copy(c.worldQuat);
            }
        }
    }

    /** Dispose all geometries and materials. */
    dispose() {
        for (const item of this.items) {
            if (item.geo) item.geo.dispose();
            if (item.fillMat) item.fillMat.dispose();
            if (item.wireMat) item.wireMat.dispose();
            this.group.remove(item.fill);
            this.group.remove(item.wire);
        }
        this.items = [];
    }
}

// ==========================================
// HairParticleHelper — live Verlet particle visualizer
// ==========================================

/**
 * Renders the raw Verlet particles of every active HairChain as 3D spheres.
 * Position is synced directly from particle.x/y/z each frame — this is the
 * true simulation state, not the bone world positions.
 *
 * Usage:
 *   const particleHelper = new HairParticleHelper(hairPhysics);
 *   scene.add(particleHelper.group);
 *   // each frame, after hairPhysics.update():
 *   particleHelper.update();
 */
export class HairParticleHelper {
    /**
     * @param {import('./hairphysic.js').HairPhysics} hairPhysics
     */
    constructor(hairPhysics) {
        this._physics = hairPhysics;
        this.group = new THREE.Group();
        this.group.name = 'HairParticleHelper';

        // Shared geometries — one size per type, reused across all spheres
        this._geoFree   = new THREE.SphereGeometry(PARTICLE_RADIUS_FREE,   10, 8);
        this._geoPinned = new THREE.SphereGeometry(PARTICLE_RADIUS_PINNED, 10, 8);

        // Pinned particle material: bright white, always on top
        this._matPinned = new THREE.MeshBasicMaterial({
            color: 0xffffff,
            depthTest: false,
            transparent: true,
            opacity: 0.95,
        });

        // Per-chain materials for free particles (created in _build)
        this._chainMats = [];

        // Flat list of { mesh, particle } for fast per-frame update
        this._items = [];
        this._chainCount = 0;

        this._build();
    }

    /** (Re)create all meshes to match current chains. Call after physics re-init. */
    rebuild() {
        this._destroy();
        this._build();
    }

    _build() {
        const physics = this._physics;
        if (!physics || !physics.chains) return;

        // Dispose old per-chain materials
        for (const mat of this._chainMats) mat.dispose();
        this._chainMats = [];

        for (let ci = 0; ci < physics.chains.length; ci++) {
            const chain = physics.chains[ci];
            const colors = PALETTE[ci % PALETTE.length];

            // Free-particle material, per chain color
            const mat = new THREE.MeshBasicMaterial({
                color: colors.fill,
                depthTest: false,
                transparent: true,
                opacity: 0.85,
            });
            this._chainMats.push(mat);

            for (let pi = 0; pi < chain.particles.length; pi++) {
                const particle = chain.particles[pi];
                const isPinned = particle.isPinned;

                const mesh = new THREE.Mesh(
                    isPinned ? this._geoPinned : this._geoFree,
                    isPinned ? this._matPinned : mat
                );
                mesh.position.set(particle.x, particle.y, particle.z);
                this.group.add(mesh);
                this._items.push({ mesh, particle });
            }
        }

        this._chainCount = physics.chains.length;
    }

    _destroy() {
        for (const { mesh } of this._items) {
            this.group.remove(mesh);
            // geometries and per-chain materials are shared — disposed in dispose()
        }
        this._items = [];
    }

    /** Show or hide the particle overlay. */
    setVisible(v) {
        this.group.visible = v;
    }

    /**
     * Sync all particle sphere positions from live simulation state.
     * Call once per frame after hairPhysics.update().
     */
    update() {
        const physics = this._physics;
        if (!physics || !physics.initialized) return;

        // Rebuild if chain count changed OR particle references changed (e.g. after physics reset/re-init)
        const refsChanged = physics.chains.length > 0 && this._items.length > 0 &&
                            this._items[0].particle !== physics.chains[0].particles[0];

        if (physics.chains.length !== this._chainCount || refsChanged) {
            this.rebuild();
        }

        for (const { mesh, particle } of this._items) {
            mesh.position.set(particle.x, particle.y, particle.z);
        }
    }

    /** Free all Three.js resources. */
    dispose() {
        this._destroy();
        this._geoFree.dispose();
        this._geoPinned.dispose();
        this._matPinned.dispose();
        for (const mat of this._chainMats) mat.dispose();
        this._chainMats = [];
    }
}
