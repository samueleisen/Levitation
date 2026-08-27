// hairphysic.js
//"A lightweight Verlet physics system that drives 3 dynamic bone chains of any length, 
// mapping simulated particle positions back into bone quaternion rotations with sphere and capsule collisions."

// Verlet integration hair physics for GLB bone chains.
// Simulates depth 3+ hair bones with gravity + collision while keeping depth ≤ 2 untouched.

import * as THREE from 'three';
import { HairColliderHelper } from './hair-helper.js';

export { HairColliderHelper };

// ==========================================
// Module-level reusable temporaries (avoid GC in hot loop)
// _worldPos / _worldDir / _localDir / _up are truly stateless across chains
// so they remain at module scope. Quaternion accumulators (_parentWorldQuat,
// _invParentQuat) are now per-chain instance fields (see HairChain ctor)
// to eliminate shared-singleton mutation risk.
// ==========================================
const _worldPos = new THREE.Vector3();
const _worldDir = new THREE.Vector3();
const _localDir = new THREE.Vector3();
// NOTE: The old module-level _up constant has been removed.
// Each HairChain now holds its own this.boneForward (set from config.boneAxis)
// to support rigs that orient bones along axes other than +Y.
// Temporaries for collider transform computation
const _offsetVec = new THREE.Vector3();
const _rootQuat = new THREE.Quaternion();

// ==========================================
// Default Collider Configuration (empty by default)
// ==========================================
export const DEFAULT_COLLIDERS = [];

// ==========================================
// Verlet Particle
// ==========================================
class Particle {
    constructor(x, y, z, isPinned = false) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.oldX = x;
        this.oldY = y;
        this.oldZ = z;
        this.isPinned = isPinned;
        this.invMass = isPinned ? 0 : 1;
    }

    // NOTE: Particle.update() has been intentionally removed.
    // Integration is now performed inline inside HairPhysics._stepSimulation()
    // where dtSq is precomputed once per step rather than once per particle.
}

// ==========================================
// Distance Constraint (positional projection)
// ==========================================
class DistanceConstraint {
    constructor(p1, p2, distance) {
        this.p1 = p1;
        this.p2 = p2;
        this.distance = distance;
    }

    solve() {
        const dx = this.p2.x - this.p1.x;
        const dy = this.p2.y - this.p1.y;
        const dz = this.p2.z - this.p1.z;

        const currentDist = Math.sqrt(dx * dx + dy * dy + dz * dz) || 0.0001;
        const diff = (currentDist - this.distance) / currentDist;

        const totalInvMass = this.p1.invMass + this.p2.invMass;
        if (totalInvMass === 0) return;

        // p1Share / p2Share are already 0 for pinned particles (invMass = 0),
        // so these additions are mathematical no-ops — no explicit isPinned guard needed.
        const p1Share = this.p1.invMass / totalInvMass;
        const p2Share = this.p2.invMass / totalInvMass;

        this.p1.x += dx * diff * p1Share;
        this.p1.y += dy * diff * p1Share;
        this.p1.z += dz * diff * p1Share;

        this.p2.x -= dx * diff * p2Share;
        this.p2.y -= dy * diff * p2Share;
        this.p2.z -= dz * diff * p2Share;
    }
}

// ==========================================
// Hair Chain — one per strand (chair, lhair, rhair)
// ==========================================
class HairChain {
    /**
     * @param {THREE.Bone} anchorBone  - The depth-1 bone (e.g. chair1). Read-only parent reference, never modified.
     * @param {THREE.Bone[]} physicsBones - Depth 2+ bones [chair2, chair3, ...]. We set their quaternions.
     * @param {string} [boneAxis='+Y']  - Local axis the bone points along. One of '+Y','−Y','+X','−X','+Z','−Z'.
     *   Determines which local-space direction is mapped to the simulated direction vector in writeBack().
     *   Defaults to '+Y' (the standard Blender/Three.js bone convention).
     */
    constructor(anchorBone, physicsBones, boneAxis = '+Y') {
        this.anchorBone = anchorBone;
        this.physicsBones = physicsBones;
        this.particles = [];
        this.constraints = [];

        // Per-chain bone forward direction derived from boneAxis config.
        // Used in writeBack() instead of the module-level _up constant so that
        // rigs that orient bones along a different local axis work correctly.
        this.boneForward = new THREE.Vector3();
        switch (boneAxis) {
            case '+X': this.boneForward.set(1, 0, 0); break;
            case '-X': this.boneForward.set(-1, 0, 0); break;
            case '+Z': this.boneForward.set(0, 0, 1); break;
            case '-Z': this.boneForward.set(0, 0, -1); break;
            case '-Y': this.boneForward.set(0, -1, 0); break;
            default: this.boneForward.set(0, 1, 0); break; // '+Y'
        }

        // Per-chain quaternion temporaries — eliminates shared module-level
        // singleton mutation. Safe even if multiple HairChain instances exist.
        this._parentWorldQuat = new THREE.Quaternion();
        this._invParentQuat = new THREE.Quaternion();

        this._initFromBones();
    }

    _initFromBones() {
        // Particle 0: pinned at first physics bone's world position
        // (Its position is fixed by the untouched parent chain)
        this.physicsBones[0].getWorldPosition(_worldPos);
        this.particles.push(new Particle(_worldPos.x, _worldPos.y, _worldPos.z, true));

        // Free particles for remaining physics bones
        for (let i = 1; i < this.physicsBones.length; i++) {
            this.physicsBones[i].getWorldPosition(_worldPos);
            this.particles.push(new Particle(_worldPos.x, _worldPos.y, _worldPos.z, false));
        }

        // Distance constraints: use WORLD-SPACE distances between particles at rest pose.
        // NOTE: bone.json childDistance is in unscaled local space; the model may have
        // an armature scale (e.g. 0.45x), so we compute rest distances directly from
        // the actual world positions to avoid scale mismatch.
        for (let i = 0; i < this.particles.length - 1; i++) {
            const p1 = this.particles[i];
            const p2 = this.particles[i + 1];
            const dx = p2.x - p1.x;
            const dy = p2.y - p1.y;
            const dz = p2.z - p1.z;
            const restDist = Math.sqrt(dx * dx + dy * dy + dz * dz);
            if (restDist > 0.0001) {
                this.constraints.push(new DistanceConstraint(p1, p2, restDist));
            }
        }
    }

    /**
     * Sync the pinned (root) particle toward the first physics bone's world position.
     *
     * @param {number} inertia - [0–1] How slowly the root follows the bone.
     *   0 = hard-snap (original behaviour).
     *   0.3–0.6 = smooth lag making the chain feel less reactive / stiffer.
     *
     * When inertia > 0 the root particle is lerped toward the target each step,
     * carrying velocity (oldPos ≠ newPos) into the constraint system so the
     * whole chain responds to root motion proportionally rather than all at once.
     *
     * Safety: if the target is more than 0.5 world-units away (e.g. on respawn
     * or large jump), we hard-snap to avoid the hair floating mid-air.
     */
    updatePinnedParticle(inertia = 0) {
        this.physicsBones[0].getWorldPosition(_worldPos);
        const p = this.particles[0];

        if (inertia > 0) {
            const dx = _worldPos.x - p.x;
            const dy = _worldPos.y - p.y;
            const dz = _worldPos.z - p.z;
            const distSq = dx * dx + dy * dy + dz * dz;

            if (distSq > 0.25) {
                // Bone moved too far in one step (teleport / large jump) — snap immediately
                // so the chain doesn’t visibly drift in empty air.
                p.x = p.oldX = _worldPos.x;
                p.y = p.oldY = _worldPos.y;
                p.z = p.oldZ = _worldPos.z;
            } else {
                // Lerp the pin toward the bone.
                // Preserve old position so the constraint system sees a real velocity.
                p.oldX = p.x;
                p.oldY = p.y;
                p.oldZ = p.z;

                const alpha = 1.0 - inertia;  // inertia=0.7 → alpha=0.3 (30 % per step)
                p.x += dx * alpha;
                p.y += dy * alpha;
                p.z += dz * alpha;
            }
        } else {
            // inertia = 0: original hard-snap, zero velocity on pinned particle.
            p.x = p.oldX = _worldPos.x;
            p.y = p.oldY = _worldPos.y;
            p.z = p.oldZ = _worldPos.z;
        }
    }

    /**
     * Write particle world positions back as bone quaternion rotations.
     *
     * For each physics bone, compute the direction from its particle to the next
     * particle (world space), convert to the parent bone's local space, and set
     * the bone's quaternion to rotate local Y-axis to that direction.
     * We accumulate the parent world quaternion manually to avoid needing
     * matrix updates between bones.
     */
    writeBack() {
        // Start with the anchor bone's (depth-2) world quaternion.
        // Uses per-chain instance quaternions (FIX #3) instead of shared module singletons.
        this.anchorBone.getWorldQuaternion(this._parentWorldQuat);

        for (let i = 0; i < this.physicsBones.length; i++) {
            const bone = this.physicsBones[i];
            const p = this.particles[i];

            // Direction to next particle (or reuse last direction for leaf bone)
            if (i < this.particles.length - 1) {
                const pNext = this.particles[i + 1];
                _worldDir.set(
                    pNext.x - p.x,
                    pNext.y - p.y,
                    pNext.z - p.z
                );
                const len = _worldDir.length();
                if (len < 0.0001) {
                    // Degenerate — accumulate with identity and skip
                    this._parentWorldQuat.multiply(bone.quaternion);
                    continue;
                }
                _worldDir.divideScalar(len);
            }
            // else: leaf bone keeps _worldDir from previous iteration

            // Convert world direction to parent bone's local space
            this._invParentQuat.copy(this._parentWorldQuat).invert();
            _localDir.copy(_worldDir).applyQuaternion(this._invParentQuat);

            // Quaternion that rotates the bone's forward axis to this direction.
            // Uses per-chain boneForward (set from config.boneAxis) rather than the
            // module-level _up so that non-+Y rigs produce correct rotations.
            bone.quaternion.setFromUnitVectors(this.boneForward, _localDir);

            // Accumulate world quaternion for next bone in chain
            // next parent world quat = current parent world quat × this bone's local quat
            this._parentWorldQuat.multiply(bone.quaternion);
        }
    }
}

// ==========================================
// HairPhysics — main manager (exported)
// ==========================================
export class HairPhysics {
    constructor(collidersData = null) {
        this.chains = [];
        this.config = {
            gravity: -9.8,
            damping: 0.94,
            substeps: 10,
            inertia: 0.5,
        };

        this.colliderDefs = collidersData || DEFAULT_COLLIDERS;
        this.colliders = []; // compiled runtime colliders
        this.modelRoot = null;
        this.lastConfig = null;

        this.initialized = false;
        this._accumulator = 0; // Time accumulator for fixed-step integration
    }

    /** Set or replace collider definitions (e.g. from glb-collider.json). */
    loadColliders(jsonDefs) {
        this.colliderDefs = Array.isArray(jsonDefs) ? jsonDefs : [];
        this._buildColliders();
        this._updateColliders();
    }

    /**
     * Fetch hair chain definitions from a JSON file URL (e.g. './hair-config.json').
     * If the file does not exist (404, network error) or contains invalid data,
     * it safely returns null without throwing.
     *
     * @param {string} [url='./hair-config.json']
     * @returns {Promise<object|null>}
     */
    async fetchConfig(url = './hair-config.json') {
        try {
            const res = await fetch(url);
            if (!res.ok) {
                console.warn(`[HairPhysics] Hair config file not found at '${url}' (${res.status}).`);
                return null;
            }
            const data = await res.json();
            return data && typeof data === 'object' ? data : null;
        } catch (err) {
            console.warn(`[HairPhysics] Could not load hair config from '${url}':`, err.message);
            return null;
        }
    }

    /**
     * Fetch collider definitions from a JSON file URL (e.g. './glb-collider.json').
     * If the file does not exist (404, network error) or contains invalid data,
     * it safely falls back to loading no colliders ([]) without throwing.
     *
     * @param {string} [url='./glb-collider.json']
     * @returns {Promise<Array>} The loaded colliders array (or [] if failed).
     */
    async fetchColliders(url = './glb-collider.json') {
        try {
            const res = await fetch(url);
            if (!res.ok) {
                console.warn(`[HairPhysics] Collider file not found at '${url}' (${res.status}). Loaded 0 colliders.`);
                this.loadColliders([]);
                return [];
            }
            const data = await res.json();
            if (Array.isArray(data)) {
                this.loadColliders(data);
                return data;
            } else {
                console.warn(`[HairPhysics] Invalid collider data in '${url}' (expected array). Loaded 0 colliders.`);
                this.loadColliders([]);
                return [];
            }
        } catch (err) {
            console.warn(`[HairPhysics] Could not load colliders from '${url}': ${err.message}. Loaded 0 colliders.`);
            this.loadColliders([]);
            return [];
        }
    }

    _buildColliders() {
        this.colliders = [];
        for (const def of this.colliderDefs) {
            const pos = def.position || { x: 0, y: 0, z: 0 };
            const type = (def.type || 'sphere').toLowerCase();
            const radius = typeof def.radius === 'number' ? def.radius : 0.2;

            if (type === 'capsule') {
                const q = def.quaternion || { x: 0, y: 0, z: 0, w: 1 };
                const height = typeof def.height === 'number' ? def.height : 1.0;
                this.colliders.push({
                    name: def.name || 'Capsule',
                    type: 'capsule',
                    localPosition: new THREE.Vector3(pos.x, pos.y, pos.z),
                    localQuat: new THREE.Quaternion(q.x, q.y, q.z, q.w),
                    radius,
                    height,
                    rSq: radius * radius,
                    // Runtime world state
                    worldPos: new THREE.Vector3(),
                    worldQuat: new THREE.Quaternion(),
                    ax: 0, ay: 0, az: 0,
                    bx: 0, by: 0, bz: 0,
                    abx: 0, aby: 0, abz: 0,
                    abLenSq: 0,
                });
            } else {
                this.colliders.push({
                    name: def.name || 'Sphere',
                    type: 'sphere',
                    localPosition: new THREE.Vector3(pos.x, pos.y, pos.z),
                    radius,
                    rSq: radius * radius,
                    // Runtime world state
                    worldPos: new THREE.Vector3(),
                    cx: 0, cy: 0, cz: 0,
                });
            }
        }
    }

    /** Clear all chains and mark as uninitialized. */
    reset() {
        this.chains = [];
        this.modelRoot = null;
        this.initialized = false;
        this._accumulator = 0;
    }

    /**
     * Initialize hair physics from a loaded GLTF scene.
     *
     * @param {THREE.Object3D} gltfScene - The loaded GLTF scene root.
     * @param {object|null}    [config]  - Data-driven chain config (e.g. from hair-config.json).
     *   Shape: { chains: [{ anchor: string, bones: string[], boneAxis?: string }] }
     * @param {Array|null}     [colliderDefs] - Optional collider definitions array (overrides loaded colliders).
     */
    init(gltfScene, config = null, colliderDefs = null) {
        this.chains = [];
        this.modelRoot = gltfScene;

        const chainConfig = config || this.lastConfig;
        this.lastConfig = chainConfig;

        // Collect all bones from the scene graph
        const sceneBones = {};
        gltfScene.traverse(node => {
            if (node.isBone) sceneBones[node.name] = node;
        });

        // Ensure world matrices are current before reading rest positions.
        gltfScene.updateMatrixWorld(true);

        if (chainConfig && Array.isArray(chainConfig.chains) && chainConfig.chains.length > 0) {
            for (const chainDef of chainConfig.chains) {
                const anchorBone = sceneBones[chainDef.anchor];
                if (!anchorBone) {
                    console.warn(`[HairPhysics] Anchor bone '${chainDef.anchor}' not found, skipping chain.`);
                    continue;
                }

                const physicsBones = [];
                for (const boneName of (chainDef.bones || [])) {
                    const bone = sceneBones[boneName];
                    if (bone) {
                        physicsBones.push(bone);
                    } else {
                        console.warn(`[HairPhysics] Bone '${boneName}' not found, chain truncated.`);
                        break;
                    }
                }

                if (physicsBones.length < 2) {
                    console.warn(`[HairPhysics] Chain '${chainDef.anchor}' needs ≥2 physics bones, skipping.`);
                    continue;
                }

                this.chains.push(new HairChain(anchorBone, physicsBones, chainDef.boneAxis || '+Y'));
            }
        } else {
            console.info('[HairPhysics] No hair chain config provided. Initialized with 0 chains.');
        }

        // ── Collider setup ───────────────────────────────────────────────────────
        // If external collider defs were provided (e.g. from the auto-collider
        // generator or an imported hair-setup.json), use them in place of
        // DEFAULT_COLLIDERS. The runtime collider format is unchanged.
        if (colliderDefs && Array.isArray(colliderDefs) && colliderDefs.length > 0) {
            this.colliderDefs = colliderDefs;
        }

        this._buildColliders();
        this._updateColliders();

        this.initialized = this.chains.length > 0;
        console.log(`[HairPhysics] Initialized ${this.chains.length} hair chains ` +
            `(${this.chains.reduce((s, c) => s + c.particles.length, 0)} total particles).`);
    }

    // ------------------------------------------
    // Collision system
    // ------------------------------------------

    /**
     * Update collider world positions and orientations from model root.
     */
    _updateColliders() {
        if (!this.modelRoot) return;

        this.modelRoot.getWorldQuaternion(_rootQuat);

        for (const c of this.colliders) {
            // Transform local center to world position
            c.worldPos.copy(c.localPosition).applyMatrix4(this.modelRoot.matrixWorld);

            if (c.type === 'sphere') {
                c.cx = c.worldPos.x;
                c.cy = c.worldPos.y;
                c.cz = c.worldPos.z;
                c.rSq = c.radius * c.radius;
            } else if (c.type === 'capsule') {
                c.worldQuat.copy(_rootQuat).multiply(c.localQuat);

                // Capsule segment half-height along local Y
                const halfH = Math.max(0.0001, c.height * 0.5);
                _offsetVec.set(0, halfH, 0).applyQuaternion(c.worldQuat);

                c.ax = c.worldPos.x - _offsetVec.x;
                c.ay = c.worldPos.y - _offsetVec.y;
                c.az = c.worldPos.z - _offsetVec.z;

                c.bx = c.worldPos.x + _offsetVec.x;
                c.by = c.worldPos.y + _offsetVec.y;
                c.bz = c.worldPos.z + _offsetVec.z;

                c.abx = c.bx - c.ax;
                c.aby = c.by - c.ay;
                c.abz = c.bz - c.az;
                c.abLenSq = c.abx * c.abx + c.aby * c.aby + c.abz * c.abz;
                c.rSq = c.radius * c.radius;
            }
        }
    }

    /**
     * Push a particle outside all collision shapes.
     */
    _resolveCollisions(p) {
        for (const c of this.colliders) {
            if (c.type === 'sphere') {
                this._resolveSphere(p, c);
            } else if (c.type === 'capsule') {
                this._resolveCapsule(p, c);
            }
        }
    }

    /**
     * Push particle p outside one sphere collider.
     */
    _resolveSphere(p, s) {
        const sdx = p.x - s.cx;
        const sdy = p.y - s.cy;
        const sdz = p.z - s.cz;
        const sDistSq = sdx * sdx + sdy * sdy + sdz * sdz;

        if (sDistSq < s.rSq && sDistSq > 0.000001) {
            const dist = Math.sqrt(sDistSq);
            const factor = s.radius / dist;
            p.x = s.cx + sdx * factor;
            p.y = s.cy + sdy * factor;
            p.z = s.cz + sdz * factor;

            const odx = p.oldX - s.cx;
            const ody = p.oldY - s.cy;
            const odz = p.oldZ - s.cz;
            const oDistSq = odx * odx + ody * ody + odz * odz;
            if (oDistSq < s.rSq && oDistSq > 0.000001) {
                const oDist = Math.sqrt(oDistSq);
                const oFactor = s.radius / oDist;
                p.oldX = s.cx + odx * oFactor;
                p.oldY = s.cy + ody * oFactor;
                p.oldZ = s.cz + odz * oFactor;
            }
        }
    }

    /**
     * Push particle p outside one capsule collider.
     * @param {Particle} p   - Particle to resolve.
     * @param {object}   cap - Capsule state from _updateColliders().
     */
    _resolveCapsule(p, cap) {
        const apx = p.x - cap.ax;
        const apy = p.y - cap.ay;
        const apz = p.z - cap.az;
        let t = (apx * cap.abx + apy * cap.aby + apz * cap.abz) / (cap.abLenSq || 0.0001);
        t = Math.max(0, Math.min(1, t));

        const ccx = cap.ax + cap.abx * t;
        const ccy = cap.ay + cap.aby * t;
        const ccz = cap.az + cap.abz * t;

        const cdx = p.x - ccx;
        const cdy = p.y - ccy;
        const cdz = p.z - ccz;
        const cDistSq = cdx * cdx + cdy * cdy + cdz * cdz;

        if (cDistSq < cap.rSq && cDistSq > 0.000001) {
            const dist = Math.sqrt(cDistSq);
            const factor = cap.radius / dist;
            p.x = ccx + cdx * factor;
            p.y = ccy + cdy * factor;
            p.z = ccz + cdz * factor;

            const oapx = p.oldX - cap.ax;
            const oapy = p.oldY - cap.ay;
            const oapz = p.oldZ - cap.az;
            let ot = (oapx * cap.abx + oapy * cap.aby + oapz * cap.abz) / (cap.abLenSq || 0.0001);
            ot = Math.max(0, Math.min(1, ot));
            const occx = cap.ax + cap.abx * ot;
            const occy = cap.ay + cap.aby * ot;
            const occz = cap.az + cap.abz * ot;
            const ocdx = p.oldX - occx;
            const ocdy = p.oldY - occy;
            const ocdz = p.oldZ - occz;
            const ocDistSq = ocdx * ocdx + ocdy * ocdy + ocdz * ocdz;
            if (ocDistSq < cap.rSq && ocDistSq > 0.000001) {
                const oDist = Math.sqrt(ocDistSq);
                const oFactor = cap.radius / oDist;
                p.oldX = occx + ocdx * oFactor;
                p.oldY = occy + ocdy * oFactor;
                p.oldZ = occz + ocdz * oFactor;
            }
        }
    }

    // ------------------------------------------
    // Main update loop
    // ------------------------------------------

    /**
     * Run one frame of physics simulation. Call from animate() before render.
     * @param {number} dt - Real elapsed frame time (seconds) from clock.getDelta().
     */
    update(dt) {
        if (!this.initialized || dt <= 0) return;

        const fixedDT = 1 / 60;
        this._accumulator += Math.min(dt, 0.1);

        // Consume accumulated time in fixed-size slices
        while (this._accumulator >= fixedDT) {
            this._updateColliders();
            this._stepSimulation(fixedDT);
            this._accumulator -= fixedDT;
        }

        // Write final bone rotations after all steps for this frame are done
        for (const chain of this.chains) {
            chain.writeBack();
        }
    }

    /**
     * Advance the simulation by one fixed timestep.
     * Extracted from update() so the accumulator loop can call it N times.
     * @param {number} fixedDT - The fixed timestep in seconds (1/60).
     */
    _stepSimulation(fixedDT) {
        const { gravity, damping, substeps, inertia } = this.config;
        const dtSq = fixedDT * fixedDT; // Precomputed once per step, not per particle

        for (const chain of this.chains) {
            // 1. Sync pinned particle with the bone's current world position.
            //    inertia > 0 makes the root lerp instead of snap — see updatePinnedParticle().
            chain.updatePinnedParticle(inertia);

            // 2. Verlet integration: apply gravity to all free particles
            //    Start at index 1 — particle 0 is always pinned.
            for (let i = 1; i < chain.particles.length; i++) {
                const p = chain.particles[i];
                const vx = (p.x - p.oldX) * damping;
                const vy = (p.y - p.oldY) * damping;
                const vz = (p.z - p.oldZ) * damping;

                p.oldX = p.x;
                p.oldY = p.y;
                p.oldZ = p.z;

                p.x += vx;
                p.y += vy + gravity * dtSq;
                p.z += vz;
            }

            // 3. Sub-stepped constraint solving + collision
            //    Running collisions inside the substep loop ensures stable contact
            //    — constraints and collisions reinforce each other iteratively.
            for (let s = 0; s < substeps; s++) {
                for (const c of chain.constraints) {
                    c.solve();
                }
                // Collision resolution — skip index 0 (always pinned)
                for (let i = 1; i < chain.particles.length; i++) {
                    this._resolveCollisions(chain.particles[i]);
                }
            }
        }
    }

    // ------------------------------------------
    // Debug helper factory
    // ------------------------------------------

    /**
     * Create a live 3D visualizer for the collision shapes.
     * Add the returned helper's .group to your scene, then call helper.update()
     * each frame (after hairPhysics.update()).
     *
     * @returns {HairColliderHelper}
     */
    createDebugHelper() {
        return new HairColliderHelper(this);
    }

    /**
     * Convenience toggle — callers hold the helper reference directly,
     * so this is a no-op placeholder kept for API symmetry.
     */
    setDebug(_enabled) { /* callers use HairColliderHelper.setVisible() */ }

    // ------------------------------------------
    // Auto Chain Detection
    // ------------------------------------------

    /**
     * Traverse a GLTF scene's bone hierarchy and automatically detect
     * hair/physics bone chains based on topology (linear chain length ≥ 3)
     * and a blacklist of known non-hair bone name patterns.
     *
     * Returns a config object in the same format as hair-config.json:
     *   { chains: [{ anchor, bones, boneAxis }] }
     *
     * Does NOT call init() — pure scanner with no side effects.
     *
     * @param {THREE.Object3D} gltfScene
     * @returns {{ chains: Array<{ anchor: string, bones: string[], boneAxis: string }> }}
     */
    autoDetectChains(gltfScene) {
        const BLACKLIST = /body|cloth|collar|elbow|hand|foot|neutral|eye|jaw|ear|tongue|brow|mouth/i;
        const MIN_CHAIN_LENGTH = 3; // anchor + at least 2 physics bones

        // Collect all bones
        const allBones = [];
        gltfScene.traverse(node => { if (node.isBone) allBones.push(node); });
        gltfScene.updateMatrixWorld(true);

        // Find bones that are the START of a linear chain:
        // - Has exactly 1 bone child (linear, not branching)
        // - Parent is NOT also exactly-1-bone-child (i.e. this is the chain root, not the middle)
        const chainRoots = allBones.filter(bone => {
            const boneKids = bone.children.filter(c => c.isBone);
            if (boneKids.length !== 1) return false;
            const parentBoneKids = bone.parent?.isBone
                ? bone.parent.children.filter(c => c.isBone)
                : [];
            // A chain root: parent either is not a bone, or parent branches (length !== 1)
            return parentBoneKids.length !== 1;
        });

        const chains = [];

        for (const root of chainRoots) {
            // Trace full chain downward
            const chain = [];
            let current = root;
            while (current) {
                chain.push(current);
                const kids = current.children.filter(c => c.isBone);
                current = kids.length === 1 ? kids[0] : null;
            }

            // Filter 1: minimum length (anchor + ≥2 physics bones)
            if (chain.length < MIN_CHAIN_LENGTH) continue;

            // Filter 2: blacklist — skip if any bone in the chain matches
            if (chain.some(b => BLACKLIST.test(b.name))) continue;

            // Anchor = index 0 (chain root / attachment point), physics bones = index 1+
            const anchor = chain[0];
            const physicsBones = chain.slice(1);
            if (!anchor || physicsBones.length < 1) continue;

            // Auto-detect boneAxis from world-space direction
            const boneAxis = this._detectBoneAxis(anchor, physicsBones[0]);

            chains.push({
                anchor: anchor.name,
                bones: physicsBones.map(b => b.name),
                boneAxis,
            });
        }

        console.log(`[HairPhysics] autoDetectChains: found ${chains.length} chain(s).`);
        return { chains };
    }

    /**
     * Determine which local axis (+Y, -Y, +X, -X, +Z, -Z) of anchorBone
     * most closely aligns with the world-space direction from anchorBone
     * to firstPhysicsBone. Used to auto-set boneAxis.
     *
     * @param {THREE.Bone} anchorBone
     * @param {THREE.Bone} firstPhysicsBone
     * @returns {string}
     */
    _detectBoneAxis(anchorBone, firstPhysicsBone) {
        const AXES = [
            { axis: '+Y', dir: new THREE.Vector3(0, 1, 0) },
            { axis: '-Y', dir: new THREE.Vector3(0, -1, 0) },
            { axis: '+X', dir: new THREE.Vector3(1, 0, 0) },
            { axis: '-X', dir: new THREE.Vector3(-1, 0, 0) },
            { axis: '+Z', dir: new THREE.Vector3(0, 0, 1) },
            { axis: '-Z', dir: new THREE.Vector3(0, 0, -1) },
        ];

        const wp1 = new THREE.Vector3();
        const wp2 = new THREE.Vector3();
        anchorBone.getWorldPosition(wp1);
        firstPhysicsBone.getWorldPosition(wp2);

        // World direction anchor → first physics bone
        const worldDir = wp2.sub(wp1).normalize();

        // Convert to anchor's local space
        const invMat = new THREE.Matrix4().copy(anchorBone.matrixWorld).invert();
        const localDir = worldDir.clone().transformDirection(invMat);

        let best = '+Y', bestDot = -Infinity;
        for (const { axis, dir } of AXES) {
            const dot = localDir.dot(dir);
            if (dot > bestDot) { bestDot = dot; best = axis; }
        }
        return best;
    }
}
