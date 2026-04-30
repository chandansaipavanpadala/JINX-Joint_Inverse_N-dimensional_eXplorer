import { trapProfile } from '../math/Trajectory.js';

export class WeldingStateMachine {
    constructor() {
        this.states = {
            IDLE: 0,
            APPROACH: 1,
            WELDING: 2,
            RETRACT: 3,
            RETURN: 4,
        };

        this.currentState = this.states.IDLE;

        // Default rest position
        this.homePos = [0.25, 0, 0.4];

        // Rectangular seam on the table
        // 5 waypoints: 4 corners + repeat of corner 0 to close the rectangle
        this.weldCorners = [
            [0.35, -0.15, 0.05],
            [0.55, -0.15, 0.05],
            [0.55, 0.15, 0.05],
            [0.35, 0.15, 0.05],
            [0.35, -0.15, 0.05],  // Close the rectangular seam
        ];

        this.hoverOffset = 0.15; // Z-offset for approach/retract

        this.pStart = [...this.homePos];
        this.pTarget = [...this.homePos];

        this.timer = 0;
        this.duration = 2; // seconds
        this._cornerIndex = 0;

        // Callbacks
        this.onTaskStateChange = null;
        this.onWeldActive = null;
    }

    start() {
        if (this.currentState === this.states.IDLE) {
            this._transitionTo(this.states.APPROACH);
        }
    }

    stop() {
        if (this.currentState !== this.states.IDLE) {
            this._transitionTo(this.states.IDLE);
            if (this.onWeldActive) this.onWeldActive(false);
        }
    }

    _transitionTo(newState) {
        this.currentState = newState;
        this.timer = 0;

        if (this.onTaskStateChange) {
            const stateName = Object.keys(this.states).find(k => this.states[k] === newState);
            this.onTaskStateChange(stateName);
        }

        if (newState === this.states.APPROACH) {
            this.pTarget = [
                this.weldCorners[0][0],
                this.weldCorners[0][1],
                this.weldCorners[0][2] + this.hoverOffset
            ];
            this.duration = 2;
        } else if (newState === this.states.WELDING) {
            this._cornerIndex = 0;
            this._setupWeldSegment();
            if (this.onWeldActive) this.onWeldActive(true);
        } else if (newState === this.states.RETRACT) {
            if (this.onWeldActive) this.onWeldActive(false);
            this.pStart = [...this.weldCorners[this._cornerIndex]];
            this.pTarget = [
                this.pStart[0],
                this.pStart[1],
                this.pStart[2] + this.hoverOffset
            ];
            this.duration = 1.5;
        } else if (newState === this.states.RETURN) {
            this.pTarget = [...this.homePos];
            this.duration = 2.5;
        } else if (newState === this.states.IDLE) {
            this.pStart = [...this.homePos];
            this.pTarget = [...this.homePos];
        }
    }

    _setupWeldSegment() {
        // When just starting to weld, we start from the hover point
        this.pStart = this._cornerIndex === 0
            ? [this.weldCorners[0][0], this.weldCorners[0][1], this.weldCorners[0][2] + this.hoverOffset]
            : [...this.weldCorners[this._cornerIndex - 1]];

        this.pTarget = [...this.weldCorners[this._cornerIndex]];
        this.duration = this._cornerIndex === 0 ? 1.0 : 2.0;
    }

    _setupNextWeldSegment() {
        this.pStart = [...this.weldCorners[this._cornerIndex]];
        this._cornerIndex++;
        this.pTarget = [...this.weldCorners[this._cornerIndex]];
        // Adjust duration arbitrarily based on segment distance or fix it
        this.duration = 2.0;
    }

    update(dt, currentPos) {
        if (this.currentState === this.states.IDLE) {
            return this.homePos;
        }

        // First frame initialization of pStart from actual given position if provided
        if (this.timer === 0 && currentPos) {
            this.pStart = [...currentPos];
        }

        this.timer += dt;

        // Evaluate trajectory profile
        let s = 1.0;
        if (this.timer < this.duration) {
            const { s: sVal } = trapProfile(this.timer, this.duration, 1.5, 3.0);
            s = sVal;
        }

        // Cartesian interpolation
        const x = this.pStart[0] + (this.pTarget[0] - this.pStart[0]) * s;
        const y = this.pStart[1] + (this.pTarget[1] - this.pStart[1]) * s;
        const z = this.pStart[2] + (this.pTarget[2] - this.pStart[2]) * s;

        // Phase transition check
        if (this.timer >= this.duration) {
            if (this.currentState === this.states.APPROACH) {
                this._transitionTo(this.states.WELDING);
            } else if (this.currentState === this.states.WELDING) {
                if (this._cornerIndex < this.weldCorners.length - 1) {
                    this._setupNextWeldSegment();
                    this.timer = 0;
                } else {
                    this._transitionTo(this.states.RETRACT);
                }
            } else if (this.currentState === this.states.RETRACT) {
                this._transitionTo(this.states.RETURN);
            } else if (this.currentState === this.states.RETURN) {
                this._transitionTo(this.states.IDLE);
            }
        }

        return [x, y, z];
    }
}
