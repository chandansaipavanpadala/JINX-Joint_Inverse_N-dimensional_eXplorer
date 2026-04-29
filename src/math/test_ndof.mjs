/**
 * Quick smoke-test for KinematicsNDOF.js
 * Run with:  node src/math/test_ndof.mjs
 */
import { fk, jacobian, ik_dls, SCARA_DH_CONFIG } from './KinematicsNDOF.js';

const DEG = (d) => d * Math.PI / 180;
const RAD = (r) => r * 180 / Math.PI;
const fmt = (v, d = 4) => v.map(x => x.toFixed(d));

console.log('═══════════════════════════════════════');
console.log('  KinematicsNDOF.js — Validation Tests');
console.log('═══════════════════════════════════════\n');

// ── TEST 1: FK at home position (all zeros) ──
console.log('TEST 1: FK at home position [0, 0, 0, 0]');
const q0 = [0, 0, 0, 0];
const fk0 = fk(q0, SCARA_DH_CONFIG);
console.log('  Position:', fmt(fk0.position));
// Expected: x = L1 + L2 = 0.55, y = 0, z = 0.35 (base height)
console.log('  Expected: [0.5500, 0.0000, 0.3500]');
console.log('  ✓ Pass:', Math.abs(fk0.position[0] - 0.55) < 1e-6 && Math.abs(fk0.position[2] - 0.35) < 1e-6);

// ── TEST 2: FK with J1 = 90° ──
console.log('\nTEST 2: FK with J1 = 90°');
const q1 = [DEG(90), 0, 0, 0];
const fk1 = fk(q1, SCARA_DH_CONFIG);
console.log('  Position:', fmt(fk1.position));
// Expected: x ≈ 0, y = 0.55, z = 0.35
console.log('  Expected: [0.0000, 0.5500, 0.3500]');
console.log('  ✓ Pass:', Math.abs(fk1.position[1] - 0.55) < 1e-4 && Math.abs(fk1.position[0]) < 1e-4);

// ── TEST 3: FK with prismatic joint ──
console.log('\nTEST 3: FK with J3 = 0.1 (prismatic 100mm)');
const q2 = [0, 0, 0.1, 0];
const fk2 = fk(q2, SCARA_DH_CONFIG);
console.log('  Position:', fmt(fk2.position));
// SCARA J2 has α=π, so Z flips. Prismatic moves along -Z_base → z should decrease
console.log('  Note: Z should differ from 0.35 due to prismatic along flipped Z');

// ── TEST 4: FK with J2 = 90° ──
console.log('\nTEST 4: FK with J2 = 90°');
const q3 = [0, DEG(90), 0, 0];
const fk3 = fk(q3, SCARA_DH_CONFIG);
console.log('  Position:', fmt(fk3.position));
// J2 bends the forearm 90° CCW in the XY plane

// ── TEST 5: Jacobian dimensions ──
console.log('\nTEST 5: Jacobian dimensions');
const jac = jacobian(q0, SCARA_DH_CONFIG);
console.log('  Shape: ' + jac.rows + '×' + jac.cols);
console.log('  ✓ Pass:', jac.rows === 6 && jac.cols === 4);

// Print Jacobian at home
console.log('  Jacobian at home:');
for (let r = 0; r < 6; r++) {
  const row = [];
  for (let c = 0; c < 4; c++) row.push(jac.J[r * 4 + c].toFixed(4));
  console.log('    [' + row.join(', ') + ']');
}

// ── TEST 6: IK round-trip ──
console.log('\nTEST 6: IK round-trip (DLS solver)');
const target = [0.40, 0.20, 0.30];
console.log('  Target:', target);
const ikResult = ik_dls(target, [0, 0, 0, 0], SCARA_DH_CONFIG, 200, 0.05);
console.log('  Solved q:', ikResult.q.map(v => RAD(v).toFixed(2) + '°'));
console.log('  Achieved:', fmt(ikResult.position));
console.log('  Error:', ikResult.error.toFixed(6), 'm');
console.log('  Converged:', ikResult.converged);
console.log('  Iterations:', ikResult.iterations);

// ── TEST 7: IK from a known FK pose ──
console.log('\nTEST 7: IK from known FK pose');
const qKnown = [DEG(30), DEG(-45), 0.05, 0];
const fkKnown = fk(qKnown, SCARA_DH_CONFIG);
console.log('  FK position:', fmt(fkKnown.position));
const ikBack = ik_dls(fkKnown.position, [0, 0, 0, 0], SCARA_DH_CONFIG, 200, 0.05);
console.log('  IK achieved:', fmt(ikBack.position));
console.log('  Error:', ikBack.error.toFixed(6), 'm');
console.log('  Converged:', ikBack.converged);

console.log('\n═══════════════════════════════════════');
console.log('  All tests complete.');
console.log('═══════════════════════════════════════');
