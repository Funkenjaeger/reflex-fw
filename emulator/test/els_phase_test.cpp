/*
 * Permutation tests for the ELS phase-correction sign (els_phase.h).
 *
 * Models the lossless end-to-end ELS cycle — latch -> manual retract -> backlash
 * takeup -> applyPhaseCorrection -> synced feed -> Z-triggered stop — for every
 * VALID direction configuration, and asserts the groove phase at the stop is
 * INDEPENDENT of the configured backlash (the bug made it walk 2x the backlash).
 *
 * Direction degrees of freedom (signs the firmware/UI can produce):
 *   rs = sign(syncRatioNum)          (set by els_forward via direction_sign)
 *   sd = stopDirection               (set by els_forward via stop_direction_value)
 *   ks = DRO-vs-leadscrew polarity   (how a +leadscrew step moves the Z DRO)
 *
 * A configuration is physically VALID (the cut feeds toward the shoulder so the
 * stop can trigger, and the takeup loads the cutting wall) iff ks == sd*cuttingDir,
 * i.e. ks == sd*rs (cuttingDir == rs for positive geometry). Of those, the
 * UI-reachable ones additionally have sd == -rs (els_forward couples them). We
 * test every valid combo; invalid combos (carriage would feed away from the
 * shoulder — improper setup) are intentionally NOT tested, per design.
 *
 * Build/run: compiled as the `els_phase_test` CTest target (see CMakeLists).
 */
#include "els_phase.h"

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <algorithm>

/* Fixed geometry for the model (exactly divisible to avoid float fuzz):
 *   1 thread pitch = CPR spindle counts = TPS leadscrew steps = ZCPP DRO counts. */
static const int    CPR  = 6000;
static const float  TPS  = 1000.0f;
static const float  ZCPP = 400.0f;
static const int    DEN  = 6;        /* ratio magnitude = TPS/CPR = 1000/6000 = 1/6 */

/* DRO counts produced per leadscrew step, including the DRO polarity ks. */
static double droPerStep(int ks) { return (double)ZCPP / (double)TPS * (double)ks; }

/* Old (buggy) correction: phaseError = idealAdvance - actualAdvance. */
static int oldCorrectionSteps(int deltaSpindle, int deltaZ, int num, int den) {
  double ideal  = (double)deltaSpindle * num / den;
  double actual = (double)deltaZ * TPS / ZCPP;
  double pitch  = TPS;
  double corr   = std::fmod(ideal - actual, pitch);
  if (corr >  pitch / 2.0) corr -= pitch;
  if (corr < -pitch / 2.0) corr += pitch;
  int cuttingDir = (num > 0) ? 1 : -1;            /* TPS*ZCPP > 0 */
  if (cuttingDir * corr < 0.0) corr += cuttingDir * pitch;
  return (int)std::lround(corr);
}

/*
 * Simulate one stop-to-stop cycle and return the groove phase at the stop,
 * in spindle counts mod CPR. The reference (latch) is at spindle=0, Z=0.
 *
 *   useFix: true -> elsComputePhaseCorrection (the fix); false -> old form.
 */
static int cycleStopPhase(int rs, int sd, int ks, int backlash,
                          int sRet, int zRet, int sTk, bool useFix) {
  const int  num = rs;            /* syncRatioNum = rs (magnitude 1, den 6) */
  const int  den = DEN;
  const double dps = droPerStep(ks);

  /* Manual retract: leadscrew held, carriage parked at zRet, spindle turned sRet. */
  double Z = (double)zRet;
  double S = (double)sRet;

  /* Backlash takeup: leadscrew moves signedTakeup = cuttingDir*backlash (cuttingDir=rs). */
  int signedTakeup = rs * backlash;
  Z += signedTakeup * dps;        /* carriage follows the leadscrew (nut engaged) */
  S += sTk;                       /* spindle turns during the takeup */

  /* Phase correction at resume. */
  int deltaSpindle = (int)std::lround(S);
  int deltaZ       = (int)std::lround(Z);
  int add;
  if (useFix) {
    elsCorrResult_t r = elsComputePhaseCorrection(
        deltaSpindle, deltaZ, num, den, TPS, ZCPP, (int16_t)sd);
    add = r.stepsToAdd;
  } else {
    add = oldCorrectionSteps(deltaSpindle, deltaZ, num, den);
  }
  Z += add * dps;                 /* correction jog moves the carriage (instantaneous) */

  /* Synced feed to the shoulder (Z -> 0). dZ = dS * ratio * droPerStep. */
  double droPerSpindle = ((double)num / den) * dps;   /* = rs/6 * (0.4*ks) */
  double dSfeed = (0.0 - Z) / droPerSpindle;
  double Sstop  = S + dSfeed;

  long m = (long)std::llround(Sstop) % CPR;
  if (m < 0) m += CPR;
  return (int)m;
}

struct Cfg { int rs, sd, ks; const char* name; bool uiReachable; };

int main() {
  /* All 8 sign combos; keep only physically valid ones (ks == sd*rs). */
  std::vector<Cfg> all = {
    {+1, -1, -1, "els_forward=true  (HW: ratio>0, stopDir=-1)", true},
    {-1, +1, -1, "els_forward=false (ratio<0, stopDir=+1)",     true},
    {+1, +1, +1, "ratio>0, stopDir=+1 (non-UI, valid stop)",    false},
    {-1, -1, +1, "ratio<0, stopDir=-1 (non-UI, valid stop)",    false},
  };

  /* Backlash sweep (leadscrew steps), all non-trivial. Fixed retract/resume so
   * only the backlash varies — any phase spread is the backlash leak. */
  std::vector<int> backlashes = {100, 250, 400, 550, 700, 850, 1000};
  const int sRet = 21737;   /* spindle turned during retract (arbitrary, fixed) */
  const int zRet = 5174;    /* parked carriage offset (arbitrary, non-integer pitch) */
  const int sTk  = 1873;    /* spindle turned during takeup (arbitrary, fixed) */

  int failures = 0;

  printf("=== ELS phase-correction permutation tests ===\n");
  for (const Cfg& c : all) {
    /* sanity: only valid configs in the list */
    int cuttingDir = c.rs;  /* TPS*ZCPP>0 */
    if (c.ks != c.sd * cuttingDir) {
      printf("  [SKIP] %s — not a valid stop config\n", c.name);
      continue;
    }

    std::vector<int> fixPhase, oldPhase;
    for (int b : backlashes) {
      fixPhase.push_back(cycleStopPhase(c.rs, c.sd, c.ks, b, sRet, zRet, sTk, true));
      oldPhase.push_back(cycleStopPhase(c.rs, c.sd, c.ks, b, sRet, zRet, sTk, false));
    }
    auto spread = [](std::vector<int>& v) {
      int lo = *std::min_element(v.begin(), v.end());
      int hi = *std::max_element(v.begin(), v.end());
      return hi - lo;
    };
    int fixSpread = spread(fixPhase);
    int oldSpread = spread(oldPhase);

    /* Fix must hold the groove phase constant across all backlash values
     * (a few counts of lroundf jitter allowed). */
    const int TOL = 6;
    bool fixOk = fixSpread <= TOL;
    printf("  [%s] %-46s fixSpread=%d oldSpread=%d%s\n",
           fixOk ? "PASS" : "FAIL", c.name, fixSpread, oldSpread,
           c.uiReachable ? "  (UI-reachable)" : "");
    if (!fixOk) {
      failures++;
      printf("        fix phases:");
      for (int p : fixPhase) printf(" %d", p);
      printf("\n");
    }
    /* For UI-reachable configs the OLD code must visibly leak (regression guard:
     * proves the test exercises the bug). Non-UI configs happened to be immune. */
    if (c.uiReachable && oldSpread <= TOL) {
      failures++;
      printf("        [FAIL] old code did NOT leak on a UI config — test not exercising the bug\n");
    }
  }

  /* Also: with the fix, the stop phase must be independent of the RETRACT/resume
   * specifics too (not just backlash) — vary retract with backlash fixed. */
  printf("--- retract-independence (fix) ---\n");
  for (const Cfg& c : all) {
    if (c.ks != c.sd * c.rs) continue;
    std::vector<int> ph;
    for (int sr = 10000; sr <= 30000; sr += 5000)
      for (int zr = 1000; zr <= 9000; zr += 2000)
        ph.push_back(cycleStopPhase(c.rs, c.sd, c.ks, 613, sr, zr, sTk, true));
    int lo = *std::min_element(ph.begin(), ph.end());
    int hi = *std::max_element(ph.begin(), ph.end());
    bool ok = (hi - lo) <= 6;
    printf("  [%s] %-46s retractSpread=%d\n", ok ? "PASS" : "FAIL", c.name, hi - lo);
    if (!ok) failures++;
  }

  printf("=== %s ===\n", failures == 0 ? "ALL PASS" : "FAILURES");
  return failures == 0 ? 0 : 1;
}
