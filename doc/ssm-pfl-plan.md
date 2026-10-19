# SSM+PFL baseline implementation plan

1. Keep KinovaSweepController's posture task, contact constraint and dynamics
   constraint. Require the installed Tasks QP backend for a custom linear
   inequality; reject other backends at initialization.
2. Extend the existing asynchronous SMPL-H snapshot with the selected MuJoCo
   body and body-local robot witness. Use tracked mesh vertex identities for
   human material-point velocity; allow a frozen playback frame.
3. On each controller tick, validate the sample, map each regional witness into
   the current mc_rtc model, calculate its world Jacobian and full joint-space
   inertia, then assemble one acceleration inequality per region. Reuse the
   existing Stop/hold path for invalid or near-contact data and log fallback.
4. Build the controller and bridge, validate the algebra and a reproducible
   synthetic scene, then compare predicted QP residuals with simulated motion.

This is a sampled simulation research baseline. The selected closest pair per
region does not cover all robot geometry pairs or guarantee the worst
directional margin. Moving human velocity is estimated from mesh material
vertices because the visual SMPL-H surface deforms independently of the
MuJoCo skeleton's rigid-body velocities.
