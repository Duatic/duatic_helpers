# Proofs for `KinematicTrajectoryExponentialApproach`

This document collects the mathematical background for the exponential-approach
trajectory implemented in
[`kinematic_trajectory_exponential_approach.hpp`](kinematic_trajectory_exponential_approach.hpp),
covering both the double-pole (C1, velocity-continuous) variant in
[`kinematic_trajectory_exponential_approach_C1.hpp`](kinematic_trajectory_exponential_approach_C1.hpp)
and the triple-pole (C2, acceleration-continuous) variant in
[`kinematic_trajectory_exponential_approach_C2.hpp`](kinematic_trajectory_exponential_approach_C2.hpp).

# C1 (Twist-continuity) variant

## Definitions

The trajectory converges towards `goal` with convergence rate `omega > 0`:

$$
x(t) = \text{goal} + (A + B t)\,e^{-\omega t}
$$

$$
x'(t) = \big(B - \omega (A + Bt)\big) e^{-\omega t} = \big(v_0 - \omega B t\big) e^{-\omega t}
$$

$$
x''(t) = \omega\big(\omega (A+Bt) - 2B\big) e^{-\omega t}
$$

where

$$
A = x_0 - \text{goal} \quad \text{(initial offset from the goal)}, \qquad
B = v_0 + \omega A \quad \text{(initial velocity, corrected for the offset)}.
$$

`goal`, `x_0`, `v_0`, `A`, `B` are vectors;<br>
`omega > 0`, `t >= 0` are scalars.<br>
$\lVert\cdot\rVert$ denotes the Euclidean (2-)norm.

---

### Initial Conditions
$$
x(0) = \text{goal} + A = x_0
$$

$$
x'(0) = v_0
$$

### Terminal Conditions
$$
x(\infty) = goal
$$

$$
x'(\infty) = 0
$$

$$
x''(\infty) = 0
$$
---

## Invariant

> **Claim.** For all $t \ge 0$,
> $$
> \lVert x'(t)\rVert \;\le\; \max\left\{\lVert x'(0)\rVert,\; \frac{\lVert v_0\rVert + \omega\lVert A\rVert}{e}\right\}
> $$

This is the bound used by the `ALTERNATIVE: Approximate V-Limit` note in the header: it lets a
maximum-speed constraint be (conservatively) satisfied by choosing `omega` appropriately,
without needing to explicitly solve for the true maximum of $\lVert x'(t)\rVert$ where mathematically no close-form solution exists.

### Setup

Let $a = \lVert v_0\rVert$ and $c = \lVert A\rVert$. Substituting $B = v_0 + \omega A$ into $x'(t)$:

$$
x'(t) = \Big[(1-\omega t)\,v_0 - \omega^2 t\,A\Big] e^{-\omega t}. \tag{1}
$$

Since $e^{-\omega t} > 0$ is a scalar,

$$
\lVert x'(t)\rVert = e^{-\omega t}\,\big\lVert (1-\omega t) v_0 - \omega^2 t A \big\rVert,
$$

and $x'(0) = v_0$, so $\lVert x'(0)\rVert = a$. We must show

$$
\lVert x'(t)\rVert \le \max\left\{a,\ \tfrac{a+\omega c}{e}\right\} \quad \text{for all } t \ge 0. \tag{★}
$$

### Step 1 — The worst-case relative direction of $v_0$ and $A$ is collinear

Let $s = \cos\theta \in [-1, 1]$ be the cosine of the angle between $v_0$ and $A$. From (1), define
the quantity we actually care about — the squared norm itself, **including the decay factor**:

$$
G(t,s) := \lVert x'(t)\rVert^2 = e^{-2\omega t}\Big[(1-\omega t)^2 a^2 - 2(1-\omega t)\,\omega^2 t\, a c\, s + \omega^4 t^2 c^2\Big] .
$$

> ⚠️ It is tempting to drop the $e^{-2\omega t}$ factor here and work with the bracket alone — but that
> quantity grows like $\omega^4c^2t^2$ as $t\to\infty$ and is therefore **unbounded**. Any argument built
> on $\sup_{t\ge0}$ of the undecayed bracket is vacuous ($\infty \le \infty$). The factor must stay.

For **fixed $t$**, $G(t,\cdot)$ is still *affine* in $s$ — $e^{-2\omega t}$ is just a positive constant
for that $t$, so multiplying by it does not affect affineness in $s$. Hence

$$
M(s) := \sup_{t \ge 0} G(t, s)
$$

is a supremum of a family of affine functions of $s$, so $M$ is **convex** on $[-1,1]$. Moreover $M(s)$
is finite for every $s$: $G(t,s) \to 0$ as $t \to \infty$ and $G(\cdot,s)$ is continuous on $[0,\infty)$,
so it is bounded. A convex function on an interval attains its maximum at an endpoint, so

$$
\sup_{t \ge 0} \lVert x'(t)\rVert^2 = M(s) \;\le\; \max\big(M(1),\ M(-1)\big)
\quad\Longrightarrow\quad
\sup_{t \ge 0} \lVert x'(t)\rVert \;\le\; \max\Big(\sqrt{M(1)},\ \sqrt{M(-1)}\Big).
$$

In other words: for fixed norms $a = \lVert v_0\rVert$, $c = \lVert A\rVert$, the worst case over all
possible relative orientations occurs when $A$ is **exactly parallel or exactly anti-parallel** to
$v_0$. It therefore suffices to prove (★) in these two collinear cases.

### Step 2 — Two scalar lemmas

**Lemma A.** For $p, q \ge 0,\ \omega > 0$:

$$
\max_{t \ge 0} (p + qt)\, e^{-\omega t} =
\begin{cases}
p, & q \le \omega p \\[4pt]
\dfrac{q}{\omega}\, e^{\,\omega p / q - 1}, & q > \omega p
\end{cases}
$$

*Proof.* The derivative is $[q - \omega p - \omega q t]\,e^{-\omega t}$. If $q \le \omega p$, this is
$\le 0$ for all $t \ge 0$, so the function is non-increasing and its max is at $t = 0$. If
$q > \omega p$, the derivative vanishes at $t^* = (q - \omega p)/(\omega q) > 0$ (sign changes
$+ \to -$, i.e. a maximum); evaluating gives $p + q t^* = q/\omega$ and $\omega t^* = 1 - \omega p/q$,
hence the stated value. $\blacksquare$

**Lemma B.** For $p, q \ge 0,\ \omega > 0$:

$$
\max_{t \ge 0} \lvert p - qt \rvert\, e^{-\omega t} = \max\left\{p,\ \frac{q}{\omega e}\, e^{-\omega p / q}\right\}
$$

*Proof.* On $[0, p/q]$, $p - qt \ge 0$ and the derivative of $(p-qt)e^{-\omega t}$ is
$-[q + \omega(p - qt)]\,e^{-\omega t} \le 0$, so the max on this branch is at $t = 0$: value $p$. For
$t > p/q$, we maximize $(qt - p)\,e^{-\omega t}$; the derivative vanishes at
$t^* = \tfrac{1}{\omega} + \tfrac{p}{q}$, giving value $\tfrac{q}{\omega}\, e^{-1 - \omega p/q}$. Taking
the larger of the two branch maxima gives the result. $\blacksquare$

### Step 3 — Applying the lemmas to the collinear cases

Write $v_0 = a\,\hat e$, $A = \varepsilon c\,\hat e$ with $\varepsilon = \pm 1$. From (1), the scalar
coefficient along $\hat e$ is $a - \omega t (a + \varepsilon \omega c)$, so

$$
\lVert x'(t)\rVert = \big\lvert a - \omega t (a + \varepsilon\omega c) \big\rvert\, e^{-\omega t}.
$$

#### Case $\varepsilon = +1$ ($A \parallel v_0$)

Here $q := \omega(a + \omega c) \ge 0$, and Lemma B gives

$$
\max_t \lVert x'(t)\rVert = \max\left\{a,\ \frac{a+\omega c}{e}\, e^{-a/(a+\omega c)}\right\}
\le \max\left\{a,\ \frac{a+\omega c}{e}\right\},
$$

since the exponent is $\le 0$. This is exactly (★).

#### Case $\varepsilon = -1$ ($A \parallel -v_0$)

- **If $\omega c \le a$:** the coefficient $a - \omega t (a - \omega c)$ has
  $q = \omega(a - \omega c) \ge 0$; Lemma B gives max $\le a$ — (★) holds.

- **If $\omega c > a$:** the coefficient equals $a + \omega t (\omega c - a) > 0$ for all $t \ge 0$ —
  this is Lemma A's $(p+qt)$ form with $p = a$, $q = \omega(\omega c - a)$.

  - **If $\omega c \le 2a$:** Lemma A gives max $= a$ — (★) holds.

  - **If $\omega c > 2a$:** Lemma A gives

    $$
    \max_t \lVert x'(t)\rVert = (\omega c - a)\, \exp\!\left(\frac{a}{\omega c - a} - 1\right).
    $$

    Set $x = \dfrac{a}{\omega c - a} \in (0, 1)$ (valid since $\omega c > 2a \Rightarrow \omega c - a > a$).
    We need

    $$
    (\omega c - a)\, e^{x-1} \le \frac{a + \omega c}{e}
    \quad \Longleftrightarrow \quad
    e^{x} \le 1 + 2x .
    $$

    **Key inequality.** For $x \in [0, 1]$: $\;e^x \le 1 + 2x$.

    *Proof.* Let $\varphi(x) = 1 + 2x - e^x$, which is strictly concave ($\varphi'' = -e^x < 0$). Since
    $\varphi(0) = 0$ and $\varphi(1) = 3 - e > 0$, concavity implies $\varphi$ lies above the chord
    joining $(0,0)$ and $(1, 3-e)$, i.e. $\varphi(x) \ge x(3-e) \ge 0$ on $[0,1]$. $\blacksquare$

    Applying this with $x = a/(\omega c - a) \in (0,1)$:

    $$
    (\omega c - a)\, e^{x-1} = \frac{(\omega c - a)\, e^x}{e}
    \le \frac{(\omega c - a)(1 + 2x)}{e}
    = \frac{(\omega c - a) + 2a}{e}
    = \frac{a + \omega c}{e},
    $$

    using $(\omega c - a) \cdot 2x = 2a$ by definition of $x$. This establishes (★) in the last
    remaining sub-case.

### Conclusion

Every sub-case — both collinear orientations $\varepsilon = \pm 1$, and by the convexity argument of
Step 1, every possible relative angle between $v_0$ and $A$ — satisfies

$$
\lVert x'(t)\rVert \le \max\left\{\lVert x'(0)\rVert,\ \frac{\lVert v_0\rVert + \omega\lVert A\rVert}{e}\right\}
\qquad \text{for all } t \ge 0,
$$

which is the claimed invariant. $\blacksquare$

## Remarks

### Tightness
The bound is essentially tight: equality is approached when $A$ points opposite to $v_0$ (the
$\varepsilon = -1$, $\omega c > 2a$ regime) with $x \to 1$, i.e. $\omega\lVert A\rVert \to 2\lVert v_0\rVert$
— this is exactly the "velocity overshoots before decaying" scenario for this trajectory. The speed
can transiently exceed $\lVert v_0\rVert$, but never beyond $(\lVert v_0\rVert + \omega\lVert A\rVert)/e$.

### Intuition
* The system exponentially converges toward the desired goal with no position overshoot.
* The velocity is continuously adapted from initial velocity -> to a velocity toward the goal -> to eventually zero.<br>
  This way, the velocity exponentially converges toward zero with maximum one overshoot that never exceeds the given $v_{\max}$.<br>
  However, the trajectory may initially start $x'(0) = v_0$ at a velocity exceeding this limit, which is allowed.

## Implementation relation
This is the inequality used by the header's `V-Limit` approximation:

$$
\lVert x'(t)\rVert \le \frac{\lVert v_0\rVert + \omega\lVert A\rVert}{e} =: v_{\max}
$$
Goal: Find largest $\omega > 0$ that satisfies $\lVert x'(t)\rVert \le v_{\max}$.

As $\lVert v_0 \rVert$ might be too big and $\lVert A\rVert$ might be $0$, a minimum and maximum convergence rate, $\omega_{\min}$ and $\omega_{\max}$, are necessary.<br>
Thus:
$$
\omega = 
\begin{cases}
  \omega_{\min} &:\quad \omega_{\min} \lVert A\rVert >= e\, v_{\max} - \lVert v_0 \rVert \\
  \omega_{\max} &:\quad \omega_{\max} \lVert A\rVert <  e\, v_{\max} - \lVert v_0 \rVert \quad \text{: unequal to prefer stable $\omega_{\min}$ in the case of equality}\\
  \frac{e\, v_{\max} - \lVert v_0 \rVert}{\lVert A\rVert} &:\quad else
\end{cases}
$$

---

# C2 (Accel-continuity) variant

This section covers the triple-pole extension implemented in
[`kinematic_trajectory_exponential_approach_C2.hpp`](kinematic_trajectory_exponential_approach_C2.hpp), which
additionally matches the initial acceleration $a_0$ (so the resulting trajectory is continuous through pose,
twist *and* accel at the update time, hence "C2").

## Definitions

$$
x(t) = \text{goal} + (A + Bt + Ct^2)\,e^{-\omega t}
$$

with

$$
A = x_0 - \text{goal}, \qquad B = v_0 + \omega A, \qquad C = \frac{a_0 + 2\omega B - \omega^2 A}{2} = \frac{a_0 + 2\omega v_0 + \omega^2 A}{2}
$$

(the right-hand form of $C$ follows by substituting $B = v_0 + \omega A$; both are used interchangeably below).
Differentiating,

$$
x'(t) = \Big[(B-\omega A) + (2C-\omega B)t - \omega C\,t^2\Big]\,e^{-\omega t}
$$

$$
x''(t) = \Big[(2C - 2\omega B + \omega^2 A) + (\omega^2 B - 4\omega C)\,t + \omega^2 C\,t^2\Big]\,e^{-\omega t}
$$

`goal`, `x_0`, `v_0`, `a_0`, `A`, `B`, `C` are vectors; `omega > 0`, `t >= 0` are scalars;
$\lVert\cdot\rVert$ again denotes the Euclidean norm.

### Basis decomposition

Because the underlying ODE is linear, $e(t) := x(t) - \text{goal}$ is a superposition of three fixed
responses, one per matched initial condition (position offset, velocity, acceleration):

$$
e(t) = A\,\varphi_A(t) + v_0\,\varphi_v(t) + a_0\,\varphi_a(t)
$$

$$
\varphi_A(t) = \Big(1+\omega t+\tfrac{\omega^2}{2}t^2\Big)e^{-\omega t}, \qquad
\varphi_v(t) = \big(t+\omega t^2\big)e^{-\omega t}, \qquad
\varphi_a(t) = \tfrac{t^2}{2}\,e^{-\omega t}
$$

Differentiating each basis response,

$$
x'(t) = A\,\varphi_A'(t) + v_0\,\varphi_v'(t) + a_0\,\varphi_a'(t)
$$

$$
\varphi_A'(t) = -\tfrac{\omega^3}{2}t^2 e^{-\omega t}, \qquad
\varphi_v'(t) = \big(1+\omega t-\omega^2t^2\big)e^{-\omega t}, \qquad
\varphi_a'(t) = t\Big(1-\tfrac{\omega}{2}t\Big)e^{-\omega t}
$$

One can check $\varphi_A'(0)=0,\ \varphi_v'(0)=1,\ \varphi_a'(0)=0$, consistent with $x'(0)=v_0$ below.

### Initial Conditions

$$
x(0) = \text{goal} + A = x_0, \qquad x'(0) = v_0, \qquad x''(0) = a_0
$$

### Terminal Conditions

$$
x(\infty) = \text{goal}, \qquad x'(\infty) = 0, \qquad x''(\infty) = 0
$$

---

## Invariant

> **Claim.** For all $t \ge 0$,
> $$
> \lVert x'(t)\rVert \;\le\; \lVert v_0\rVert \;+\; \frac{2}{e^2}\,\omega\lVert A\rVert \;+\; \kappa\,\frac{\lVert a_0\rVert}{\omega},
> \qquad \kappa := (\sqrt2-1)\,e^{\sqrt2-2} \approx 0.2306.
> $$

Unlike the C1 case, $x'(t)$ is a *quadratic* (rather than linear) polynomial in $t$ times $e^{-\omega t}$: its
derivative $x''(t)$ can vanish at up to **two** interior points instead of one (see the Remarks below), so no
C1-shaped two-term $\max(\cdot,\cdot)$ bound is known to be exactly tight here. What follows instead is a
triangle-inequality bound over the three basis responses above — looser than the tightest possible bound, but
fully rigorous and cheap to state.

### Proof

By the basis decomposition, $x'(t)$ is a linear combination of the *constant* vectors $A, v_0, a_0$ with
*scalar*, time-varying coefficients $\varphi_A'(t), \varphi_v'(t), \varphi_a'(t)$. The triangle inequality gives,
for every $t \ge 0$,

$$
\lVert x'(t)\rVert \le \lVert A\rVert\,\lvert\varphi_A'(t)\rvert + \lVert v_0\rVert\,\lvert\varphi_v'(t)\rvert + \lVert a_0\rVert\,\lvert\varphi_a'(t)\rvert
\le \lVert A\rVert\,\overline{\varphi_A'} + \lVert v_0\rVert\,\overline{\varphi_v'} + \lVert a_0\rVert\,\overline{\varphi_a'},
$$

where $\overline{f} := \sup_{t \ge 0}\lvert f(t)\rvert$ — no collinear-worst-case reduction (à la the C1 proof's
Step 1) is needed here, since the triangle inequality already holds unconditionally for any relative
orientation of $A, v_0, a_0$. It remains to compute each of the three suprema; writing $u = \omega t$ makes all
three plain single-variable calculus exercises.

**$\overline{\varphi_A'}$.** $\varphi_A'(t) = -\tfrac{\omega}{2}u^2 e^{-u}$, so
$\overline{\varphi_A'} = \tfrac{\omega}{2}\sup_{u\ge0}u^2e^{-u}$. Since
$\frac{d}{du}\big(u^2e^{-u}\big) = (2u-u^2)e^{-u}$ vanishes at $u=2$ (a maximum, as the sign changes $+\to-$),
$\sup_{u\ge0}u^2e^{-u} = 4/e^2$, hence

$$
\overline{\varphi_A'} = \frac{\omega}{2}\cdot\frac{4}{e^2} = \frac{2\omega}{e^2}.
$$

**$\overline{\varphi_v'}$.** $\varphi_v'(t) = (1+u-u^2)e^{-u}$. Its derivative is
$\big[(1-2u)-(1+u-u^2)\big]e^{-u} = u(u-3)e^{-u}$, vanishing at $u=0$ (value $1$) and $u=3$ (value
$-5e^{-3}\approx-0.249$); since $u(u-3) < 0$ on $(0,3)$, $\varphi_v'$ decreases from $1$ to $-5e^{-3}$ there and
increases back toward $0$ for $u>3$. As $\lvert-5e^{-3}\rvert < 1$,

$$
\overline{\varphi_v'} = 1, \quad \text{attained at } u=0.
$$

**$\overline{\varphi_a'}$.** $\varphi_a'(t) = (u-\tfrac{u^2}{2})e^{-u}$. Its derivative is
$\big[(1-u)-(u-\tfrac{u^2}{2})\big]e^{-u} = \big(\tfrac{u^2}{2}-2u+1\big)e^{-u}$, vanishing at
$u = 2\pm\sqrt2$ (roots of $u^2-4u+2=0$). At $u=2-\sqrt2$:

$$
\varphi_a'\big(t\big) = (2-\sqrt2)\Big(1-\tfrac{2-\sqrt2}{2}\Big)e^{-(2-\sqrt2)} = (\sqrt2-1)\,e^{\sqrt2-2} =: \kappa \approx 0.2306
$$

(a local maximum), while at $u=2+\sqrt2$, $\varphi_a' = -(1+\sqrt2)e^{-(2+\sqrt2)} \approx -0.0794$ (a local
minimum, smaller in magnitude than $\kappa$). Hence

$$
\overline{\varphi_a'} = \kappa, \quad \text{attained at } u = 2-\sqrt2.
$$

Substituting the three suprema into the triangle-inequality bound gives

$$
\lVert x'(t)\rVert \le \lVert v_0\rVert\cdot 1 + \lVert A\rVert\cdot\frac{2\omega}{e^2} + \lVert a_0\rVert\cdot\frac{\kappa}{\omega},
$$

which is the claimed invariant. $\blacksquare$

## Remarks

### Tightness

Unlike the C1 bound, this one is **not** generally tight: the three suprema above are attained at three
*different* values of $u$ ($0$, $2$, and $2-\sqrt2$ respectively), so equality in the triangle inequality would
require $A$, $v_0$ and $a_0$ to simultaneously peak at the same $t$ — generically impossible. The true
supremum is smaller.

### Exact characterization

Since $e^{-\omega t}\to0$ monotonically and $x'(t) = Q(t)e^{-\omega t}$ for the quadratic
$Q(t) = (B-\omega A) + (2C-\omega B)t - \omega C t^2$, every extremum of $x'(t)$ on $[0,\infty)$ occurs either at
$t=0$ or at a root of $x''(t) = 0$, i.e. (writing $u=\omega t$) a root of

$$
C\,u^2 + (B\omega - 4C)\,u + (A\omega^2 - 2B\omega + 2C) = 0,
\qquad D = \omega^2(B^2-4AC) + 8C^2,
\qquad u_\pm = \frac{(4C-B\omega)\pm\sqrt D}{2C}
$$

(only real, non-negative roots correspond to points on the actual trajectory). So, in the worst-case collinear
configuration of $A, v_0, a_0$ (treated as signed scalars along a common axis), the *exact* peak speed is

$$
\lVert x'(t)\rVert \le \max\big(\lVert v_0\rVert,\ \lvert x'(t_+)\rvert,\ \lvert x'(t_-)\rvert\big), \qquad t_\pm = u_\pm/\omega,
$$

evaluating $x'$ at whichever of $t_\pm$ are real and non-negative. Unlike the C1 case's single linear
stationarity condition, this quadratic one has no clean closed-form inverse for $\omega$ in general — both
roots depend jointly on $A, B, C$ (via $D$), and $x'$ must be evaluated at each individually — which is why the
implementation below falls back to the simpler (if looser) two-envelope approximation rather than solving this
exactly.

### Intuition

* Same convergence behavior as C1 (no position overshoot, everything decays to the goal), but now also
  matching the initial *acceleration* exactly, rather than only pose and twist.
* Because $x'(t)$ can now have two interior extrema instead of one, the velocity profile can have a more
  complex shape (e.g. overshoot, partially recover, overshoot again) than C1's single-hump profile, especially
  when $a_0$ is large and opposes the direction of approach.

## Implementation relation

`determine_omega()` doesn't attempt to invert the exact (root-of-a-quadratic) characterization above — instead
it uses two simpler, independently-invertible proxy constraints, each an algebraic rearrangement of one
dominant piece of the (rigorously proven) sum invariant, requiring both to individually stay under $v_{\max}$:

$$
\text{(linear term)} \qquad \frac{\lVert v_0\rVert + 2\omega\lVert A\rVert}{e^2} \le v_{\max}
$$

$$
\text{(quadratic term)} \qquad \frac{\lVert v_0\rVert + \dfrac{\lVert a_0\rVert}{2\omega} + \dfrac{\omega\lVert A\rVert}{2}}{e} \le v_{\max}
$$

As with the C1 "Approximate V-Limit" section, this trades exactness for a closed form: each constraint alone is
easy to invert for $\omega$, and their intersection (the smaller of the two resulting $\omega$ estimates) is used
as a practical, not provably-tight, stand-in for the exact bound above — matching the spirit of C1's own
estimate, just with an extra term for $a_0$.

**Linear term** is monotonically increasing in $\omega$ (the same shape as the C1 bound, just rescaled: $e^2$
instead of $e$, $2\lVert A\rVert$ instead of $\lVert A\rVert$), so it simply clamps $\omega$ from above,
analogously to C1:

$$
\omega_{\text{lin}} =
\begin{cases}
  \omega_{\min} &:\quad \omega_{\min}\cdot 2\lVert A\rVert \ge e^2 v_{\max} - \lVert v_0\rVert \\
  \omega_{\max} &:\quad \omega_{\max}\cdot 2\lVert A\rVert < e^2 v_{\max} - \lVert v_0\rVert \\
  \dfrac{e^2 v_{\max} - \lVert v_0\rVert}{2\lVert A\rVert} &:\quad \text{else}
\end{cases}
$$

**Quadratic term** rearranges (multiplying through by $2\omega > 0$) to
$\lVert A\rVert\,\omega^2 - 2\big(e\,v_{\max}-\lVert v_0\rVert\big)\,\omega + \lVert a_0\rVert \le 0$ — an
upward parabola in $\omega$, feasible between its two roots $\omega_-\le\omega_+$ (if real). The desired answer
is always the point in $[\omega_{\min},\omega_{\max}]$ closest to $\omega_+$: if $\omega_+$ itself falls outside
that range, the least-bad choice is simply the nearest endpoint, i.e. exactly
$\mathrm{clamp}(\omega_+, \omega_{\min}, \omega_{\max})$. $\omega_-$ never changes that answer (so it is not
computed at all): $\omega_-\le\omega_+$ always holds, so moving from $\omega_+$ *toward* $[\omega_{\min},
\omega_{\max}]$ can only ever move into the feasible interval $[\omega_-,\omega_+]$ or stop short of it, never
overshoot past $\omega_-$ out the other side. Writing $v_{\text{dec}} := e\,v_{\max}-\lVert v_0\rVert$ and
$D_2 := v_{\text{dec}}^2 - \lVert A\rVert\lVert a_0\rVert$:

$$
\omega_{\text{sq}} =
\begin{cases}
  \omega_{\max} &:\quad \lVert A\rVert = 0 \quad \text{(no upper bound from this term at all)} \\
  \mathrm{clamp}\Big(\sqrt{\lVert a_0\rVert / \lVert A\rVert},\ \omega_{\min},\ \omega_{\max}\Big) &:\quad D_2 < 0 \quad \text{(this term's minimum alone exceeds } v_{\max}\text{)} \\
  \mathrm{clamp}\left(\dfrac{v_{\text{dec}} + \sqrt{D_2}}{\lVert A\rVert},\ \omega_{\min},\ \omega_{\max}\right) &:\quad \text{else}
\end{cases}
$$

**Combining** the two, exactly as `determine_omega()` does:

$$
\omega = \min(\omega_{\text{lin}},\ \omega_{\text{sq}})
$$

which is safe to combine this way (without any extra clamping) precisely because $\mathrm{clamp}(\cdot,
\omega_{\min}, \omega_{\max})$ is monotonic: for a monotonic $f$, $\min(f(a), f(b)) = f(\min(a,b))$, so the min
of the two already-clamped candidates equals the clamp of their raw min.

As in the C1 case, separate linear and angular estimates are computed this way and the smaller $\omega$ of the
two is used, synchronizing both motions to the more restrictive one.
