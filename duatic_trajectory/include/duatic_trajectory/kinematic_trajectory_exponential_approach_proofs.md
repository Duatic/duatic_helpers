# Proofs for `KinematicTrajectoryExponentialApproach`

This document collects the mathematical background for the exponential-approach
trajectory implemented in
[`kinematic_trajectory_exponential_approach.hpp`](kinematic_trajectory_exponential_approach.hpp),
covering both the double-pole (C1, velocity-continuous) variant in
[`kinematic_trajectory_exponential_approach_C1.hpp`](kinematic_trajectory_exponential_approach_C1.hpp)
and the accel-continuous (C2) variant in
[`kinematic_trajectory_exponential_approach_C2.hpp`](kinematic_trajectory_exponential_approach_C2.hpp), which is
built as an *additive patch* $x_{C2}(t) = x_{C1}(t) + h(t)$ on top of C1 rather than as a separate
triple-pole polynomial trajectory.
The C1 section covers both of that variant's limits: the velocity (`V-Limit`) bound on $x'(t)$, and
the acceleration (`A-Limit`) bound on $x''(t)$ enforced by `determine_acc_omega()`. The C2 section
covers both of $\omega_{pv}$'s own limits (`determine_omega_pv()`, extending C1's own bounds to
account for the full combined acceleration) and $\omega_a$'s own limits (`determine_omega_a()`,
governing how fast the patch term $h(t)$ itself decays).

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

## Acceleration (A-Limit) Invariant

> **Claim.** For all $t \ge 0$,
> $$
> \lVert x''(t)\rVert \;\le\; \max\big\{\lVert x''(0)\rVert,\ \lvert x''(t^*)\rvert\big\}, \qquad t^* = \frac{3}{\omega} - \frac{A}{B}\quad(\text{only when } t^*\ge0),
> $$
> and, writing $k := \omega A/B$ (as in the header note), $\lvert x''(t^*)\rvert$ exceeds $\lVert x''(0)\rVert$ **only** for
> $$
> k \in (k_{\text{th}},\,3), \qquad k_{\text{th}} := 2 - W(1/e) \approx 1.721535457,
> $$
> where $W$ is the Lambert $W$ function (the inverse of $w \mapsto w e^w$). Outside that window
> (including all $k > 3$, where $t^* < 0$ falls outside the trajectory's domain), $\lVert x''(0)\rVert$
> alone is the exact peak.

This is the bound behind the header's `ALTERNATIVE: Approximate A-Limit` note and the analysis
`determine_acc_omega()` implements. Unlike $x'(t)$ — whose worst case is governed by a single
stationary point of $x''(t)$ — $x''(t)$ is one derivative further into the same polynomial-times-exponential
family, so *its* extremum condition (a stationary point of $x'''(t)$, i.e. jerk $=0$) is only ever
relevant strictly inside $t>0$; the domain boundary $t=0$ is always a second candidate, and either
one can be the true global peak depending on the initial conditions.

### Setup — reduce to a single scalar critical-point problem

Substituting $B=v_0+\omega A$ into $x''(t) = \omega\big(\omega(A+Bt)-2B\big)e^{-\omega t}$ and
expanding shows that $x''(t)$, exactly like $x'(t)$, is a linear combination of the *constant*
vectors $A, v_0$ with scalar, time-varying coefficients. By the **same convexity argument as Step 1**
above (only the coefficients' shape differs, not the structure of the argument), the worst case over
all relative orientations of $A$ and $v_0$ — for fixed norms $c=\lVert A\rVert,\ a=\lVert v_0\rVert$ —
again occurs when they are collinear. Writing $v_0 = a\hat e,\ A=\varepsilon c\hat e$
($\varepsilon=\pm1$) and $x''(t) = h(t)\,e^{-\omega t}$ reduces the problem to a single scalar linear
function $h(t) = Qt+P$.

**Lemma C** (single stationary point). For $h(t)=Qt+P$ ($P,Q\in\mathbb R,\ Q\ne0$) and $\omega>0$:
$$
\frac{d}{dt}\big[h(t)e^{-\omega t}\big] = \big[Q-\omega(Qt+P)\big]e^{-\omega t},
$$
which has a single zero at $t_{\text{ext}} = \tfrac1\omega - \tfrac PQ$. If $t_{\text{ext}}<0$,
$h(t)e^{-\omega t}$ is monotonic on all of $[0,\infty)$ (its derivative's sign only changes at
$t_{\text{ext}}$, which lies outside the domain), so
$$
\sup_{t\ge0}\lvert h(t)\rvert e^{-\omega t} = \lvert h(0)\rvert = \lvert P\rvert.
$$
If $t_{\text{ext}}\ge0$, $h(t)e^{-\omega t}$ is separately monotonic on $[0,t_{\text{ext}}]$ and on
$[t_{\text{ext}},\infty)$ (same reason) and decays to $0$ as $t\to\infty$, so
$$
\sup_{t\ge0}\lvert h(t)\rvert e^{-\omega t} = \max\big(\lvert h(0)\rvert,\ \lvert h(t_{\text{ext}})\rvert e^{-\omega t_{\text{ext}}}\big).
$$
(The degenerate case $Q=0$ — a constant $h$ — is not excluded in spirit: it simply means
$t_{\text{ext}}$ recedes to $\mp\infty$, so $h(t)e^{-\omega t}$ is trivially monotonic on $[0,\infty)$
and $\lvert h(0)\rvert$ is the sup, consistent with the $t_{\text{ext}}<0$ branch.) $\blacksquare$

Collecting the $A, v_0$ coefficients of $x''(t)$ along $\hat e$ gives
$$
h(t) = Qt+P, \qquad Q = \omega^2(\omega\varepsilon c + a), \qquad P = -\omega(\omega\varepsilon c+2a) = x''(0)
\ \text{(along }\hat e\text{)},
$$
matching the header's $x''(0)=-(\omega^2A+2\omega v_0)$. Writing $B=a+\omega\varepsilon c$ (i.e. $B$
along $\hat e$) and $k:=\omega A/B$, a direct substitution shows Lemma C's $t_{\text{ext}}$ works out
to exactly $t^*=3/\omega-A/B$ from the header note, and

$$
h(t_{\text{ext}})\,e^{-\omega t_{\text{ext}}} = \frac{Q}{\omega}\,e^{\omega P/Q - 1} = \omega B\,e^{k-3} = x''(t^*),
$$

reproducing the header's $x''(t^*)=\omega B\,e^{\omega A/B-3}$ exactly (both this identity and
$t_{\text{ext}}=t^*$ hold for **either** sign of $\varepsilon$ — Lemma C never needed a case split on
$\varepsilon$ in the first place, which is what makes this route shorter than repeating the $V$-limit
proof's $\varepsilon=\pm1$ analysis verbatim).

### When does the interior point dominate?

By Lemma C, $\lVert x''(t)\rVert \le \max(\lvert x''(0)\rvert, \lvert x''(t^*)\rvert)$ whenever
$t^*\ge0$ (i.e. $k\le3$), and $\lVert x''(t)\rVert\le\lvert x''(0)\rvert$ outright when $t^*<0$
($k>3$). Their ratio, using $v_0 = B(1-k)$ (from $k=\omega A/B$ and $B=v_0+\omega A$, so
$x''(0)=-\omega(B+v_0)=-\omega B(2-k)$):

$$
\frac{x''(0)}{x''(t^*)} \;=\; \frac{-\omega B(2-k)}{\omega B\,e^{k-3}} \;=\; (k-2)\,e^{3-k} \;=:\; h(k).
$$

$h(k)$ is strictly **increasing** throughout $k<3$ (since $h'(k)=e^{3-k}(3-k)>0$ there), with
$h(2)=0$ and $h(3)=1$. Consequently $\lvert h(k)\rvert$ *decreases* from $+\infty$ down to $0$ as $k$
increases from $-\infty$ to $2$, then *increases* from $0$ back up to $1$ as $k$ goes from $2$ to $3$
— a "V" shape with its minimum at $k=2$. So $\lvert h(k)\rvert=1$ has exactly one solution besides
$k=3$, at some $k_{\text{th}}<2$:

**Key Lemma (threshold).** $\lvert h(k)\rvert=1$ for $k<2$ iff $k = 2-W(1/e)$, where $W$ is the
Lambert $W$ function.

*Proof.* For $k<2$, $\lvert h(k)\rvert = (2-k)e^{3-k}$. Substitute $w=2-k>0$:
$(2-k)e^{3-k} = w\,e^{1+w}$. Setting this to $1$: $w\,e^w = e^{-1}$, i.e. $w=W(e^{-1})$, so
$k=2-W(1/e)$. $\blacksquare$

Numerically $k_{\text{th}} := 2-W(1/e) \approx 1.721535457$. Since $\lvert h(k)\rvert$ decreases on
$(-\infty,2)$ and increases on $(2,3)$ (shown above), $\lvert h(k)\rvert<1$ — the interior term
dominates — **exactly** for $k\in(k_{\text{th}},3)$, and $\lvert h(k)\rvert\ge1$ (boundary dominates
or, at the two endpoints, ties) everywhere else with $k<3$; for $k\ge3$ there is no interior critical
point inside the domain at all, so $\lvert x''(0)\rvert$ is trivially the whole story.

### Combined invariant

Putting the collinear reduction (Step 1) together with Lemma C and the threshold above:

$$
\lVert x''(t)\rVert \;\le\; \max\big\{\lVert x''(0)\rVert,\ \lvert x''(t^*)\rvert\big\} \quad\text{for all } t\ge0,
$$

with the second term inside the max only ever exceeding the first for
$k=\omega A/B \in (k_{\text{th}}, 3)$, $k_{\text{th}}=2-W(1/e)\approx1.721535457$. This is the claimed
invariant. $\blacksquare$

## Remarks

### Tightness
Unlike the $V$-limit's $(\lVert v_0\rVert+\omega\lVert A\rVert)/e$ bound (itself already a loose-by-a-
controlled-margin estimate — see that section's own Tightness remark), this acceleration bound is
**exact**: the collinear reduction is loss-free (Step 1's convexity argument), and Lemma C computes
the true suprema $\lvert x''(0)\rvert,\ \lvert x''(t^*)\rvert$ rather than an over-approximation of
them.

### Intuition
* Because only pose and twist — not accel — are matched at replan time (this is exactly what "C1"
  means), $x''(0)$ is a **free** consequence of whatever $\omega$ the velocity limit ends up choosing;
  nothing forces it to be small.
* $\lVert x''(0)\rVert = \omega^2\lVert A\rVert + 2\omega\lVert v_0\rVert$ in the worst case, so once
  $\omega$ is pinned at the velocity limit's own $\omega_{\min}$ floor (which happens for a large,
  uncompensated offset — see the "$v_0$ already exceeds $v_{\max}$" regime in the $V$-limit
  implementation relation above), the $t=0$ acceleration jump grows **without bound** as the offset
  grows, since $\omega_{\min}$ no longer shrinks to compensate. This is exactly the gap
  `determine_acc_omega()` exists to close.
* The interior term only ever wins in the narrow $k\in(k_{\text{th}},3)$ window: physically, this is
  the regime where the initial velocity is *already* directed toward the goal at close to (but not
  exceeding) the "natural" rate $\omega\lVert A\rVert$ the trajectory itself would pick — so
  $x''(0)\approx0$ (little correction needed at the very start) while the trajectory still has to
  decelerate later, producing the interior bump $x''(t^*)$ instead.

## Implementation relation

`determine_acc_omega()` does **not** invert the tight invariant above directly. Doing so exactly would
require knowing the *signed* relationship between $v_0$ and $A$ (i.e. which collinear case applies),
but the function only ever receives $\lVert v_0\rVert$ and $\lVert A\rVert$ as unsigned magnitudes —
mirroring `determine_vel_omega()`'s own `v_max, v_zero, a` signature (there too, `a` denotes
$\lVert A\rVert$ — *not* the $a := \lVert v_0\rVert$ shorthand from the $V$-limit proof's Setup above;
see the naming note at the end of this section). It also deliberately **ignores the $k_{\text{th}}$
threshold** derived above: rather than gating the interior term on whether $k\in(k_{\text{th}},3)$, it
bounds *both* candidate peaks unconditionally, using two independent worst-case sign assumptions, one
per term.

**Boundary term** ($\lvert x''(0)\rvert \le a_{\max}$): solved using the worst-case *diverging*
alignment ($\varepsilon=+1$, which maximizes $\lvert x''(0)\rvert$ for given magnitudes), by mirroring
`determine_vel_omega()`'s own structure of comparing the (division-free) constraint at the range's
endpoints first, rather than solving and clamping afterward:

$$
f(\omega) := \omega^2\lVert A\rVert + 2\omega\lVert v_0\rVert \qquad \text{(non-decreasing in $\omega$, since $\lVert A\rVert,\lVert v_0\rVert\ge0$)}
$$

$$
\omega_{\text{zero}} =
\begin{cases}
  \omega_{\min} &:\quad f(\omega_{\min}) \ge a_{\max} \quad\text{(already violated at the bottom of the range -- best effort)}\\
  \omega_{\max} &:\quad f(\omega_{\max}) < a_{\max} \quad\text{(unconstrained even at the top of the range)}\\
  \dfrac{a_{\max}}{\sqrt{\lVert v_0\rVert^2+\lVert A\rVert\, a_{\max}}+\lVert v_0\rVert} &:\quad \text{else}
\end{cases}
$$

The $\omega_{\min}$ case is checked **first**, so an exact tie — both endpoint conditions holding at
once, e.g. $\lVert A\rVert=\lVert v_0\rVert=a_{\max}=0$ — resolves to the safer, more restrictive
$\omega_{\min}$ rather than the permissive $\omega_{\max}$.

The `else` branch is the *rationalized* form of the direct quadratic root
$\big({-}\lVert v_0\rVert+\sqrt{\lVert v_0\rVert^2+\lVert A\rVert\, a_{\max}}\big)/\lVert A\rVert$:
multiplying numerator and denominator by the conjugate
$\sqrt{\lVert v_0\rVert^2+\lVert A\rVert\, a_{\max}}+\lVert v_0\rVert$ cancels the $\lVert A\rVert$ in
the denominator against a matching factor in the numerator, leaving no division by $\lVert A\rVert$ at
all. This is both more numerically stable (no subtracting two close values when
$\lVert A\rVert a_{\max} \ll \lVert v_0\rVert^2$) and, unlike the direct form, well-defined as
$\lVert A\rVert\to0$ — it reduces exactly to $a_{\max}/(2\lVert v_0\rVert)$, the correct answer to the
then-linear constraint — so no separate $\lVert A\rVert=0$ branch is needed at all.

**Interior term**: bounded directly against $a_{\max}$ using the already-established approximation
$\lvert x''(t^*)\rvert \approx (\omega/e)\,v_{\max}$ (since the velocity limiter keeps
$\lvert x'(t_{a0})\rvert\approx v_{\max}$ — the $V$-limit invariant above), **applied
unconditionally** rather than gated on $k\in(k_{\text{th}},3)$ as the tight invariant above would allow:

$$
\omega_{\text{interior}} =
\begin{cases}
  \omega_{\min} &:\quad \omega_{\min}\,v_{\max} \ge e\,a_{\max} \\
  \omega_{\max} &:\quad \omega_{\max}\,v_{\max} < e\,a_{\max} \\
  e\,a_{\max}/v_{\max} &:\quad \text{else}
\end{cases}
$$

(rearranged as $\omega\,v_{\max} \le e\,a_{\max}$ — multiplying through by $v_{\max}\ge0$ — to sidestep
$v_{\max}=0$ the same way the boundary term sidesteps $\lVert A\rVert=0$, rather than special-casing
it), with $\omega_{\min}$ again checked first for the same tie-break reason.

**Why unconditional is safe, just not tight**: outside the window $(k_{\text{th}},3)$, the tight
invariant above already guarantees $\lvert x''(0)\rvert \ge \lvert x''(t^*)\rvert$, so additionally
requiring $\omega\le\omega_{\text{interior}}$ there can only make the result *more* conservative than
necessary — never unsafe. Skipping the $k$-window check entirely — rather than computing a trial $k$
from a trial $\omega$ and branching on it — trades a small amount of unnecessary conservatism outside
that narrow window for a simpler function with fewer branches and no dependency between the two terms.
This is the same "conservative rather than exact" trade-off as using two different worst-case sign
assumptions in the first place: it does not reproduce the tight collinear invariant above bit-for-bit,
but guarantees a safe bound regardless of which alignment (or which side of $k_{\text{th}}$) the true
configuration actually falls on — the same spirit as the C2 section below's own choice of two simpler,
independently-invertible proxy constraints over its exact quadratic characterization.

**Combining**:

$$
\omega = \min(\omega_{\text{zero}},\ \omega_{\text{interior}})
$$

clamped once more into $[\omega_{\min},\omega_{\max}]$ as cheap insurance against floating-point edge
cases at the boundaries (both branches above are already constructed to land inside that range by
themselves). As with the $V$-limit, separate linear and angular estimates are computed this way,
combined with the velocity-based $\omega$ via a further $\min(\cdot,\cdot)$ in the caller, and the
whole result is clamped to $[\omega_{\min},\omega_{\max}]$ once more there too.

> **Naming note.** The code's parameter for $\lVert A\rVert$ (the offset magnitude) is the bare
> identifier `a` — used, consistently across `determine_vel_omega()`, `determine_acc_omega()`, and
> C2's `determine_omega()` overload, as the lowercase scalar counterpart of the vector member `A_`.
> This is easy to misread against this document's *unrelated* convention of `a := \lVert v_0\rVert`
> (used only within the $V$-limit and $A$-limit proofs' Setup sections above) — the two conventions
> never appear together in this document's prose, but the code itself passes both `a_max` and `a` as
> arguments to the very same function, which is worth keeping in mind when cross-referencing the two.

---

# C2 (Accel-continuity) variant

This section covers the additive-patch construction implemented in
[`kinematic_trajectory_exponential_approach_C2.hpp`](kinematic_trajectory_exponential_approach_C2.hpp): rather
than a separate triple-pole polynomial trajectory, C2 holds a private C1 (Twist-continuity) instance `c1_` and
adds a correction term on top of it that brings the initial acceleration up to the requested $a_0$ exactly,
without disturbing $x_{C1}$'s own pose/twist match at $t=0$.

## Definitions

$$
x_{C2}(t) = x_{C1}(t) + h(t), \qquad h(t) = D\,t^2\,e^{-\omega_a t}
$$

$x_{C1}(t)$ is exactly the C1 trajectory proved above, evaluated at its own convergence rate $\omega_{pv}$
(renamed from the C1 section's bare $\omega$ purely to disambiguate it here from the patch's own, independent
rate $\omega_a$):

$$
x_{C1}(t) = \text{goal} + (A+Bt)\,e^{-\omega_{pv}t}, \qquad A = x_0-\text{goal}, \qquad B = v_0+\omega_{pv}A
$$

$D$ is half the *residual* acceleration $a_1$ — whatever $x_{C1}$'s own $t=0$ curvature doesn't already supply:

$$
P_1 := x_{C1}''(0) = -\omega_{pv}^2A - 2\omega_{pv}v_0, \qquad a_1 := a_0 - P_1, \qquad D := \frac{a_1}{2}
$$

matching `calculate()`'s own `p1`, `a1`, `D_` exactly. Differentiating $x_{C1}$ (already proved above) and $h$
separately and adding gives the code's own derivative formulas:

$$
x_{C2}'(t) = x_{C1}'(t) + D\,t(2-\omega_a t)\,e^{-\omega_a t}, \qquad
x_{C2}''(t) = x_{C1}''(t) + D\,(2-4\omega_a t+\omega_a^2t^2)\,e^{-\omega_a t}
$$

`goal`, `x_0`, `v_0`, `a_0`, `A`, `B`, `P_1`, `a_1`, `D` are vectors; $\omega_{pv},\omega_a>0,\ t\ge0$ are
scalars; $\lVert\cdot\rVert$ again denotes the Euclidean norm.

An identity used repeatedly below: substituting $B=v_0+\omega_{pv}A$ into
$x_{C1}''(t)=\omega_{pv}\big(\omega_{pv}(A+Bt)-2B\big)e^{-\omega_{pv}t}$ shows that $x_{C1}''$ is *itself* a
linear-times-exponential of exactly the shape the C1 V-Limit proof's Lemma A/B were built for:

$$
x_{C1}''(t) = (P_1+Q_1t)\,e^{-\omega_{pv}t}, \qquad Q_1 := \omega_{pv}^2v_0+\omega_{pv}^3A
$$

matching the header comment's $Q_1$ (never computed numerically in code — `determine_acc_omega_pv()` below only
ever needs a closed-form *bound* on it, not its value).

### Boundary conditions

> **Claim.** $x_{C2}(0)=x_0,\quad x_{C2}'(0)=v_0,\quad x_{C2}''(0)=a_0$, for **any** $\omega_a>0$.

*Proof.* $h(t)=Dt^2e^{-\omega_at}$ has a double zero at $t=0$: $h(0)=0$, and
$h'(t)=Dt(2-\omega_at)e^{-\omega_at}$ gives $h'(0)=0$ too. So $x_{C2}(0)=x_{C1}(0)=x_0$ and
$x_{C2}'(0)=x_{C1}'(0)=v_0$, both already established in the C1 section. For the acceleration,
$h''(0)=D\cdot(2-0+0)\cdot1=2D=a_1=a_0-P_1$, so $x_{C2}''(0)=x_{C1}''(0)+h''(0)=P_1+(a_0-P_1)=a_0$ — exactly,
and independent of $\omega_a$, since every $\omega_a$-bearing term of $h''$ is killed by the $t=0$ evaluation
regardless of $\omega_a$'s actual value. $\blacksquare$

This is precisely what lets `calculate()` solve for $\omega_{pv}$ and then $\omega_a$ **independently**, in that
order: $\omega_{pv}$ alone fixes $A,B,P_1$ (hence $a_1,D$), and $\omega_a$ only ever controls *how fast* $h$
decays, never the exactness of the $t=0$ match.

---

## Lemma D — exact peaks of the patch term

Because $h,h',h''$ are each a **fixed vector** $D$ times a **scalar** function of $t$,
$\lVert h^{(n)}(t)\rVert = \lvert(\text{scalar factor})\rvert\cdot\lVert D\rVert$ exactly for every $t$ — no
collinear worst-case reduction (à la the C1 proofs' Step 1) is needed here, since only one vector direction is
ever involved.

> **Lemma D.** For $D\ne0,\ \omega_a>0$, writing $u=\omega_at$ and $\kappa:=(\sqrt2-1)e^{\sqrt2-2}\approx0.2306$:
> $$
> \text{(a)}\ \sup_{t\ge0}\lVert h(t)\rVert = \frac{4\lVert D\rVert}{\omega_a^2e^2} = \frac{2\lVert a_1\rVert}{\omega_a^2e^2}, \text{ at } t=\tfrac2{\omega_a}
> \qquad
> \text{(b)}\ \sup_{t\ge0}\lVert h'(t)\rVert = \frac{2\kappa\lVert D\rVert}{\omega_a} = \frac{\kappa\lVert a_1\rVert}{\omega_a}, \text{ at } t=\tfrac{2-\sqrt2}{\omega_a}
> $$
> $$
> \text{(c)}\ \sup_{t\ge0}\lVert h''(t)\rVert = 2\lVert D\rVert = \lVert a_1\rVert, \text{ at } t=0
> $$

*Proof.*

**(a)** $h(t)=Dt^2e^{-\omega_at}=(D/\omega_a^2)\,u^2e^{-u}$. $\frac{d}{du}u^2e^{-u}=(2u-u^2)e^{-u}$ vanishes at
$u=2$ (sign change $+\to-$, a maximum; $u^2e^{-u}\to0$ at both $u=0$ and $u\to\infty$), with value $4/e^2$.
Hence $\sup_t\lVert h(t)\rVert=\lVert D\rVert\cdot4/(\omega_a^2e^2)$.

**(b)** $h'(t)=Dt(2-\omega_at)e^{-\omega_at}=(D/\omega_a)\,u(2-u)e^{-u}$. Let $g_1(u):=(2u-u^2)e^{-u}$;
$g_1'(u)=e^{-u}(u^2-4u+2)$, vanishing at $u=2\mp\sqrt2$ (roots of $u^2-4u+2=0$). At $u=2-\sqrt2\in(0,2)$ (a
maximum, since $u^2-4u+2$ changes sign $+\to-$ there):
$g_1(2-\sqrt2)=(2-\sqrt2)\sqrt2\,e^{-(2-\sqrt2)}=2(\sqrt2-1)e^{\sqrt2-2}=2\kappa$. At $u=2+\sqrt2$ (a minimum):
$g_1(2+\sqrt2)=-(2+\sqrt2)\sqrt2\,e^{-(2+\sqrt2)}=-2(\sqrt2+1)e^{-(2+\sqrt2)}$, with
$$
\frac{\lvert g_1(2+\sqrt2)\rvert}{g_1(2-\sqrt2)} = \frac{\sqrt2+1}{\sqrt2-1}\,e^{-2\sqrt2} = (3+2\sqrt2)\,e^{-2\sqrt2}\approx5.83\times0.059\approx0.35 < 1,
$$
so the $u=2-\sqrt2$ maximum dominates; since $g_1\to0$ at both $u=0$ and $u\to\infty$, these two critical points
are the only candidates, giving $\sup_{u\ge0}\lvert g_1(u)\rvert=2\kappa$. Hence
$\sup_t\lVert h'(t)\rVert=\lVert D\rVert\cdot2\kappa/\omega_a$.

**(c)** $h''(t)=D(2-4\omega_at+\omega_a^2t^2)e^{-\omega_at}=D\,g_2(u)$, $g_2(u):=(u^2-4u+2)e^{-u}$, so
$g_2(0)=2$. $g_2'(u)=e^{-u}(-u^2+6u-6)$, vanishing at $u=3\mp\sqrt3$. At $u=3-\sqrt3\approx1.27$:
$g_2=(2-2\sqrt3)e^{-(3-\sqrt3)}\approx-0.41$. At $u=3+\sqrt3\approx4.73$: $g_2=(2+2\sqrt3)e^{-(3+\sqrt3)}\approx0.05$.
Both are smaller in magnitude than $g_2(0)=2$, and $g_2\to0$ as $u\to\infty$, so $\sup_{u\ge0}\lvert g_2(u)\rvert=2$
is attained at the **boundary** $u=0$ — unlike (a) and (b), not at an interior point. Hence
$\sup_t\lVert h''(t)\rVert=2\lVert D\rVert=\lVert a_1\rVert$ (using $D=a_1/2$), attained at $t=0$, where it
trivially equals $a_1$ by the very construction of $D$. $\blacksquare$

Part (c) is the structural fact behind `determine_acc_omega_a()` below: **no** choice of $\omega_a$ ever changes
$h''$'s peak, only *where in $t$* it occurs — $\omega_a$ has no lever at all on the acceleration side, only on
the velocity (b) and position (a) sides, where it appears in the denominator (with (a) shrinking faster, as
$1/\omega_a^2$, than (b)'s $1/\omega_a$).

---

## Bound on $\omega_{pv}$ — `determine_omega_pv()`

$\omega_{pv}$ is solved **first** (Step 2 of `calculate()`, before $\omega_a$ exists at all), combining a
velocity ceiling (reusing C1's own bound unchanged) and a new acceleration ceiling that additionally accounts
for the patch's own contribution to $x_{C2}''$. Both ceilings must therefore hold for *any* subsequent
$\omega_a$ — made possible below by Lemma D(c)'s $\omega_a$-independence.

### Velocity ceiling — `determine_vel_omega_pv()`

This delegates directly to `c1_.determine_vel_omega()`, i.e. reuses the C1 section's **V-Limit invariant**
unchanged, applied to the $x_{C1}'$ term alone:

$$
\lVert x_{C1}'(t)\rVert \le \max\Big\{\lVert v_0\rVert,\ \frac{\lVert v_0\rVert+\omega_{pv}\lVert A\rVert}{e}\Big\}
$$

This bounds only the $x_{C1}'$ term of $x_{C2}'(t)=x_{C1}'(t)+h'(t)$, not the full sum — see the Remarks below
for why the $h'(t)$ remainder is deliberately left for $\omega_a$ to shrink, rather than folded into this
ceiling.

### Acceleration ceiling — `determine_acc_omega_pv()`

> **Claim.** For all $t\ge0$,
> $$
> \lVert x_{C2}''(t)\rVert \;\le\; \Big(2+\frac1e\Big)\omega_{pv}^2\lVert A\rVert \;+\; \Big(4+\frac1e\Big)\omega_{pv}\lVert v_0\rVert \;+\; \lVert a_0\rVert
> $$

This is exactly the bound `determine_acc_omega_pv()` inverts (setting the right-hand side $=a_{\max}$ and
solving for the smallest non-negative $\omega_{pv}$ satisfying the resulting quadratic).

**Lemma E** (exponential envelope). For $x\in[0,1]$: $\;e^{x-1}\le x+\tfrac1e$.

*Proof.* Let $\phi(x)=x+\tfrac1e-e^{x-1}$, strictly concave since $\phi''(x)=-e^{x-1}<0$. $\phi(0)=\tfrac1e-e^{-1}=0$
and $\phi(1)=1+\tfrac1e-1=\tfrac1e>0$. A concave function lies above the chord joining any two of its points, so
on $[0,1]$, $\phi(x)\ge x\cdot\phi(1)=x/e\ge0$. $\blacksquare$

**Proposition** (bound on $x_{C1}''$).
$$
\sup_{t\ge0}\lVert x_{C1}''(t)\rVert \;\le\; \Big(1+\frac1e\Big)\omega_{pv}^2\lVert A\rVert + \Big(2+\frac1e\Big)\omega_{pv}\lVert v_0\rVert.
$$

*Proof.* From the identity in Definitions, $x_{C1}''(t)=(P_1+Q_1t)e^{-\omega_{pv}t}$, so by the vector triangle
inequality (valid unconditionally, for any relative orientation of $P_1,Q_1$ — no collinear reduction needed):
$$
\lVert x_{C1}''(t)\rVert \le (\lVert P_1\rVert+\lVert Q_1\rVert t)\,e^{-\omega_{pv}t}, \qquad \text{so} \qquad
\sup_t\lVert x_{C1}''(t)\rVert \le \max_{t\ge0}(p+qt)e^{-\omega_{pv}t}, \quad p=\lVert P_1\rVert,\ q=\lVert Q_1\rVert.
$$
By **Lemma A** (C1 section), this max equals $p$ if $q\le\omega_{pv}p$, or $\tfrac{q}{\omega_{pv}}e^{\omega_{pv}p/q-1}$
otherwise. In the latter case, write $x=\omega_{pv}p/q\in[0,1)$ and apply **Lemma E**:
$$
\frac{q}{\omega_{pv}}e^{x-1} \le \frac{q}{\omega_{pv}}\Big(x+\frac1e\Big) = p+\frac{q}{\omega_{pv}e}.
$$
In the other case ($q\le\omega_{pv}p$), $p\le p+q/(\omega_{pv}e)$ trivially. Either way,
$$
\max_{t\ge0}(p+qt)e^{-\omega_{pv}t} \le \lVert P_1\rVert + \frac{\lVert Q_1\rVert}{\omega_{pv}e}.
$$
Now bound $P_1,Q_1$ by the triangle inequality:
$\lVert P_1\rVert=\lVert{-\omega_{pv}^2A-2\omega_{pv}v_0}\rVert\le\omega_{pv}^2\lVert A\rVert+2\omega_{pv}\lVert v_0\rVert$,
and $\lVert Q_1\rVert=\lVert\omega_{pv}^2v_0+\omega_{pv}^3A\rVert\le\omega_{pv}^2\lVert v_0\rVert+\omega_{pv}^3\lVert A\rVert$.
Substituting,
$$
\sup_t\lVert x_{C1}''(t)\rVert \le \omega_{pv}^2\lVert A\rVert+2\omega_{pv}\lVert v_0\rVert + \frac{\omega_{pv}^2\lVert v_0\rVert+\omega_{pv}^3\lVert A\rVert}{\omega_{pv}e} = \Big(1+\frac1e\Big)\omega_{pv}^2\lVert A\rVert+\Big(2+\frac1e\Big)\omega_{pv}\lVert v_0\rVert. \quad\blacksquare
$$

*Proof of the Claim.* By the triangle inequality, $\lVert x_{C2}''(t)\rVert\le\lVert x_{C1}''(t)\rVert+\lVert h''(t)\rVert$
for every $t$, so it suffices to bound each term's own supremum and add:
$$
\sup_t\lVert x_{C1}''(t)\rVert \le \Big(1+\frac1e\Big)\omega_{pv}^2\lVert A\rVert+\Big(2+\frac1e\Big)\omega_{pv}\lVert v_0\rVert \qquad\text{(Proposition above)}
$$
$$
\sup_t\lVert h''(t)\rVert = \lVert a_1\rVert \le \lVert a_0\rVert+\lVert P_1\rVert \le \lVert a_0\rVert+\omega_{pv}^2\lVert A\rVert+2\omega_{pv}\lVert v_0\rVert \qquad\text{(Lemma D(c), exact; then triangle ineq. twice)}
$$
Adding the two bounds (each holds for the corresponding sup separately, so their sum bounds
$\sup_t\lVert x_{C2}''(t)\rVert$ directly):
$$
\sup_t\lVert x_{C2}''(t)\rVert \le \Big(2+\frac1e\Big)\omega_{pv}^2\lVert A\rVert + \Big(4+\frac1e\Big)\omega_{pv}\lVert v_0\rVert + \lVert a_0\rVert. \qquad\blacksquare
$$

Crucially, this derivation never needed $\omega_a$ — Lemma D(c) is exact and $\omega_a$-independent — which is
exactly what makes it sound to fix $\omega_{pv}$ *before* $\omega_a$ is even chosen.

#### Implementation relation

Writing $c_2=\lVert A\rVert(2+\tfrac1e),\ c_1=\lVert v_0\rVert(4+\tfrac1e),\ c_0=\lVert a_0\rVert-a_{\max}$, the
Claim's right-hand side $\le a_{\max}$ is exactly the quadratic inequality
$$
c_2\,\omega_{pv}^2+c_1\,\omega_{pv}+c_0\le0
$$
solved by `determine_acc_omega_pv()`. Since $c_2,c_1\ge0$ (norms are non-negative), the left-hand side is
non-decreasing in $\omega_{pv}\ge0$, so — exactly as with C1's own `determine_acc_omega()` — the range endpoints
are checked first ($\omega_{\min}$ first, for the same "safer on an exact tie" reason), and the quadratic is
solved only in the remaining, strictly monotonic in-between case: there $c_0<0$ (else the $\omega_{\min}$ branch
above would already have fired) with $c_2,c_1\ge0$, so exactly one non-negative root exists, at
$$
\omega^* = \frac{-c_1+\sqrt{c_1^2-4c_2c_0}}{2c_2},
$$
rationalized (multiplying by the conjugate $-c_1-\sqrt{c_1^2-4c_2c_0}$ over itself) to
$$
\omega^* = \frac{-2c_0}{c_1+\sqrt{c_1^2-4c_2c_0}},
$$
which avoids both the division-by-near-zero-$c_2$ instability of the direct form and a separate $c_2=0$ branch
(as $c_2\to0$, this reduces cleanly to $-c_0/c_1$, the correct root of the then-linear equation) — the identical
rationalization trick C1's own `determine_acc_omega()` already uses.

---

## Bound on $\omega_a$ — `determine_omega_a()`

$\omega_a$ is solved **second** (Step 4 of `calculate()`), downstream of the now-fixed $\omega_{pv}$ and the
resulting $a_1$ — governing only how fast the patch $h(t)$ itself decays.

### Velocity ceiling — `determine_vel_omega_a()`

By Lemma D(b), $\sup_t\lVert h'(t)\rVert=\kappa\lVert a_1\rVert/\omega_a$ **exactly**, strictly *decreasing* in
$\omega_a$ (since $a_1$ — hence $D$ — does not depend on $\omega_a$ at all). Consequently:

* If this axis' component of $a_1$ is $\le0$ (no residual to correct), $D=a_1/2=0$ on that axis, so $h\equiv0$
  there and *no* choice of $\omega_a$ matters — the function returns $\omega_{\min}$, a deliberately
  non-binding placeholder (see the axis-combination discussion below).
* Otherwise, since the bound is strictly decreasing in $\omega_a$, the *largest* available rate $\omega_{\max}$
  minimizes it, with no smaller $\omega_a$ ever doing better. By Lemma D(a), the *position* residual
  $\sup_t\lVert h(t)\rVert=2\lVert a_1\rVert/(\omega_a^2e^2)$ shrinks even faster under the same choice (as
  $1/\omega_a^2$ rather than $1/\omega_a$), so maximizing $\omega_a$ is simultaneously optimal for both the
  velocity and the position side of the patch. The function returns $\omega_{\max}$ in this case.

**Honesty note.** Unlike every other $\omega$-ceiling in this document, this one is not the inverse of a *tight
target* (there is no $v_{\max}$-style threshold being solved for): it is a monotone-optimal choice given Lemma
D(b)'s shape, not a guarantee that $\lVert h'(t)\rVert$ stays under any particular numeric margin. Combining it
with the C1 V-Limit bound via the triangle inequality gives the honest combined statement
$$
\lVert x_{C2}'(t)\rVert \le \underbrace{\max\Big\{\lVert v_0\rVert,\ \frac{\lVert v_0\rVert+\omega_{pv}\lVert A\rVert}{e}\Big\}}_{\text{C1's V-Limit bound, at }\omega_{pv}} \;+\; \underbrace{\frac{\kappa\lVert a_1\rVert}{\omega_a}}_{\text{Lemma D(b), exact}}
$$
which is minimized, given $\omega_{pv}$ already fixed, by pushing $\omega_a$ as high as `omega_max()` allows —
exactly what the function does — but which is not itself clamped against any $v_{\max}$.

### Acceleration ceiling — `determine_acc_omega_a()`

By Lemma D(c), $\sup_t\lVert h''(t)\rVert=\lVert a_1\rVert$ for **every** $\omega_a>0$: the peak is attained at
$t=0$, where it equals $a_1$ by the boundary-condition construction itself, and this value simply does not
depend on $\omega_a$. There is therefore no $\omega_a$ this function could return to bring an over-limit
$\lVert a_1\rVert$ back under $a_{\max}$ — reducing that peak requires reshaping $a_1$ itself (i.e. $\omega_{pv}$,
$A$, or $a_0$, which the previous section's acceleration ceiling on $\omega_{pv}$ is responsible for), not
$\omega_a$. The function accordingly always returns the non-binding $\omega_{\max}$.

### Combining axes and terms

`determine_omega_a_component()` takes $\min$ of the two ceilings above (mirroring
`determine_omega_pv_component()`'s structure), trivially collapsing to the velocity ceiling alone since the
acceleration ceiling is always $\omega_{\max}$. `determine_omega_a()` then combines the linear and angular
axes' candidates via $\max$ — not $\min$, unlike $\omega_{pv}$'s combination — because $\omega_a$ is a single
*shared* rate applied to both axes' residuals simultaneously: each axis independently only ever asks for either
$\omega_{\min}$ (no residual) or $\omega_{\max}$ (residual present, strictly the better choice per the velocity
ceiling above), so the shared rate must be $\omega_{\max}$ whenever *either* axis has a residual — exactly what
$\max(\cdot,\cdot)$ produces, since an $\omega_{\min}$ candidate from a residual-free axis can never outrank a
genuine $\omega_{\max}$ need from the other.

---

## Remarks

### Ordering and independence

$\omega_{pv}$ is solved using only $v_0,A,a_0$ — it never needs $\omega_a$ to exist. $\omega_a$ is solved
second, using $\omega_{pv}$'s result (through $A,B,P_1,a_1$) but never needing to revisit $\omega_{pv}$. This
one-directional dependency is exactly what makes the two-stage `calculate()` correct: the acceleration-ceiling
Claim on $\omega_{pv}$ above holds for *any* subsequently chosen $\omega_a>0$ (Lemma D(c)'s $\omega_a$-independence),
so nothing computed in Step 2 needs to be revisited once Step 4 picks a concrete $\omega_a$.

### Tightness

The $\omega_{pv}$ acceleration bound above is intentionally loose in three separate places — (i) Lemma A's
two-branch max is bounded rather than solved exactly, via Lemma E; (ii) $P_1,Q_1$ are bounded by the triangle
inequality rather than by their own worst-case collinear analysis (as the C1 A-Limit section did for $x''$
alone); (iii) $\lVert a_1\rVert$ is bounded via $\lVert a_0\rVert+\lVert P_1\rVert$ rather than computed exactly.
Each loosening keeps the final inversion closed-form (one quadratic in $\omega_{pv}$) at the cost of
conservatism — the same trade-off C1's own `determine_acc_omega()` already makes. By contrast, Lemma D itself is
**exact**: each of its three suprema is the true peak of a scalar-times-fixed-vector function, not an
over-approximation.

### Intuition

* The patch $h(t)$ is a pure *correction* term: it and its own velocity are exactly zero at $t=0$, contributing
  nothing to the pose/twist match, and it exists solely to make up whatever acceleration $x_{C1}$ alone doesn't
  already supply.
* $a_1=a_0-P_1$ can be small even when $a_0$ is large, if $x_{C1}$'s own $t=0$ jump $P_1$ already happens to
  point the same way — conversely $a_1$ can *exceed* $a_0$ in magnitude if $P_1$ points the opposite way, which
  is exactly why the acceleration ceiling on $\omega_{pv}$ bounds $\lVert a_1\rVert$ via
  $\lVert a_0\rVert+\lVert P_1\rVert$ rather than assuming $\lVert a_1\rVert\le\lVert a_0\rVert$.
* Because $\omega_a$ has no lever on the acceleration side (Lemma D(c)) but a strictly beneficial one on both
  the velocity and position sides (Lemma D(a),(b)), the optimal policy is always "push $\omega_a$ as high as
  `omega_max()` allows" — there is no interior trade-off to solve for, unlike every $\omega_{pv}$/C1 ceiling
  above. This is exactly why `determine_vel_omega_a()`'s logic collapses to a two-way choice between
  $\omega_{\min}$ (non-binding) and $\omega_{\max}$, rather than an inverted closed-form threshold.
