# Proofs for `KinematicTrajectoryExponentialApproach`

This document collects the mathematical background for the exponential-approach
trajectory implemented in
[`kinematic_trajectory_exponential_approach.hpp`](kinematic_trajectory_exponential_approach.hpp).

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
