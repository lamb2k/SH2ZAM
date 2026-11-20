# Two-CPU try-claim (dual-claim-1, experimental)

`sh2zam/dual_claim_experimental.h` lets two processors share an optional job
so that exactly one of them runs it, without an atomic instruction. The SH-2
pair has no snooping caches and no lock the other CPU must honour, so the
claim is two plain words and a decision word.

## Use

A frame has one job that either CPU can do, for example a table prepare.
The master does its own work first and the job last. The slave takes the job
if its own work ends first. The output must not depend on which CPU ran it:
same inputs, same destination, and the master commits the result in both
cases.

```c
/* master, at the job */
if (sh2z_claim_a(&claim, frame)) prepare();      /* else wait for the slave's done word */
/* slave, when idle */
if (sh2z_claim_b(&claim, frame)) { prepare(); done = frame; }
```

## Contract

- **One owner.** For a tag, at most one of `sh2z_claim_a` and
  `sh2z_claim_b` returns true. If both sides claim it, exactly one does.
- **A may wait, B never waits.** A spins only when it sees B's claim on the
  same tag, until B's decision word names the tag. B writes it two
  operations after its claim. A wins only when B has seen A's claim.
- **Declining.** `sh2z_claim_b_decline` records that B will not take the tag,
  for a side that skips the job (for example past a deadline). A then owns
  the job without waiting. When A does not claim, B owns it.
- **Tags.** Nonzero, below 2^31, new for every job. Old words never match a
  new tag, so the record needs no reset.
- **Memory.** Both CPUs must access the record in program order with no
  stale copy: on the Saturn, place it in the cache-through window (libyaul
  `.uncached`). The job's own data is the caller's: read it through the
  cache-through window, or purge, after the owner reports completion.

## Tests

`tests/test_dual_claim.c` runs every interleaving of the two sides' memory
operations from a fresh record and from records left by older jobs: exactly
one owner and no deadlock on each path. It then checks the real functions in
each sequential order. It has four negative controls, each a broken protocol
that must give two owners:
- A owns without waiting;
- B ignores A;
- A loads before it stores;
- B loads before it stores.
