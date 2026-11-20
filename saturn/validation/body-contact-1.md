# Validation record — 2026-11-19 (body contact, body-contact-1)

`sh2zam/body_contact_experimental.h` ([guide](../BODY_CONTACT.md)) resolves contact between bodies
that touch as spheres: impact damage shared by weight, a push while the pair closes, attackers,
and the credit of the last credited body to start a contact. It is integer only and promises
identical host and SH-2 bits. **No console run was done in this repository.** Evidence by tier:

- **HOST:** `body_contact_exact` passes 2,445,910 checks under GCC and Clang Release with
  `SH2Z_STRICT=ON` and under ASan/UBSan:
  - 400,000 random pairs (about half in reach, a quarter first contacts, a quarter pushed, 12%
    attack hits, and degenerate separations) against an independent double-precision transcription
    of the rule: worst damage error 18 Q16 LSB, worst push error 4.1 LSB, against the stated bound
    of 256 (2^-8). Every decision (in reach, first, closing, attack case) matches exactly;
  - edge cases: exactly at reach and one LSB beyond, in reach on every axis but not in distance,
    coincident bodies with and without relative motion, a separating pair, hand-worked numbers for
    equal weights and for a heavy attacker, and the shortest attack push;
  - the field walk against a brute-force walk over all 435 pairs of 30 bodies with its own history,
    40 races of 300 ticks on a short wrapping line: identical outputs every tick, with 454,619 pairs
    tested instead of 5.2 million;
  - contact history and credit on a scripted scene: one charge per contact, a second contact after
    separating, credit through tick 70 for a contact at tick 10 with a 60-tick window.

  Controls: the oracle with one rule changed must disagree with the library, and does, for each of
  impact gain 1.2, attack push 2.5, shares swapped, weak share 0.5, heavy gain 9, a push while
  separating, attack minimum 2 and push reach 45. A credit window of 61 changes the scene's answer.
- **SH-2 build:** the source compiles with the CI flags (`-m2 -mb -O2 -Werror`). The object is
  5,200 bytes. It needs `sh2z_divu_div64`, `sh2z_isqrt64_q12`, `sh2z_vec3_normalize_u30` and
  `___ashiftrt_r4_30`.
- **Not done:** an instruction-model capture, console execution and any frame measurement.
